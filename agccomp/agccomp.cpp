#include "usermodfx.h"
#include <stdint.h>

// #############################################################################
// agccomp: オートゲインコントローラ (AGC) -> コンプレッサー -> リミッター
//
// 信号連鎖:
//   入力 -> AGCゲイン -> コンプレッサー -> リミッター -> 出力
//
// AGCが入力レベルを目標ラウドネス(約 -14 LUFS)へ自動調整するため、
// stcompにあったメークアップゲインは廃止している。
// #############################################################################

namespace {
constexpr float kSampleRate = 48000.0f;
constexpr float kDbToLog2 = 0.16609640474436813f;  // 1 / 6.020599913...

// --- AGC (オートゲインコントローラ) ---
constexpr float kAgcTargetLufs = -14.0f;      // 目標ラウドネス (約 -14 LUFS)
constexpr float kLufsOffset = 0.691f;          // BS.1770のラウドネスオフセット
constexpr float kAgcWindowSeconds = 0.4f;      // ラウドネス測定窓 (モーメンタリ相当)
constexpr float kAgcReactionMin = 1.0f;        // 反応時間の最小 (秒)
constexpr float kAgcReactionMax = 30.0f;       // 反応時間の最大 (秒)
constexpr float kAgcMaxGainDb = 40.0f;         // AGCゲイン上限 (+40 dB)
constexpr float kAgcMinGainDb = -24.0f;        // AGCゲイン下限 (-24 dB)
constexpr float kAgcGateDb = -45.0f;           // 無音判定スレッショルド (RMS)
constexpr uint32_t kAgcControlInterval = 32U;  // AGC目標ゲインの更新間隔 (サンプル)

// --- コンプレッサー (設定固定) ---
constexpr float kCompThresholdDb = -18.0f;     // 固定スレッショルド
constexpr float kRatioExponent = 0.75f;        // 4:1 のコンプレッションレシオ
constexpr float kAttackSeconds = 0.005f;
constexpr float kReleaseSeconds = 0.150f;
constexpr uint32_t kCompControlInterval = 8U;

// --- リミッター ---
constexpr float kLimiterReleaseSeconds = 0.050f;
constexpr float kLimiterCeilingDb = -1.0f;

// AGCの状態
float g_agc_target_rms = 0.216f;   // 目標RMS振幅 (約 -14 LUFS 相当)
float g_agc_ms = 0.0f;             // 平滑化した平均二乗値 (ラウドネス推定)
float g_agc_target_gain = 1.0f;    // 制御レートで更新する目標ゲイン
float g_agc_gain = 1.0f;           // 実際に適用するAGCゲイン
float g_agc_window_coeff = 0.0f;   // 測定窓の平滑化係数
float g_agc_reaction_coeff = 0.0f; // 反応時間の平滑化係数 (TIMEで変化)
float g_agc_max_gain = 100.0f;     // ゲイン上限 (線形)
float g_agc_min_gain = 0.063f;     // ゲイン下限 (線形)
float g_agc_gate_ms = 0.0f;        // 無音判定 (平均二乗値)
uint32_t g_agc_count = 0U;

// コンプレッサーの状態
float g_comp_threshold = 0.12589254f;  // -18 dBFS
float g_comp_target = 1.0f;
float g_comp_gain = 1.0f;
float g_comp_peak = 0.0f;
uint32_t g_comp_count = 0U;
float g_attack_coeff = 0.0f;
float g_release_coeff = 0.0f;

// リミッターの状態
float g_limiter_gain = 1.0f;
float g_limiter_release_coeff = 0.0f;
float g_limiter_ceiling = 0.89125094f;  // -1 dBFS

// dB値を線形ゲインへ変換する
inline float db_to_linear(const float db) {
  return fastpow2f(db * kDbToLog2);
}

// 時定数(秒)から1次ローパスの平滑化係数を求める
inline float smoothing_coeff(const float seconds) {
  return fastexpf(-1.0f / (seconds * kSampleRate));
}

// 平方根 (log2/pow2の恒等式で近似計算)
inline float fast_sqrt(const float x) {
  return fastpow2f(0.5f * fastlog2f(x));
}

// ステレオ検出: 左右の絶対値の大きい方を返す
inline float max_abs(const float left, const float right) {
  const float a = si_fabsf(left);
  const float b = si_fabsf(right);
  return (a > b) ? a : b;
}

// normalized(0..1)からAGC反応時間(秒)を対数的に求め、平滑化係数へ反映する
inline void set_agc_reaction(const float normalized) {
  const float span = fastlog2f(kAgcReactionMax / kAgcReactionMin);
  const float seconds = kAgcReactionMin * fastpow2f(span * normalized);
  g_agc_reaction_coeff = smoothing_coeff(seconds);
}
}  // namespace

void MODFX_INIT(uint32_t platform, uint32_t api) {
  (void)platform;
  (void)api;

  // AGC初期化。K特性は省略し、BS.1770のオフセットのみを考慮した近似ラウドネス。
  g_agc_target_rms = db_to_linear(kAgcTargetLufs + kLufsOffset);
  g_agc_ms = g_agc_target_rms * g_agc_target_rms;  // 目標付近から開始
  g_agc_target_gain = 1.0f;
  g_agc_gain = 1.0f;
  g_agc_window_coeff = smoothing_coeff(kAgcWindowSeconds);
  g_agc_max_gain = db_to_linear(kAgcMaxGainDb);
  g_agc_min_gain = db_to_linear(kAgcMinGainDb);
  const float gate_rms = db_to_linear(kAgcGateDb);
  g_agc_gate_ms = gate_rms * gate_rms;
  g_agc_count = 0U;
  set_agc_reaction(0.5f);  // 反応時間の初期値 (TIMEつまみで上書きされる)

  // コンプレッサー初期化 (スレッショルド固定)
  g_comp_threshold = db_to_linear(kCompThresholdDb);
  g_comp_target = 1.0f;
  g_comp_gain = 1.0f;
  g_comp_peak = 0.0f;
  g_comp_count = 0U;
  g_attack_coeff = smoothing_coeff(kAttackSeconds);
  g_release_coeff = smoothing_coeff(kReleaseSeconds);

  // リミッター初期化
  g_limiter_gain = 1.0f;
  g_limiter_release_coeff = smoothing_coeff(kLimiterReleaseSeconds);
  g_limiter_ceiling = db_to_linear(kLimiterCeilingDb);
}

void MODFX_PROCESS(const float *main_xn, float *main_yn,
                   const float *sub_xn, float *sub_yn,
                   uint32_t frames) {
  (void)sub_xn;
  (void)sub_yn;

  for (uint32_t frame = 0U; frame < frames; ++frame) {
    const uint32_t i = frame * 2U;
    const float left = main_xn[i];
    const float right = main_xn[i + 1U];

    // --- AGC: ラウドネス測定 ---
    // ステレオの平均二乗値を測定窓で平滑化してラウドネスを推定する
    const float ms_inst = 0.5f * (left * left + right * right);
    g_agc_ms = g_agc_window_coeff * g_agc_ms +
               (1.0f - g_agc_window_coeff) * ms_inst;

    // --- AGC: 目標ゲインの更新 (制御レート) ---
    if (++g_agc_count >= kAgcControlInterval) {
      g_agc_count = 0U;
      if (g_agc_ms > g_agc_gate_ms) {
        // 目標RMSに合わせるためのゲインを求め、上下限でクリップする
        const float rms = fast_sqrt(g_agc_ms);
        float desired = g_agc_target_rms / rms;
        if (desired > g_agc_max_gain) {
          desired = g_agc_max_gain;
        } else if (desired < g_agc_min_gain) {
          desired = g_agc_min_gain;
        }
        g_agc_target_gain = desired;
      }
      // ゲート以下(無音)では目標ゲインを保持し、ノイズの増幅を防ぐ
    }

    // 反応時間に応じてAGCゲインを目標へ緩やかに追従させる
    g_agc_gain = g_agc_reaction_coeff * g_agc_gain +
                 (1.0f - g_agc_reaction_coeff) * g_agc_target_gain;

    const float agc_left = left * g_agc_gain;
    const float agc_right = right * g_agc_gain;

    // --- コンプレッサー (AGC後の信号を検出) ---
    const float comp_peak = max_abs(agc_left, agc_right);
    if (comp_peak > g_comp_peak) {
      g_comp_peak = comp_peak;
    }

    if (++g_comp_count >= kCompControlInterval) {
      g_comp_count = 0U;
      if (g_comp_peak > g_comp_threshold && g_comp_peak > 1.0e-12f) {
        const float ratio = g_comp_threshold / g_comp_peak;
        g_comp_target = fastpow2f(kRatioExponent * fastlog2f(ratio));
      } else {
        g_comp_target = 1.0f;
      }
      g_comp_peak = 0.0f;
    }

    const float comp_coeff =
        (g_comp_target < g_comp_gain) ? g_attack_coeff : g_release_coeff;
    g_comp_gain = comp_coeff * g_comp_gain +
                  (1.0f - comp_coeff) * g_comp_target;

    const float comp_left = agc_left * g_comp_gain;
    const float comp_right = agc_right * g_comp_gain;

    // --- リミッター (コンプレッサー後の信号を検出) ---
    const float out_peak = max_abs(comp_left, comp_right);
    const float limiter_target =
        (out_peak > g_limiter_ceiling && out_peak > 1.0e-12f)
            ? (g_limiter_ceiling / out_peak)
            : 1.0f;

    if (limiter_target < g_limiter_gain) {
      g_limiter_gain = limiter_target;  // ルックアヘッド無し、アタックは即時
    } else {
      g_limiter_gain =
          g_limiter_release_coeff * g_limiter_gain +
          (1.0f - g_limiter_release_coeff) * limiter_target;
    }

    main_yn[i] = comp_left * g_limiter_gain;
    main_yn[i + 1U] = comp_right * g_limiter_gain;
  }
}

void MODFX_PARAM(uint8_t index, int32_t value) {
  const float normalized = clip01f(q31_to_f32(value));

  switch (index) {
    case k_user_modfx_param_time: {
      // TIME = AGCの反応時間。時計回りで 1秒 -> 30秒 (対数)。
      set_agc_reaction(normalized);
      break;
    }

    case k_user_modfx_param_depth: {
      // DEPTH = 未使用。
      (void)normalized;
      break;
    }

    default:
      break;
  }
}
