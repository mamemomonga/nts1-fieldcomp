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
//
// AGCはdB(対数)領域で、かつ制御レートで動作させる。反応時間が1〜30秒と
// 非常に長いため、毎サンプルの1次フィルタ係数(1に極めて近い値)を高速近似関数
// fastexpfで作ると近似誤差で係数が1を超え、フィルタが発散する。そこで
// 微小xでの近似 1 - exp(-x) ≒ x を用い、係数が必ず[0,1]に収まるようにする。
// #############################################################################

namespace {
constexpr float kSampleRate = 48000.0f;
constexpr float kDbToLog2 = 0.16609640474436813f;  // 1 / 6.020599913...
constexpr float kMsToDb = 3.010299957f;            // 10 / log2(10), 平均二乗->dB

// --- AGC (オートゲインコントローラ) ---
constexpr float kAgcTargetLufs = -14.0f;      // 目標ラウドネス (約 -14 LUFS)
constexpr float kLufsOffset = 0.691f;          // BS.1770のラウドネスオフセット
// 目標RMSレベル(dBFS)。ラウドネス ≒ -0.691 + RMS(dBFS) の近似から導出。
constexpr float kAgcTargetRmsDb = kAgcTargetLufs + kLufsOffset;
constexpr float kAgcWindowSeconds = 0.4f;      // ラウドネス測定窓 (モーメンタリ相当)
constexpr float kAgcReactionMin = 1.0f;        // 反応時間の最小 (秒)
constexpr float kAgcReactionMax = 30.0f;       // 反応時間の最大 (秒)
constexpr float kAgcMaxGainDb = 40.0f;         // AGCゲイン上限 (+40 dB)
constexpr float kAgcMinGainDb = -24.0f;        // AGCゲイン下限 (-24 dB)
constexpr float kAgcGateDb = -45.0f;           // 無音判定スレッショルド (RMS)
constexpr uint32_t kAgcControlInterval = 32U;  // AGCゲインの更新間隔 (サンプル)

// --- コンプレッサー (設定固定) ---
constexpr float kCompThresholdDb = -18.0f;     // 固定スレッショルド
constexpr float kRatioExponent = 0.75f;        // 4:1 のコンプレッションレシオ
constexpr float kAttackSeconds = 0.005f;
constexpr float kReleaseSeconds = 0.150f;
constexpr uint32_t kCompControlInterval = 8U;

// --- リミッター ---
constexpr float kLimiterReleaseSeconds = 0.050f;
constexpr float kLimiterCeilingDb = -1.0f;

// AGCの状態 (dB領域で制御)
float g_agc_ms = 0.0f;             // 平滑化した平均二乗値 (ラウドネス推定)
float g_agc_gain_db = 0.0f;        // 適用中のAGCゲイン (dB)
float g_agc_gain = 1.0f;           // 適用中のAGCゲイン (線形, dBから変換)
float g_agc_window_alpha = 0.0f;   // 測定窓の1次係数 (毎サンプル)
float g_agc_reaction_alpha = 0.0f; // 反応時間の1次係数 (制御レート, TIMEで変化)
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

// 時定数(秒)から1次ローパスの平滑化係数を求める (短い時定数向け)
inline float smoothing_coeff(const float seconds) {
  return fastexpf(-1.0f / (seconds * kSampleRate));
}

// ステレオ検出: 左右の絶対値の大きい方を返す
inline float max_abs(const float left, const float right) {
  const float a = si_fabsf(left);
  const float b = si_fabsf(right);
  return (a > b) ? a : b;
}

// normalized(0..1)からAGC反応時間(秒)を対数的に求め、制御レートの1次係数へ反映する。
// 1 - exp(-T/tau) ≒ T/tau (T<<tau) の近似を使い、係数を必ず[0,1]に収める。
inline void set_agc_reaction(const float normalized) {
  const float span = fastlog2f(kAgcReactionMax / kAgcReactionMin);  // log2(30)
  const float seconds = kAgcReactionMin * fastpow2f(span * normalized);  // 1..30秒
  const float control_period = (float)kAgcControlInterval / kSampleRate;
  float alpha = control_period / seconds;
  if (alpha > 1.0f) {
    alpha = 1.0f;
  }
  g_agc_reaction_alpha = alpha;
}
}  // namespace

void MODFX_INIT(uint32_t platform, uint32_t api) {
  (void)platform;
  (void)api;

  // AGC初期化。K特性は省略し、BS.1770のオフセットのみを考慮した近似ラウドネス。
  // 平均二乗値の初期値は目標レベル相当 (ms = 10^(RMSdB/10) = db_to_linear(2*RMSdB))。
  g_agc_ms = db_to_linear(2.0f * kAgcTargetRmsDb);
  g_agc_gain_db = 0.0f;
  g_agc_gain = 1.0f;
  g_agc_window_alpha = 1.0f / (kAgcWindowSeconds * kSampleRate);
  g_agc_gate_ms = db_to_linear(2.0f * kAgcGateDb);
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
    g_agc_ms += g_agc_window_alpha * (ms_inst - g_agc_ms);

    // --- AGC: ゲイン更新 (制御レート, dB領域) ---
    if (++g_agc_count >= kAgcControlInterval) {
      g_agc_count = 0U;
      if (g_agc_ms > g_agc_gate_ms) {
        // 入力RMSレベル(dBFS)を求め、目標との差を必要ゲイン(dB)とする
        const float level_db = kMsToDb * fastlog2f(g_agc_ms);
        float gain_db = kAgcTargetRmsDb - level_db;
        if (gain_db > kAgcMaxGainDb) {
          gain_db = kAgcMaxGainDb;
        } else if (gain_db < kAgcMinGainDb) {
          gain_db = kAgcMinGainDb;
        }
        // 反応時間に応じてdB領域で目標へ緩やかに追従する (発散しない安定な更新)
        g_agc_gain_db += g_agc_reaction_alpha * (gain_db - g_agc_gain_db);
        g_agc_gain = db_to_linear(g_agc_gain_db);
      }
      // ゲート以下(無音)ではゲインを保持し、ノイズの増幅を防ぐ
    }

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
