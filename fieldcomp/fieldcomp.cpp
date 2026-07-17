#include "usermodfx.h"
#include <stdint.h>

// #############################################################################
// fieldcomp: 手動LEVEL -> コンプレッサー -> オートマークアップ -> リミッター
//
// 信号連鎖:
//   入力 -> LEVEL(手動ゲイン) -> コンプレッサー -> オートマークアップ -> リミッター -> 出力
//
// DJI Mic Miniなどラインより低いレベルの入力を、まず手動のLEVELで大まかに持ち上げ、
// コンプレッサー後段のオートマークアップゲインが目標ラウドネス(約 -14 LUFS)へ
// 精密に合わせる。合計で最大 +50 dB (LEVEL +30 dB + マークアップ +20 dB) の増幅が可能。
//
// マークアップのAGCは agccomp と同じくdB(対数)領域・制御レートで動作させ、
// 平滑化係数は 1 - exp(-T/tau) ≒ T/tau の近似で生成して発散を防ぐ。
//
// 操作:
//   A: TIME  = マークアップの反応時間 (1秒 -> 10秒, 対数)
//   B: LEVEL = 入力ゲイン (0 dB -> +30 dB)
// #############################################################################

namespace {
constexpr float kSampleRate = 48000.0f;
constexpr float kDbToLog2 = 0.16609640474436813f;  // 1 / 6.020599913...
constexpr float kMsToDb = 3.010299957f;            // 10 / log2(10), 平均二乗->dB

// --- 入力レベル (手動, B:LEVEL) ---
constexpr float kLevelMaxDb = 30.0f;           // LEVELゲイン上限 (+30 dB, 0で下限)

// --- コンプレッサー (設定固定) ---
constexpr float kCompThresholdDb = -18.0f;     // 固定スレッショルド
constexpr float kRatioExponent = 0.75f;        // 4:1 のコンプレッションレシオ
constexpr float kAttackSeconds = 0.005f;
constexpr float kReleaseSeconds = 0.150f;
constexpr uint32_t kCompControlInterval = 8U;

// --- オートマークアップゲイン (A:TIMEで反応時間) ---
constexpr float kMakeupTargetLufs = -14.0f;    // 目標ラウドネス (約 -14 LUFS)
constexpr float kLufsOffset = 0.691f;          // BS.1770のラウドネスオフセット
// 目標RMSレベル(dBFS)。ラウドネス ≒ -0.691 + RMS(dBFS) の近似から導出。
constexpr float kMakeupTargetRmsDb = kMakeupTargetLufs + kLufsOffset;
constexpr float kMakeupWindowSeconds = 0.4f;   // ラウドネス測定窓 (モーメンタリ相当)
constexpr float kMakeupReactionMin = 1.0f;     // 反応時間の最小 (秒)
constexpr float kMakeupReactionMax = 10.0f;    // 反応時間の最大 (秒)
constexpr float kMakeupMaxDb = 20.0f;          // マークアップ上限 (+20 dB)
constexpr float kMakeupMinDb = 0.0f;           // マークアップ下限 (0 dB, 減衰はしない)
constexpr float kMakeupGateDb = -45.0f;        // 無音判定スレッショルド (RMS)
constexpr uint32_t kMakeupControlInterval = 32U;  // マークアップ更新間隔 (サンプル)

// --- リミッター (ルックアヘッド付き) ---
// 先読み時間だけ信号を遅延させ、ピークが来る前にゲインを下げることで、
// ゼロルックアヘッド瞬時アタックで生じる波形潰れ(クリップ状の歪み)を防ぐ。
constexpr float kLimiterAttackSeconds = 0.0005f;   // アタック 0.5 ms (先読み内で下げ切る)
constexpr float kLimiterReleaseSeconds = 0.050f;   // リリース 50 ms
constexpr float kLimiterCeilingDb = -1.0f;
constexpr uint32_t kLimiterLookahead = 96U;        // 先読み 2 ms @ 48 kHz (サンプル数)

// 入力レベルの状態
float g_level_gain = 1.0f;             // LEVELゲイン (線形)

// コンプレッサーの状態
float g_comp_threshold = 0.12589254f;  // -18 dBFS
float g_comp_target = 1.0f;
float g_comp_gain = 1.0f;
float g_comp_peak = 0.0f;
uint32_t g_comp_count = 0U;
float g_attack_coeff = 0.0f;
float g_release_coeff = 0.0f;

// オートマークアップの状態 (dB領域で制御)
float g_mk_ms = 0.0f;              // 平滑化した平均二乗値 (コンプ後のラウドネス推定)
float g_mk_gain_db = 0.0f;         // 適用中のマークアップゲイン (dB)
float g_mk_gain = 1.0f;            // 適用中のマークアップゲイン (線形)
float g_mk_window_alpha = 0.0f;    // 測定窓の1次係数 (毎サンプル)
float g_mk_reaction_alpha = 0.0f;  // 反応時間の1次係数 (制御レート, TIMEで変化)
float g_mk_gate_ms = 0.0f;         // 無音判定 (平均二乗値)
uint32_t g_mk_count = 0U;

// リミッターの状態 (ルックアヘッド付き)
float g_limiter_gain = 1.0f;
float g_limiter_attack_coeff = 0.0f;
float g_limiter_release_coeff = 0.0f;
float g_limiter_ceiling = 0.89125094f;  // -1 dBFS
float g_limiter_peak = 0.0f;            // ピークホールド検出値
uint32_t g_limiter_hold = 0U;           // ピークホールド残りサンプル数
float g_la_buf_l[kLimiterLookahead];    // 先読み用の遅延バッファ (左)
float g_la_buf_r[kLimiterLookahead];    // 先読み用の遅延バッファ (右)
uint32_t g_la_pos = 0U;                 // 遅延バッファの読み書き位置

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

// normalized(0..1)からマークアップ反応時間(秒)を対数的に求め、制御レートの1次係数へ反映する。
// 1 - exp(-T/tau) ≒ T/tau (T<<tau) の近似を使い、係数を必ず[0,1]に収める。
inline void set_makeup_reaction(const float normalized) {
  const float span = fastlog2f(kMakeupReactionMax / kMakeupReactionMin);  // log2(10)
  const float seconds = kMakeupReactionMin * fastpow2f(span * normalized);  // 1..10秒
  const float control_period = (float)kMakeupControlInterval / kSampleRate;
  float alpha = control_period / seconds;
  if (alpha > 1.0f) {
    alpha = 1.0f;
  }
  g_mk_reaction_alpha = alpha;
}
}  // namespace

void MODFX_INIT(uint32_t platform, uint32_t api) {
  (void)platform;
  (void)api;

  // 入力レベル初期化 (0 dB。LEVELつまみで上書きされる)
  g_level_gain = 1.0f;

  // コンプレッサー初期化 (スレッショルド固定)
  g_comp_threshold = db_to_linear(kCompThresholdDb);
  g_comp_target = 1.0f;
  g_comp_gain = 1.0f;
  g_comp_peak = 0.0f;
  g_comp_count = 0U;
  g_attack_coeff = smoothing_coeff(kAttackSeconds);
  g_release_coeff = smoothing_coeff(kReleaseSeconds);

  // オートマークアップ初期化。K特性は省略し、BS.1770のオフセットのみを考慮した近似。
  // 平均二乗値の初期値は目標レベル相当 (ms = 10^(RMSdB/10) = db_to_linear(2*RMSdB))。
  g_mk_ms = db_to_linear(2.0f * kMakeupTargetRmsDb);
  g_mk_gain_db = 0.0f;
  g_mk_gain = 1.0f;
  g_mk_window_alpha = 1.0f / (kMakeupWindowSeconds * kSampleRate);
  g_mk_gate_ms = db_to_linear(2.0f * kMakeupGateDb);
  g_mk_count = 0U;
  set_makeup_reaction(0.5f);  // 反応時間の初期値 (TIMEつまみで上書きされる)

  // リミッター初期化 (先読みバッファをクリア)
  g_limiter_gain = 1.0f;
  g_limiter_attack_coeff = smoothing_coeff(kLimiterAttackSeconds);
  g_limiter_release_coeff = smoothing_coeff(kLimiterReleaseSeconds);
  g_limiter_ceiling = db_to_linear(kLimiterCeilingDb);
  g_limiter_peak = 0.0f;
  g_limiter_hold = 0U;
  for (uint32_t k = 0U; k < kLimiterLookahead; ++k) {
    g_la_buf_l[k] = 0.0f;
    g_la_buf_r[k] = 0.0f;
  }
  g_la_pos = 0U;
}

void MODFX_PROCESS(const float *main_xn, float *main_yn,
                   const float *sub_xn, float *sub_yn,
                   uint32_t frames) {
  (void)sub_xn;
  (void)sub_yn;

  for (uint32_t frame = 0U; frame < frames; ++frame) {
    const uint32_t i = frame * 2U;

    // --- 入力レベル (手動, B:LEVEL) ---
    const float lvl_left = main_xn[i] * g_level_gain;
    const float lvl_right = main_xn[i + 1U] * g_level_gain;

    // --- コンプレッサー (レベル後の信号を検出) ---
    const float comp_peak = max_abs(lvl_left, lvl_right);
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

    const float comp_left = lvl_left * g_comp_gain;
    const float comp_right = lvl_right * g_comp_gain;

    // --- オートマークアップゲイン (コンプ後を測定し -14 LUFS を目指す) ---
    // ステレオの平均二乗値を測定窓で平滑化してラウドネスを推定する
    const float ms_inst = 0.5f * (comp_left * comp_left + comp_right * comp_right);
    g_mk_ms += g_mk_window_alpha * (ms_inst - g_mk_ms);

    if (++g_mk_count >= kMakeupControlInterval) {
      g_mk_count = 0U;
      if (g_mk_ms > g_mk_gate_ms) {
        // コンプ後のRMSレベル(dBFS)を求め、目標との差を必要マークアップ量(dB)とする
        const float level_db = kMsToDb * fastlog2f(g_mk_ms);
        float makeup_db = kMakeupTargetRmsDb - level_db;
        if (makeup_db > kMakeupMaxDb) {
          makeup_db = kMakeupMaxDb;
        } else if (makeup_db < kMakeupMinDb) {
          makeup_db = kMakeupMinDb;
        }
        // 反応時間に応じてdB領域で目標へ緩やかに追従する (発散しない安定な更新)
        g_mk_gain_db += g_mk_reaction_alpha * (makeup_db - g_mk_gain_db);
        g_mk_gain = db_to_linear(g_mk_gain_db);
      }
      // ゲート以下(無音)ではマークアップ量を保持し、ノイズの増幅を防ぐ
    }

    const float mk_left = comp_left * g_mk_gain;
    const float mk_right = comp_right * g_mk_gain;

    // --- リミッター (ルックアヘッド付き, マークアップ後の信号を検出) ---
    // 未遅延のピークをホールドしながら検出し、遅延させた信号へゲインを適用する。
    // ピークを先読み時間だけ保持することで、遅延バッファ内をピークが通過し終える
    // まで確実にゲインを下げ続け、波形を潰さず(クリップ状の歪みを出さず)に抑える。
    const float peak = max_abs(mk_left, mk_right);
    if (peak >= g_limiter_peak) {
      // 新たなピーク: 値を更新し、先読み時間だけホールドを開始
      g_limiter_peak = peak;
      g_limiter_hold = kLimiterLookahead;
    } else if (g_limiter_hold > 0U) {
      --g_limiter_hold;  // ホールド中はピーク値を維持
    } else {
      g_limiter_peak = peak;  // ホールド終了、現在値へ (ゲイン側のリリースで平滑化)
    }

    const float limiter_target =
        (g_limiter_peak > g_limiter_ceiling && g_limiter_peak > 1.0e-12f)
            ? (g_limiter_ceiling / g_limiter_peak)
            : 1.0f;

    if (limiter_target < g_limiter_gain) {
      // アタック: 先読み時間内に目標まで滑らかに下げる
      g_limiter_gain = g_limiter_attack_coeff * g_limiter_gain +
                       (1.0f - g_limiter_attack_coeff) * limiter_target;
    } else {
      // リリース: ゲインを緩やかに戻す
      g_limiter_gain = g_limiter_release_coeff * g_limiter_gain +
                       (1.0f - g_limiter_release_coeff) * limiter_target;
    }

    // 遅延バッファから取り出し、現在のサンプルを書き込む (2 ms 遅延)
    const float delayed_left = g_la_buf_l[g_la_pos];
    const float delayed_right = g_la_buf_r[g_la_pos];
    g_la_buf_l[g_la_pos] = mk_left;
    g_la_buf_r[g_la_pos] = mk_right;
    if (++g_la_pos >= kLimiterLookahead) {
      g_la_pos = 0U;
    }

    main_yn[i] = delayed_left * g_limiter_gain;
    main_yn[i + 1U] = delayed_right * g_limiter_gain;
  }
}

void MODFX_PARAM(uint8_t index, int32_t value) {
  const float normalized = clip01f(q31_to_f32(value));

  switch (index) {
    case k_user_modfx_param_time: {
      // TIME = マークアップの反応時間。時計回りで 1秒 -> 10秒 (対数)。
      set_makeup_reaction(normalized);
      break;
    }

    case k_user_modfx_param_depth: {
      // LEVEL = 入力ゲイン。時計回りで 0 dB -> +30 dB。
      g_level_gain = db_to_linear(kLevelMaxDb * normalized);
      break;
    }

    default:
      break;
  }
}
