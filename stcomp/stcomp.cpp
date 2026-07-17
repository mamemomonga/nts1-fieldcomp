#include "usermodfx.h"
#include <stdint.h>

namespace {
constexpr float kSampleRate = 48000.0f;
constexpr float kRatioExponent = 0.75f;  // 4:1 のコンプレッションレシオ
constexpr float kAttackSeconds = 0.005f;
constexpr float kReleaseSeconds = 0.150f;
constexpr float kLimiterReleaseSeconds = 0.050f;
constexpr uint32_t kControlInterval = 8U;
constexpr float kDbToLog2 = 0.16609640474436813f;  // 1 / 6.020599913...

float g_threshold = 0.12589254f;  // -18 dBFS
float g_makeup = 1.9952623f;      // +6 dB
float g_comp_target = 1.0f;
float g_comp_gain = 1.0f;
float g_limiter_gain = 1.0f;
float g_control_peak = 0.0f;
uint32_t g_control_count = 0U;

float g_attack_coeff = 0.0f;
float g_release_coeff = 0.0f;
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

// ステレオ検出: 左右の絶対値の大きい方を返す
inline float max_abs(const float left, const float right) {
  const float a = si_fabsf(left);
  const float b = si_fabsf(right);
  return (a > b) ? a : b;
}
}  // namespace

void MODFX_INIT(uint32_t platform, uint32_t api) {
  (void)platform;
  (void)api;

  g_threshold = db_to_linear(-18.0f);
  g_makeup = db_to_linear(40.0f);  // メークアップゲイン初期値 +40 dB
  g_comp_target = 1.0f;
  g_comp_gain = 1.0f;
  g_limiter_gain = 1.0f;
  g_control_peak = 0.0f;
  g_control_count = 0U;

  g_attack_coeff = smoothing_coeff(kAttackSeconds);
  g_release_coeff = smoothing_coeff(kReleaseSeconds);
  g_limiter_release_coeff = smoothing_coeff(kLimiterReleaseSeconds);
  g_limiter_ceiling = db_to_linear(-1.0f);
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
    const float peak = max_abs(left, right);

    if (peak > g_control_peak) {
      g_control_peak = peak;
    }

    if (++g_control_count >= kControlInterval) {
      if (g_control_peak > g_threshold && g_control_peak > 1.0e-12f) {
        const float ratio = g_threshold / g_control_peak;
        g_comp_target = fastpow2f(kRatioExponent * fastlog2f(ratio));
      } else {
        g_comp_target = 1.0f;
      }
      g_control_peak = 0.0f;
      g_control_count = 0U;
    }

    const float comp_coeff =
        (g_comp_target < g_comp_gain) ? g_attack_coeff : g_release_coeff;
    g_comp_gain = comp_coeff * g_comp_gain +
                  (1.0f - comp_coeff) * g_comp_target;

    float out_left = left * g_comp_gain * g_makeup;
    float out_right = right * g_comp_gain * g_makeup;

    const float out_peak = max_abs(out_left, out_right);
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

    main_yn[i] = out_left * g_limiter_gain;
    main_yn[i + 1U] = out_right * g_limiter_gain;
  }
}

void MODFX_PARAM(uint8_t index, int32_t value) {
  const float normalized = clip01f(q31_to_f32(value));

  switch (index) {
    case k_user_modfx_param_time: {
      // TIME = コンプレッション度。時計回りでスレッショルド -6 -> -36 dBFS。
      const float threshold_db = -6.0f - 30.0f * normalized;
      g_threshold = db_to_linear(threshold_db);
      break;
    }

    case k_user_modfx_param_depth: {
      // DEPTH = コンプレッサー後段のメークアップゲイン。0 -> +40 dB。
      g_makeup = db_to_linear(40.0f * normalized);
      break;
    }

    default:
      break;
  }
}
