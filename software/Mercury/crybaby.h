// Crybaby-style wah encapsulation
#pragma once

#include <cmath>
#include <q/fx/biquad.hpp>
#include <q/support/frequency.hpp>

namespace funbox {

struct CrybabyWah {
  // Parameters approximating classic Crybaby voicing
  static constexpr float F_MIN = 350.0f;
  static constexpr float F_MAX = 2800.0f;   // brighter toe-down
  static constexpr float Q_HEEL = 1.6f;     // fuller heel, less thinning
  static constexpr float Q_TOE = 1.2f;      // slightly broader at toe
  static constexpr float PRE_GAIN = 1.20f;  // slight bite into amp
  static constexpr float LOW_RETAIN_CUTOFF = 180.0f;  // dry low preserve cutoff

  float low_lp = 0.0f;
  float low_alpha = 0.0f;
  float last_t = 0.0f;

  cycfi::q::bandpass_cpg filter;

  explicit CrybabyWah(float sampleRate = 48000.0f)
      : filter(cycfi::q::frequency{F_MIN}, sampleRate, Q_HEEL) {
    const float two_pi = 6.28318530717958647692f;
    low_alpha = 1.0f - std::exp(-two_pi * LOW_RETAIN_CUTOFF / sampleRate);
  }

  // Configure center frequency and Q from expression (0..1)
  inline void configure_from_expression(float expression,
                                        float sampleRate = 48000.0f) {
    // Calibrate expression range and shape for even sweep
    const float e_min = 0.0f;  // use full mechanical range
    const float e_max = 1.0f;  // use full mechanical range

    float e = (expression - e_min) / (e_max - e_min);
    e = std::fmax(0.0f, std::fmin(1.0f, e));

    // Add heel-soft bias to reduce sensitivity in the first ~10%
    float e_bias = std::pow(e, 2.4f);

    // Smoothstep shaping, then mild toe-late bias
    float t = e_bias * e_bias * (3.0f - 2.0f * e_bias);
    t = std::pow(t, 1.6f);
    last_t = t;

    float f = F_MIN * std::pow(F_MAX / F_MIN, t);
    float q = Q_TOE + (Q_HEEL - Q_TOE) * (1.0f - std::pow(t, 1.2f));
    filter.config(cycfi::q::frequency{double(f)}, sampleRate, double(q));
    const float two_pi = 6.28318530717958647692f;
    low_alpha = 1.0f - std::exp(-two_pi * LOW_RETAIN_CUTOFF / sampleRate);
  }

  // Process one sample through the wah and apply pre-gain
  inline float process(float x) {
    low_lp += low_alpha * (x - low_lp);
    float dry_low = low_lp;

    float y = filter(x);

    // mild asymmetric saturation
    y = std::tanh(y * 1.5f);

    float bass_blend = 0.35f - 0.30f * last_t;  // more dry low at heel
    float out = (1.0f - bass_blend) * y + bass_blend * dry_low;
    return out * PRE_GAIN;
  }
};

}  // namespace funbox
