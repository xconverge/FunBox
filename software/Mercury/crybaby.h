// Crybaby-style wah encapsulation
#pragma once

#include <cmath>
#include <q/fx/biquad.hpp>
#include <q/support/frequency.hpp>

namespace funbox {

struct CrybabyWah {
  // Parameters approximating classic Crybaby voicing
  static constexpr float F_MIN = 400.0f;
  static constexpr float F_MAX = 2200.0f;   // brighter toe-down
  static constexpr float Q_HEEL = 1.4f;     // pronounced resonance at heel
  static constexpr float Q_TOE = 1.0f;      // slightly broader at toe
  static constexpr float PRE_GAIN = 1.12f;  // slight bite into amp

  float prev_x = 0.0f;

  cycfi::q::bandpass_cpg filter;

  explicit CrybabyWah(float sampleRate = 48000.0f)
      : filter(cycfi::q::frequency{F_MIN}, sampleRate, Q_HEEL) {}

  // Configure center frequency and Q from expression (0..1)
  inline void configure_from_expression(float expression,
                                        float sampleRate = 48000.0f) {
    float t = std::pow(expression, 1.6f);
    t = t * 0.995f + 0.005f;
    float f = F_MIN * std::pow(F_MAX / F_MIN, t);
    float q = Q_TOE + (Q_HEEL - Q_TOE) * std::pow(1.0f - t, 0.55f);
    filter.config(cycfi::q::frequency{double(f)}, sampleRate, double(q));
  }

  // Process one sample through the wah and apply pre-gain
  inline float process(float x) {
    float x_hp = x - 0.995f * prev_x;
    prev_x = x;

    float y = filter(x_hp);

    // mild asymmetric saturation
    y = std::tanh(y * 1.5f);

    return (0.9f * y + 0.1f * x) * PRE_GAIN;
  }

  // Output level compensation to preserve unity
  inline float level_comp() const { return 1.0f / PRE_GAIN; }
};

}  // namespace funbox
