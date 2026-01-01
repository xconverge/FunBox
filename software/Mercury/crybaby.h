// Crybaby-style wah encapsulation
#pragma once

#include <cmath>
#include <q/fx/biquad.hpp>
#include <q/support/frequency.hpp>

namespace funbox {

enum class Preset { Classic, Morello };

constexpr float LOW_RETAIN_CUTOFF = 180.0f;
constexpr float TWO_PI = 6.28318530717958647692f;

struct CrybabyWah {
  float low_lp = 0.0f;
  float low_alpha = 0.0f;
  float last_t = 0.0f;

  cycfi::q::bandpass_cpg filter;

  Preset preset = Preset::Classic;

  explicit CrybabyWah(float sampleRate = 48000.0f)
      : filter(cycfi::q::frequency{350.f}, sampleRate, 1.4f) {
    low_alpha = 1.0f - std::exp(-TWO_PI * LOW_RETAIN_CUTOFF / sampleRate);
  }

  inline void SetPreset(Preset p) { preset = p; }

  inline void configure_from_expression(float expression,
                                        float sampleRate = 48000.0f) {
    float e = std::fmax(0.0f, std::fmin(1.0f, expression));

    // Shape sweep
    float t = std::pow(e, (preset == Preset::Morello) ? 1.8f : 2.4f);
    t = t * t * (3.0f - 2.0f * t);  // smoothstep
    last_t = t;

    float f_min, f_max = 0.f;
    float q_heel, q_toe = 0.f;

    if (preset == Preset::Morello) {
      // Tom Morello–style tighter, nasal sweep
      f_min = 350.0f;
      f_max = 2500.0f;
      q_heel = 1.2f;
      q_toe = 2.8f;
    } else {
      // Classic Crybaby
      f_min = 350.0f;
      f_max = 2800.0f;
      q_heel = 1.6f;
      q_toe = 1.3f;
    }

    float f = f_min * std::pow(f_max / f_min, t);

    // Toe-narrowing resonance (critical for wah quack)
    float q = q_heel + (q_toe - q_heel) * std::pow(t, 1.3f);

    filter.config(cycfi::q::frequency{double(f)}, sampleRate, double(q));
  }

  // Process one sample through the wah and apply pre-gain
  inline float process(float x) {
    low_lp += low_alpha * (x - low_lp);
    float dry_low = low_lp;

    float y = filter(x);

    if (preset == Preset::Morello) {
      // Aggressive asymmetric clipping
      float drive = 2.5f + 1.0f * last_t;
      float z = y * drive;
      y = (z > 0.0f) ? std::tanh(z) : std::tanh(z * 0.5f);
    } else {
      // Classic smoother saturation
      y = std::tanh(y * 1.5f);
    }

    float bass_blend = (preset == Preset::Morello)
                           ? (0.45f - 0.40f * last_t)   // thin & cutting at toe
                           : (0.35f - 0.30f * last_t);  // fuller classic feel

    bass_blend = std::fmax(bass_blend, 0.05f);

    float out = (1.0f - bass_blend) * y + bass_blend * dry_low;

    float pregain =
        (preset == Preset::Morello) ? (3.5f * (1.0f + 0.6f * last_t)) : 1.20f;

    return out * pregain;
  }
};

}  // namespace funbox
