#include <cmath>
#include <q/fx/biquad.hpp>
#include <q/fx/dynamic.hpp>
#include <q/fx/envelope.hpp>
#include <q/support/frequency.hpp>
#include <q/support/literals.hpp>

#include "ImpulseResponse/ImpulseResponse.h"
#include "ImpulseResponse/ir_data.h"
#include "Dattorro/Dattorro.hpp"
#include "daisysp.h"
#include "funbox_hardware.h"

using namespace daisy;
using namespace daisysp;
using namespace cycfi::q::literals;

struct DriftMod {
  float cur = 0.0f;
  float target = 0.0f;
  int countdown = 0;
  int period_samps = 1;
  float slew_a = 0.001f;       // per-sample slew coefficient
  uint32_t rng = 0x12345678u;  // xorshift state

  // update_hz: how often to pick a new random target (e.g. 2.0 Hz)
  // slew_ms:   smoothing time constant (e.g. 250 ms)
  void Init(float sr, float update_hz, float slew_ms, uint32_t seed) {
    rng = seed ? seed : 0x12345678u;
    period_samps = (int)(sr / update_hz);
    if (period_samps < 1) period_samps = 1;
    countdown = period_samps;

    float tau = slew_ms * 0.001f;
    // 1st-order smoothing coefficient
    slew_a = 1.0f - expf(-1.0f / (tau * sr));
    cur = 0.0f;
    target = 0.0f;
  }

  inline float RandBipolar() {
    // xorshift32 -> [-1, +1]
    rng ^= rng << 13;
    rng ^= rng >> 17;
    rng ^= rng << 5;
    float u = (float)rng * (1.0f / 4294967296.0f);  // [0,1)
    return 2.0f * u - 1.0f;
  }

  inline float Process() {
    if (--countdown <= 0) {
      countdown = period_samps;
      target = RandBipolar();
    }
    cur += slew_a * (target - cur);
    return cur;  // ~[-1,1]
  }
};

// ============================================================
// Hardware + UI
// ============================================================

FunboxHardware hw;
Parameter level, bass, mid, treble, expression, reverb_amt;

// ============================================================
// Model / DSP
// ============================================================

Dattorro reverb(48000.0f, 16.0f, 4.0f);
ImpulseResponse mIR;
int m_currentIRindex = 0;
int m_desiredIRindex = 0;

// ============================================================
// Toggle state
// ============================================================

enum class TogglePos { Uninitialized, Left, Middle, Right };

// 3-way toggle mapping: 0=left, 1=middle, 2=right
inline TogglePos map_three_way(bool left_pressed, bool right_pressed) {
  if (left_pressed) return TogglePos::Left;
  if (right_pressed) return TogglePos::Right;
  return TogglePos::Middle;
}

TogglePos t_toggle1 = TogglePos::Uninitialized;
TogglePos t_toggle2 = TogglePos::Uninitialized;
TogglePos t_toggle3 = TogglePos::Uninitialized;

// ============================================================
// Control targets (written by UI thread)
// ============================================================

volatile float t_level = 1.0f;
volatile float t_bass_db = 0.0f;
volatile float t_mid_db = 0.0f;
volatile float t_treble_db = 0.0f;
volatile float t_reverb_amt = 0.0f;

// ============================================================
// Smoothed values (audio thread only)
// ============================================================

float s_level = 1.0f;
float s_bass_db = 0.0f;
float s_mid_db = 0.0f;
float s_treble_db = 0.0f;
float s_reverb_amt = 0.0f;

// ============================================================
// EQ
// ============================================================

constexpr uint8_t NUM_FILTERS = 3;
constexpr float EQ_DB_RANGE = 20.0f;
constexpr float EQ_DB_OFFSET = -10.0f;

inline float mapEqDb(float v) { return v * EQ_DB_RANGE + EQ_DB_OFFSET; }

const float eq_freqs[NUM_FILTERS] = {110.f, 900.f, 4000.f};
const float eq_q[NUM_FILTERS] = {0.7f, 0.7f, 0.7f};

cycfi::q::peaking eq[NUM_FILTERS] = {{0, eq_freqs[0], 48000, eq_q[0]},
                                     {0, eq_freqs[1], 48000, eq_q[1]},
                                     {0, eq_freqs[2], 48000, eq_q[2]}};

// ============================================================
// Noise mitigation
// ============================================================

DcBlock dc_in, dc_out_L, dc_out_R;

// Headphone output conditioning
cycfi::q::peaking hp_presence{0.0f, 3200.0f, 48000.0f, 0.9f};
cycfi::q::highpass headphone_hpf{80.0f, 48000.0f, 0.707f};
cycfi::q::lowpass headphone_lpf{7000.0f, 48000.0f, 0.707f};
constexpr float crossfeed_amt = 0.08f;

// ============================================================
// Audio Callback
// ============================================================

float sigBlock[128];
float irBlock[128];

constexpr size_t kDoublerMaxDelay = 4800;  // 0.1s at 48kHz
DelayLine<float, kDoublerMaxDelay> stereoDoubler;
float doublerDelayMs = 4.0f;
bool doublerEnabled = false;
DriftMod driftL, driftR;

inline float softlimit(float x) {
  const float limit = 0.9f;
  if (x > limit) return limit + (x - limit) * 0.1f;
  if (x < -limit) return -limit + (x + limit) * 0.1f;
  return x;
}

void CalculateMix(const float mixAmount, float& wetMix, float& dryMix) {
  //    A computationally cheap mostly energy constant crossfade from
  //    SignalSmith Blog
  //    https://signalsmith-audio.co.uk/writing/2021/cheap-energy-crossfade/

  float x2 = 1.0 - mixAmount;
  float A = mixAmount * x2;
  float B = A * (1.0 + 1.4186 * A);
  float C = B + mixAmount;
  float D = B + x2;

  wetMix = C * C;
  dryMix = D * D;
}

static void AudioCallback(AudioHandle::InputBuffer in,
                          AudioHandle::OutputBuffer out, size_t size) {
  // Update the selected IR if it has changed
  if (m_desiredIRindex != m_currentIRindex) {
    mIR.setImpulseResponse(ir_collection[m_desiredIRindex].data(), 1024, true);
    m_currentIRindex = m_desiredIRindex;
  }

  const float lv = t_level;
  const float b = t_bass_db;
  const float m = t_mid_db;
  const float tr = t_treble_db;
  const float rv = t_reverb_amt;

  // block-invariant smoothing coefficient (pick tau you like)
  const float sr = hw.AudioSampleRate();
  const float tau = 0.02f;  // 20 ms
  const float a = 1.0f - expf(-(float)size / (tau * sr));

  s_level += a * (lv - s_level);
  s_bass_db += a * (b - s_bass_db);
  s_mid_db += a * (m - s_mid_db);
  s_treble_db += a * (tr - s_treble_db);
  s_reverb_amt += a * (rv - s_reverb_amt);

  // update EQ coeffs ONCE per block
  static float last[NUM_FILTERS] = {0, 0, 0};
  float cur[NUM_FILTERS] = {s_bass_db, s_mid_db, s_treble_db};
  for (int f = 0; f < NUM_FILTERS; ++f) {
    if (fabsf(cur[f] - last[f]) > 0.01f) {
      eq[f].config(cur[f], eq_freqs[f], sr, eq_q[f]);
      last[f] = cur[f];
    }
  }

  for (size_t i = 0; i < size; ++i) {
    sigBlock[i] = dc_in.Process(in[0][i]);
  }

  const bool ir_on = hw.switches[FunboxHardware::SW_10].Pressed();
  const bool doubler_on = doublerEnabled;

  const float fade_inc = 1.0f / (hw.AudioSampleRate() * 0.05f);

  if (ir_on) {
    mIR.processBlock(sigBlock, irBlock, size);
  } else {
    arm_copy_f32(sigBlock, irBlock, size);
  }

  static float lastRv = -1.0f;
  if (fabsf(s_reverb_amt - lastRv) > 0.01f) {
    float x = s_reverb_amt;
    // Map reverb knob to decay (0.3..0.95) and tank damping
    float decay = 0.3f + 0.65f * x;
    reverb.setDecay(decay);
    // Darken the tank as reverb increases
    float tankHighCut = 440.0f * powf(2.0f, (7.5f - 1.5f * x) - 5.0f);
    reverb.tank.setHighCutFrequency(tankHighCut);
    lastRv = s_reverb_amt;
  }

  // Now do additional processing per-sample
  for (size_t i = 0; i < size; ++i) {
    // Start by using the (possibly IR'd) signal for this sample
    float cab = irBlock[i];

    // EQ
    cab = eq[0](cab);
    cab = eq[1](cab);
    cab = eq[2](cab);

    // headphone conditioning AFTER tone stack
    cab = headphone_hpf(cab);
    cab = headphone_lpf(cab);
    cab = hp_presence(cab);

    // Dattorro plate reverb (mono in, stereo out)
    float wetMix, dryMix;
    CalculateMix(s_reverb_amt, wetMix, dryMix);
    reverb.process(cab, cab);
    float outL = dryMix * cab + wetMix * reverb.getLeftOutput();
    float outR = dryMix * cab + wetMix * reverb.getRightOutput();

    if (doubler_on) {
      const float mix = 0.28f;
      const float baseL_ms = 8.0f;
      const float baseR_ms = 13.0f;
      const float depth_ms = 0.10f;  // keep SMALL (0.05..0.25 ms)

      float modL = driftL.Process();  // -1..1
      float modR = driftR.Process();  // -1..1

      float dL = (baseL_ms + depth_ms * modL) * sr * 0.001f;
      float dR = (baseR_ms + depth_ms * modR) * sr * 0.001f;

      const float x = 0.5f * (outL + outR);

      float tapL = stereoDoubler.Read(dL);
      float tapR = stereoDoubler.Read(dR);
      stereoDoubler.Write(x);

      outL += mix * tapL;
      outR += mix * tapR;
    }

    // Crossfeed
    constexpr float cross_mix = crossfeed_amt * 0.7f;
    float crossL = outL + cross_mix * outR;
    float crossR = outR + cross_mix * outL;
    float norm = 1.0f / (1.0f + cross_mix);
    outL = crossL * norm;
    outR = crossR * norm;

    // Basic power on fade to avoid any pop when starting the pedal
    static float fade = 0.0f;
    fade += fade_inc;
    if (fade > 1.0f) fade = 1.0f;

    outL = softlimit(outL * s_level * fade);
    outR = softlimit(outR * s_level * fade);

    // DC block at output (separate for L/R)
    out[0][i] = dc_out_L.Process(outL);
    out[1][i] = dc_out_R.Process(outR);
  }
}

// ============================================================
// Main
// ============================================================

int main(void) {
  hw.Init(true);
  hw.SetAudioBlockSize(48);

  level.Init(hw.knob[FunboxHardware::KNOB_1], 0.0f, 1.0f, Parameter::LINEAR);
  reverb_amt.Init(hw.knob[FunboxHardware::KNOB_2], 0.0f, 1.0f,
                  Parameter::LINEAR);
  // Knob 3 unused
  bass.Init(hw.knob[FunboxHardware::KNOB_4], 0.0f, 1.0f, Parameter::LINEAR);
  mid.Init(hw.knob[FunboxHardware::KNOB_5], 0.0f, 1.0f, Parameter::LINEAR);
  treble.Init(hw.knob[FunboxHardware::KNOB_6], 0.0f, 1.0f, Parameter::LINEAR);

  reverb.setSampleRate(hw.AudioSampleRate());
  reverb.setDecay(0.7f);
  reverb.setTankDiffusion(0.7f);
  reverb.enableInputDiffusion(true);
  reverb.setPreDelay(0.0f);
  reverb.setTimeScale(1.0f);
  reverb.setTankModShape(0.5f);
  reverb.setTankModSpeed(0.8f);
  reverb.setTankModDepth(1.0f);
  reverb.setTankFilterHighCutFrequency(7.0f);
  reverb.setTankFilterLowCutFrequency(2.5f);
  reverb.setInputFilterHighCutoffPitch(7.25f);
  reverb.setInputFilterLowCutoffPitch(2.87f);
  reverb.clear();

  // Initialize with first IR
  mIR.init(ir_collection[m_currentIRindex].data(), 1024, true);
  dc_in.Init(hw.AudioSampleRate());
  dc_out_L.Init(hw.AudioSampleRate());
  dc_out_R.Init(hw.AudioSampleRate());
  stereoDoubler.Init();
  stereoDoubler.SetDelay(doublerDelayMs * (hw.AudioSampleRate() / 1000.0f));
  driftL.Init(hw.AudioSampleRate(), 2.0f, 250.0f,
              0xA341316Cu);  // update 2 Hz, slew 250 ms
  driftR.Init(hw.AudioSampleRate(), 2.3f, 280.0f,
              0xC8013EA4u);  // slightly different
  // Headphone output conditioning filters
  headphone_hpf.config(80.0f, hw.AudioSampleRate(), 0.707f);
  headphone_lpf.config(7000.0f, hw.AudioSampleRate(), 0.707f);

  hw.StartAdc();
  hw.StartAudio(AudioCallback);

  while (1) {
    hw.ProcessAnalogControls();
    hw.ProcessDigitalControls();

    // Read 3-way toggles and apply behaviors
    auto cab_freqs = [](TogglePos pos, float& hpf, float& lpf) {
      switch (pos) {
        case TogglePos::Left:
          hpf = 90.0f;
          lpf = 6500.0f;
          break;  // 1x12 open
        case TogglePos::Middle:
          hpf = 70.0f;
          lpf = 7000.0f;
          break;  // 2x12
        case TogglePos::Right:
          hpf = 60.0f;
          lpf = 5500.0f;
          break;  // 4x12
        default:
          hpf = 80.0f;
          lpf = 7000.0f;
          break;
      }
    };

    // Toggle 1: IR selection
    TogglePos new_t1 =
        map_three_way(hw.switches[FunboxHardware::SW_3].Pressed(),
                      hw.switches[FunboxHardware::SW_4].Pressed());
    if (new_t1 != t_toggle1) {
      t_toggle1 = new_t1;
      int ir_idx = 0;
      switch (t_toggle1) {
        case TogglePos::Left:
          ir_idx = 0;
          break;
        case TogglePos::Middle:
          ir_idx = 1;
          break;
        case TogglePos::Right:
          ir_idx = 2;
          break;
        default:
          ir_idx = 0;
          break;
      }
      if (ir_idx != m_desiredIRindex) {
        m_desiredIRindex = ir_idx;
      }
    }

    // Toggle 2: Headphone cab-specific EQ
    TogglePos new_t2 =
        map_three_way(hw.switches[FunboxHardware::SW_5].Pressed(),
                      hw.switches[FunboxHardware::SW_6].Pressed());
    if (new_t2 != t_toggle2) {
      t_toggle2 = new_t2;
      float hpf_freq, lpf_freq;
      cab_freqs(t_toggle2, hpf_freq, lpf_freq);
      headphone_hpf.config(hpf_freq, hw.AudioSampleRate(), 0.707f);
      headphone_lpf.config(lpf_freq, hw.AudioSampleRate(), 0.707f);
      float pres_db = 0.0f;
      switch (t_toggle2) {
        case TogglePos::Left:
          pres_db = +1.5f;
          break;  // brighter
        case TogglePos::Middle:
          pres_db = 0.0f;
          break;
        case TogglePos::Right:
          pres_db = -3.0f;
          break;  // darker/smoother
        default:
          break;
      }
      hp_presence.config(pres_db, 3200.0f, hw.AudioSampleRate(), 0.9f);
    }

    // Toggle 3: Doubler enable
    TogglePos new_t3 =
        map_three_way(hw.switches[FunboxHardware::SW_7].Pressed(),
                      hw.switches[FunboxHardware::SW_8].Pressed());
    if (new_t3 != t_toggle3) {
      t_toggle3 = new_t3;
      doublerEnabled = (t_toggle3 == TogglePos::Right);
    }

    // Write targets ONLY
    t_level = level.Process();
    t_bass_db = mapEqDb(bass.Process());
    t_mid_db = mapEqDb(mid.Process());
    t_treble_db = mapEqDb(treble.Process());
    t_reverb_amt = reverb_amt.Process();

    System::Delay(1);
  }
}
