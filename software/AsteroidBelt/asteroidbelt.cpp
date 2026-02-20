#include <cmath>
#include <q/fx/biquad.hpp>
#include <q/fx/dynamic.hpp>
#include <q/fx/envelope.hpp>
#include <q/support/frequency.hpp>
#include <q/support/literals.hpp>

#include "ImpulseResponse/ImpulseResponse.h"
#include "ImpulseResponse/ir_data.h"
#include "daisysp.h"
#include "funbox_hardware.h"

using namespace daisy;
using namespace daisysp;
using namespace cycfi::q::literals;

// ============================================================
// Hardware + UI
// ============================================================

FunboxHardware hw;
Parameter level, bass, mid, treble, expression, reverb_amt;

// ============================================================
// Model / DSP
// ============================================================

ReverbSc DSY_SDRAM_BSS reverb;
ImpulseResponse mIR;
int m_currentIRindex = 0;

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
cycfi::q::highpass headphone_hpf{80.0f, 48000.0f, 0.707f};
cycfi::q::lowpass headphone_lpf{7000.0f, 48000.0f, 0.707f};
constexpr float crossfeed_amt = 0.08f;

// ============================================================
// Audio Callback
// ============================================================

constexpr size_t kDoublerMaxDelay = 4800;  // 0.1s at 48kHz
DelayLine<float, kDoublerMaxDelay> stereoDoubler;
float doublerDelayMs = 4.0f;
bool doublerEnabled = false;

constexpr size_t kRoomDelay = 480;  // ~10 ms at 48kHz
static DelayLine<float, kRoomDelay> earlyRef;

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

  // now do only signal processing per sample
  for (size_t i = 0; i < size; ++i) {
    float sig = in[0][i];

    sig = dc_in.Process(sig);

    // IR selection
    float ir_out;
    if (hw.switches[FunboxHardware::SW_10].Pressed()) {
      ir_out = mIR.Process(sig);
    } else {
      ir_out = sig;
    }

    // Headphone EQ: HPF
    float cab = headphone_hpf(ir_out);

    // EQ
    cab = eq[0](cab);
    cab = eq[1](cab);
    cab = eq[2](cab);

    // Micro room reflection (amp-in-the-room illusion)
    if (hw.switches[FunboxHardware::SW_9].Pressed()) {
      float early = earlyRef.Read();
      earlyRef.Write(ir_out);  // write pre-EQ, pre-reflection signal
      cab += 0.05f * early;
    }

    // ReverbSc stereo processing (mono in, stereo out)
    float wetL, wetR;
    const float inL = cab;
    const float inR = cab;
    reverb.Process(inL, inR, &wetL, &wetR);

    float dryL, dryR;
    CalculateMix(s_reverb_amt, wetL, dryL);
    CalculateMix(s_reverb_amt, wetR, dryR);

    float outL = dryL + wetL;
    float outR = dryR + wetR;

    // Stereo doubler: add short delay to right channel if enabled
    if (doublerEnabled) {
      const float delayed = stereoDoubler.Read();
      stereoDoubler.Write(outR);
      outR = delayed;
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
    fade += (1.0f / (hw.AudioSampleRate() * 0.05f));  // 50 ms fade
    if (fade > 1.0f) fade = 1.0f;

    outL *= fade;
    outR *= fade;

    outL = softlimit(outL * s_level);
    outR = softlimit(outR * s_level);

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
  hw.SetAudioBlockSize(96);

  level.Init(hw.knob[FunboxHardware::KNOB_1], 0.0f, 0.4f, Parameter::LINEAR);
  reverb_amt.Init(hw.knob[FunboxHardware::KNOB_2], 0.0f, 1.0f,
                  Parameter::LINEAR);
  // Knob 3 unused
  bass.Init(hw.knob[FunboxHardware::KNOB_4], 0.0f, 1.0f, Parameter::LINEAR);
  mid.Init(hw.knob[FunboxHardware::KNOB_5], 0.0f, 1.0f, Parameter::LINEAR);
  treble.Init(hw.knob[FunboxHardware::KNOB_6], 0.0f, 1.0f, Parameter::LINEAR);

  reverb.Init(hw.AudioSampleRate());
  // Set reverb feedback (decay) and lowpass frequency (damping) to subtle
  // defaults
  reverb.SetFeedback(0.35f);
  reverb.SetLpFreq(4000.0f);

  mIR.Init(ir_collection[m_currentIRindex]);
  dc_in.Init(hw.AudioSampleRate());
  dc_out_L.Init(hw.AudioSampleRate());
  dc_out_R.Init(hw.AudioSampleRate());
  stereoDoubler.Init();
  stereoDoubler.SetDelay(doublerDelayMs * (hw.AudioSampleRate() / 1000.0f));
  earlyRef.Init();
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
      if (ir_idx != m_currentIRindex) {
        m_currentIRindex = ir_idx;
        mIR.Init(ir_collection[m_currentIRindex]);
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
