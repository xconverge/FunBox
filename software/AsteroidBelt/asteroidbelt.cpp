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
Parameter level, presence, bass, mid, treble, expression, reverb_amt;

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
volatile float t_presence_db = 0.0f;
volatile float t_reverb_amt = 0.0f;

// ============================================================
// Smoothed values (audio thread only)
// ============================================================

float s_level = 1.0f;
float s_bass_db = 0.0f;
float s_mid_db = 0.0f;
float s_treble_db = 0.0f;
float s_presence_db = 0.0f;
float s_reverb_amt = 0.0f;

// ============================================================
// EQ
// ============================================================

constexpr uint8_t NUM_FILTERS = 4;
constexpr float EQ_DB_RANGE = 20.0f;
constexpr float EQ_DB_OFFSET = -10.0f;

inline float mapEqDb(float v) { return v * EQ_DB_RANGE + EQ_DB_OFFSET; }

const float eq_freqs[NUM_FILTERS] = {110.f, 900.f, 4000.f, 8000.f};
const float eq_q[NUM_FILTERS] = {0.7f, 0.7f, 0.7f, 0.7f};

cycfi::q::peaking eq[NUM_FILTERS] = {{0, eq_freqs[0], 48000, eq_q[0]},
                                     {0, eq_freqs[1], 48000, eq_q[1]},
                                     {0, eq_freqs[2], 48000, eq_q[2]},
                                     {0, eq_freqs[3], 48000, eq_q[3]}};

// ============================================================
// Noise mitigation
// ============================================================

DcBlock dc_in, dc_out;

// ============================================================
// Audio Callback
// ============================================================

static void AudioCallback(AudioHandle::InputBuffer in,
                          AudioHandle::OutputBuffer out, size_t size) {
  const float lv = t_level;
  const float b = t_bass_db;
  const float m = t_mid_db;
  const float tr = t_treble_db;
  const float pr = t_presence_db;
  const float rv = t_reverb_amt;

  // block-invariant smoothing coefficient (pick tau you like)
  const float sr = hw.AudioSampleRate();
  const float tau = 0.02f;  // 20 ms
  const float a = 1.0f - expf(-(float)size / (tau * sr));

  s_level += a * (lv - s_level);
  s_bass_db += a * (b - s_bass_db);
  s_mid_db += a * (m - s_mid_db);
  s_treble_db += a * (tr - s_treble_db);
  s_presence_db += a * (pr - s_presence_db);
  s_reverb_amt += a * (rv - s_reverb_amt);

  // update EQ coeffs ONCE per block
  static float last[NUM_FILTERS] = {0, 0, 0, 0};
  float cur[NUM_FILTERS] = {s_bass_db, s_mid_db, s_treble_db, s_presence_db};
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
    float ir_out = mIR.Process(sig);

    // EQ
    ir_out = eq[0](ir_out);
    ir_out = eq[1](ir_out);
    ir_out = eq[2](ir_out);
    ir_out = eq[3](ir_out);

    // ReverbSc stereo processing (mono in)
    float wetL, wetR;
    float inL = ir_out;
    float inR = ir_out;
    reverb.Process(inL, inR, &wetL, &wetR);
    float out_sample = (1.0f - s_reverb_amt) * ir_out + s_reverb_amt * wetL;

    out_sample = dc_out.Process(out_sample);

    out[0][i] = out[1][i] = out_sample * s_level;
  }
}

// ============================================================
// Main
// ============================================================

int main(void) {
  hw.Init(true);
  hw.SetAudioBlockSize(48);

  level.Init(hw.knob[FunboxHardware::KNOB_1], 0.0f, 2.0f, Parameter::LINEAR);
  reverb_amt.Init(hw.knob[FunboxHardware::KNOB_2], 0.0f, 1.0f,
                  Parameter::LINEAR);
  presence.Init(hw.knob[FunboxHardware::KNOB_3], 0.0f, 1.0f, Parameter::LINEAR);
  bass.Init(hw.knob[FunboxHardware::KNOB_4], 0.0f, 1.0f, Parameter::LINEAR);
  mid.Init(hw.knob[FunboxHardware::KNOB_5], 0.0f, 1.0f, Parameter::LINEAR);
  treble.Init(hw.knob[FunboxHardware::KNOB_6], 0.0f, 1.0f, Parameter::LINEAR);

  reverb.Init(hw.AudioSampleRate());
  mIR.Init(ir_collection[m_currentIRindex]);

  hw.StartAdc();
  hw.StartAudio(AudioCallback);

  while (1) {
    hw.ProcessAnalogControls();
    hw.ProcessDigitalControls();

    // Read 3-way toggles and apply behaviors
    {
      TogglePos new_t1 =
          map_three_way(hw.switches[FunboxHardware::SW_3].Pressed(),
                        hw.switches[FunboxHardware::SW_4].Pressed());
      if (new_t1 != t_toggle1) {
        t_toggle1 = new_t1;
        // Select IR
        int ir_idx = 0;
        if (t_toggle1 == TogglePos::Left)
          ir_idx = 0;
        else if (t_toggle1 == TogglePos::Middle)
          ir_idx = 1;
        else
          ir_idx = 2;
        if (ir_idx != m_currentIRindex) {
          m_currentIRindex = ir_idx;
          mIR.Init(ir_collection[m_currentIRindex]);
        }
      }
      // toggles 2 and 3 are unused
    }

    // Write targets ONLY
    t_level = level.Process();
    t_presence_db = mapEqDb(presence.Process());
    t_bass_db = mapEqDb(bass.Process());
    t_mid_db = mapEqDb(mid.Process());
    t_treble_db = mapEqDb(treble.Process());
    t_reverb_amt = reverb_amt.Process();

    System::Delay(1);
  }
}
