#include <RTNeural/RTNeural.h>

#include <q/fx/biquad.hpp>
#include <q/support/frequency.hpp>

#include "crybaby.h"
#include "daisysp.h"
#include "funbox_hardware.h"
#include "model_data_nam.h"
#include "wavenet/wavenet_model.hpp"

using namespace funbox;
using namespace daisy;
using namespace daisysp;

// ============================================================
// Hardware + UI
// ============================================================

FunboxHardware hw;
Parameter gain, level, presence, bass, mid, treble, expression;

bool bypass = true;

// ============================================================
// Model / DSP
// ============================================================

int modelIndex = 0;
int m_currentModelindex = -1;
float nnLevelAdjust = 1.0f;

bool silence_output = false;
float popReduce = 1.0f;
float setPopReduce = 1.0f;

// Wah
bool wah_enabled = false;
CrybabyWah wah;

// ============================================================
// Control targets (written by UI thread)
// ============================================================

volatile float t_gain = 1.0f;
volatile float t_level = 1.0f;
volatile float t_bass_db = 0.0f;
volatile float t_mid_db = 0.0f;
volatile float t_treble_db = 0.0f;
volatile float t_presence_db = 0.0f;
volatile float t_expression = 0.0f;

// ============================================================
// Smoothed values (audio thread only)
// ============================================================

float s_gain = 1.0f;
float s_level = 1.0f;
float s_bass_db = 0.0f;
float s_mid_db = 0.0f;
float s_treble_db = 0.0f;
float s_presence_db = 0.0f;
float s_expression = 0.0f;

// ============================================================
// EQ
// ============================================================

constexpr uint8_t NUM_FILTERS = 4;
constexpr float EQ_DB_RANGE = 20.0f;
constexpr float EQ_DB_OFFSET = -10.0f;

inline float mapEqDb(float v) { return v * EQ_DB_RANGE + EQ_DB_OFFSET; }

const float eq_freqs[NUM_FILTERS] = {180.f, 1200.f, 4000.f, 8000.f};
const float eq_q[NUM_FILTERS] = {0.7f, 0.6f, 0.5f, 0.5f};

cycfi::q::peaking eq[NUM_FILTERS] = {{0, eq_freqs[0], 48000, eq_q[0]},
                                     {0, eq_freqs[1], 48000, eq_q[1]},
                                     {0, eq_freqs[2], 48000, eq_q[2]},
                                     {0, eq_freqs[3], 48000, eq_q[3]}};

// ============================================================
// NN
// ============================================================

struct NAMMathsProvider {
#if RTNEURAL_USE_EIGEN
  template <typename Matrix>
  static auto tanh(const Matrix& x) {
    const auto p = x.array() * (1.0f + 0.183428244899f * x.array().square());
    return p.array() * (p.array().square() + 1.0f).array().rsqrt();
  }
#endif
};

using Dilations = wavenet::Dilations<1, 2, 4, 8, 16, 32, 64>;
using Dilations2 =
    wavenet::Dilations<128, 256, 512, 1, 2, 4, 8, 16, 32, 64, 128, 256, 512>;

wavenet::Wavenet_Model<float, 1,
                       wavenet::Layer_Array<float, 1, 1, 2, 2, 3, Dilations,
                                            false, NAMMathsProvider>,
                       wavenet::Layer_Array<float, 2, 1, 1, 2, 3, Dilations2,
                                            true, NAMMathsProvider>>
    nam;

// ============================================================
// Helpers
// ============================================================

inline float zap_denorm(float x) { return (fabsf(x) < 1e-20f) ? 0.f : x; }

void SelectModel() {
  if (m_currentModelindex != modelIndex) {
    nam.load_weights(model_collection_nam[modelIndex].weights);
    nam.prepare(1);
    nam.prewarm();
    m_currentModelindex = modelIndex;
  }
}

// ============================================================
// Audio Callback
// ============================================================

static void AudioCallback(AudioHandle::InputBuffer in,
                          AudioHandle::OutputBuffer out, size_t size) {
  for (size_t i = 0; i < size; i++) {
    float sig = in[0][i];

    if (bypass) {
      out[0][i] = in[0][i];
      out[1][i] = in[1][i];
      continue;
    }

    // ===============================
    // Smooth all parameters (CRITICAL)
    // ===============================

    fonepole(s_gain, t_gain, 0.001f);
    fonepole(s_level, t_level, 0.001f);
    fonepole(s_bass_db, t_bass_db, 0.001f);
    fonepole(s_mid_db, t_mid_db, 0.001f);
    fonepole(s_treble_db, t_treble_db, 0.001f);
    fonepole(s_presence_db, t_presence_db, 0.001f);
    fonepole(s_expression, t_expression, 0.001f);

    // Wah
    if (wah_enabled) sig = wah.process(sig);

    // Model switching gate
    if (silence_output) {
      fonepole(popReduce, setPopReduce, 0.0002f);
      if (popReduce < 0.0003f && setPopReduce == 0.0f) {
        SelectModel();
        setPopReduce = 1.0f;
      }
      if (popReduce > 0.99f && setPopReduce == 1.0f) {
        popReduce = 1.0f;
        silence_output = false;
      }
    }

    float ampOut = 0.f;
    if (setPopReduce == 1.0f) {
      ampOut = nam.forward(sig * s_gain) * 0.4f * nnLevelAdjust;
    }

    // Update EQ only when smoothed values drift
    static float last[NUM_FILTERS] = {};
    float cur[NUM_FILTERS] = {s_bass_db, s_mid_db, s_treble_db, s_presence_db};

    for (int f = 0; f < NUM_FILTERS; f++) {
      if (fabsf(cur[f] - last[f]) > 0.01f) {
        eq[f].config(cur[f], eq_freqs[f], hw.AudioSampleRate(), eq_q[f]);
        last[f] = cur[f];
      }
      ampOut = eq[f](ampOut);
    }

    ampOut = zap_denorm(ampOut);

    out[0][i] = out[1][i] = ampOut * s_level * popReduce;
  }
}

// ============================================================
// Main
// ============================================================

int main(void) {
  hw.Init(true);
  hw.SetAudioBlockSize(48);

  setupWeightsNam();
  SelectModel();

  gain.Init(hw.knob[FunboxHardware::KNOB_1], 0.1f, 2.0f,
            Parameter::LOGARITHMIC);
  level.Init(hw.knob[FunboxHardware::KNOB_2], 0.0f, 2.0f, Parameter::LINEAR);
  presence.Init(hw.knob[FunboxHardware::KNOB_3], 0.0f, 1.0f, Parameter::LINEAR);
  bass.Init(hw.knob[FunboxHardware::KNOB_4], 0.0f, 1.0f, Parameter::LINEAR);
  mid.Init(hw.knob[FunboxHardware::KNOB_5], 0.0f, 1.0f, Parameter::LINEAR);
  treble.Init(hw.knob[FunboxHardware::KNOB_6], 0.0f, 1.0f, Parameter::LINEAR);
  expression.Init(hw.expression, 0.0f, 1.0f, Parameter::LINEAR);

  hw.StartAdc();
  hw.StartAudio(AudioCallback);

  while (1) {
    hw.ProcessAnalogControls();
    hw.ProcessDigitalControls();

    // Footswitch handling: toggle bypass on FS1
    if (hw.switches[FunboxHardware::SW_1].FallingEdge()) {
      bypass = !bypass;
    }

    // Write targets ONLY
    t_gain = gain.Process();
    t_level = level.Process();
    t_presence_db = mapEqDb(presence.Process());
    t_bass_db = mapEqDb(bass.Process());
    t_mid_db = mapEqDb(mid.Process());
    t_treble_db = mapEqDb(treble.Process());
    t_expression = expression.Process();

    hw.SetLed(FunboxHardware::LED_FS1, bypass ? 0.0f : 1.0f);
    hw.SetLed(FunboxHardware::LED_FS2, wah_enabled ? t_expression : 0.0f);

    hw.UpdateLeds();

    System::Delay(1);
  }
}
