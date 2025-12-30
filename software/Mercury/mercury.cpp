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
// Toggle state
// ============================================================

// 3-way toggle mapping: 0=left, 1=middle, 2=right
inline int map_three_way(bool left_pressed, bool right_pressed) {
  if (left_pressed) return 0;
  if (right_pressed) return 2;
  return 1;
}

static int t_toggle1 = 1;  // default middle
static int t_toggle2 = 1;  // default middle
static int t_toggle3 = 1;  // default middle

inline void apply_model_selection_from_toggles() {
  int idx = 0;
  if (t_toggle1 == 0) {  // low gain
    idx = (t_toggle2 == 0) ? 0 : (t_toggle2 == 1) ? 1 : 2;
  } else if (t_toggle1 == 1) {  // medium gain
    idx = (t_toggle2 == 0) ? 3 : (t_toggle2 == 1) ? 4 : 5;
  } else {  // high gain
    idx = (t_toggle2 == 0) ? 6 : (t_toggle2 == 1) ? 7 : 8;
  }
  if (idx != modelIndex) {
    modelIndex = idx;
    // Gate to prevent pops while loading new model
    silence_output = true;
    popReduce = 1.0f;
    setPopReduce = 0.0f;
  }
}

inline void apply_wah_from_toggle3() {
  // Right enables wah, left/middle disables
  wah_enabled = (t_toggle3 == 2);
}

DcBlock dc_in, dc_out;

constexpr int MAX_BLOCK = 192;
constexpr int K_HARM = 12;

struct CoherentCanceller {
  int N = 0;
  float sinLUT[K_HARM][MAX_BLOCK];
  float cosLUT[K_HARM][MAX_BLOCK];

  // I/Q estimates
  float I[K_HARM] = {0};
  float Q[K_HARM] = {0};

  // adaptation speed: smaller = narrower / less guitar impact
  float mu_base = 0.001f;

  // control
  float mix = 1.0f;
  float weight[K_HARM] = {0};
  float gate[K_HARM] = {0};

  void Init(int blockSize) {
    N = blockSize;
    // reset estimates when block size changes
    for (int k = 0; k < K_HARM; ++k) {
      I[k] = 0;
      Q[k] = 0;
    }
    for (int k = 0; k < K_HARM; ++k) {
      weight[k] = 1.0f;
      gate[k] = 0.0f;
    }

    for (int k = 1; k < K_HARM; ++k) {
      for (int n = 0; n < N; ++n) {
        // exactly k cycles over one block => frequency = k * sr / N
        float phase = 2.0f * M_PI * (float)(k * n) / (float)N;
        sinLUT[k][n] = sinf(phase);
        cosLUT[k][n] = cosf(phase);
      }
    }
  }

  inline void SetMix(float m) { mix = m; }
  inline void SetWeight(int k, float w) {
    if (k > 0 && k < K_HARM) weight[k] = w;
  }
  inline void SetGate(int k, float g) {
    if (k > 0 && k < K_HARM) gate[k] = g;
  }
  inline void SetMuBase(float m) { mu_base = m; }

  inline float Process(float x, int nInBlock) {
    float y = x;

    for (int k = 1; k < K_HARM; ++k) {
      float ss = sinLUT[k][nInBlock];
      float cc = cosLUT[k][nInBlock];

      // estimate on ORIGINAL x (not y)
      float projI = x * cc;
      float projQ = x * ss;

      float mu_k = mu_base / (1.0f + 0.5f * (float)k);
      I[k] += mu_k * (projI - I[k]);
      Q[k] += mu_k * (projQ - Q[k]);

      float A = sqrtf(I[k] * I[k] + Q[k] * Q[k]);
      float gf = 1.0f;
      if (gate[k] > 0.0f) {
        gf = (A - gate[k]) / gate[k];
        if (gf < 0.0f) gf = 0.0f;
        if (gf > 1.0f) gf = 1.0f;
      }
      float strength = weight[k] * gf;
      y -= strength * (I[k] * cc + Q[k] * ss);
    }
    return mix * y + (1.0f - mix) * x;
  }
};

static CoherentCanceller cancel_in;
static CoherentCanceller cancel_out;
static int g_lastBlockSize = 0;

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

const float eq_freqs[NUM_FILTERS] = {110.f, 900.f, 4000.f, 8000.f};
const float eq_q[NUM_FILTERS] = {0.7f, 0.7f, 0.7f, 0.7f};

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
  if ((int)hw.AudioBlockSize() != g_lastBlockSize) {
    g_lastBlockSize = hw.AudioBlockSize();
    cancel_in.Init(g_lastBlockSize);
    cancel_out.Init(g_lastBlockSize);
  }
  if (bypass) {
    for (size_t i = 0; i < size; ++i) {
      out[0][i] = in[0][i];
      out[1][i] = in[1][i];
    }
    return;
  }

  if (silence_output) {
    const float sr = hw.AudioSampleRate();
    const float tau = 0.010f;  // 10 ms fade (adjust)
    const float a_gate = 1.0f - expf(-(float)size / (tau * sr));

    popReduce += a_gate * (setPopReduce - popReduce);

    if (popReduce < 0.0003f && setPopReduce == 0.0f) {
      SelectModel();
      setPopReduce = 1.0f;
    }
    if (popReduce > 0.99f && setPopReduce == 1.0f) {
      popReduce = 1.0f;
      silence_output = false;
    }
  }

  const float g = t_gain;
  const float lv = t_level;
  const float b = t_bass_db;
  const float m = t_mid_db;
  const float tr = t_treble_db;
  const float pr = t_presence_db;

  // block-invariant smoothing coefficient (pick tau you like)
  const float sr = hw.AudioSampleRate();
  const float tau = 0.02f;  // 20 ms
  const float a = 1.0f - expf(-(float)size / (tau * sr));

  s_gain += a * (g - s_gain);
  s_level += a * (lv - s_level);
  s_bass_db += a * (b - s_bass_db);
  s_mid_db += a * (m - s_mid_db);
  s_treble_db += a * (tr - s_treble_db);
  s_presence_db += a * (pr - s_presence_db);

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

    // remove block-locked whine BEFORE gain/NAM
    sig = dc_in.Process(sig);
    sig = cancel_in.Process(sig, (int)i);

    if (wah_enabled) {
      sig = wah.process(sig);
    }

    const float namIn = sig * s_gain;
    const float yModel = nam.forward(namIn) * 0.4f * nnLevelAdjust;
    float y = yModel;

    // clean any block-locked junk created downstream
    y = cancel_out.Process(y, (int)i);

    y = eq[0](y);
    y = eq[1](y);
    y = eq[2](y);
    y = eq[3](y);
    y = dc_out.Process(y);

    // crossfade to silence during model switching
    out[0][i] = out[1][i] = y * s_level * popReduce;
  }
}

// ============================================================
// Main
// ============================================================

int main(void) {
  hw.Init(true);
  hw.SetAudioBlockSize(48);

  g_lastBlockSize = hw.AudioBlockSize();
  cancel_in.Init(g_lastBlockSize);
  cancel_out.Init(g_lastBlockSize);

  // Strengthen cancellation for the 1–5 kHz bands; remove gating.
  cancel_in.SetMix(1.0f);
  cancel_out.SetMix(1.0f);
  cancel_in.SetMuBase(0.003f);
  cancel_out.SetMuBase(0.003f);
  for (int k = 1; k <= 5; ++k) {
    cancel_in.SetWeight(k, 1.0f);
    cancel_out.SetWeight(k, 1.0f);
    cancel_in.SetGate(k, 0.0f);
    cancel_out.SetGate(k, 0.0f);
  }
  for (int k = 6; k < K_HARM; ++k) {
    cancel_in.SetWeight(k, 0.8f);
    cancel_out.SetWeight(k, 0.8f);
    cancel_in.SetGate(k, 0.0f);
    cancel_out.SetGate(k, 0.0f);
  }

  setupWeightsNam();
  SelectModel();
  silence_output = false;
  popReduce = 1.0f;
  setPopReduce = 1.0f;

  gain.Init(hw.knob[FunboxHardware::KNOB_1], 0.0f, 2.0f, Parameter::LINEAR);
  level.Init(hw.knob[FunboxHardware::KNOB_2], 0.0f, 2.0f, Parameter::LINEAR);
  presence.Init(hw.knob[FunboxHardware::KNOB_3], 0.0f, 1.0f, Parameter::LINEAR);
  bass.Init(hw.knob[FunboxHardware::KNOB_4], 0.0f, 1.0f, Parameter::LINEAR);
  mid.Init(hw.knob[FunboxHardware::KNOB_5], 0.0f, 1.0f, Parameter::LINEAR);
  treble.Init(hw.knob[FunboxHardware::KNOB_6], 0.0f, 1.0f, Parameter::LINEAR);
  expression.Init(hw.expression, 0.0f, 1.0f, Parameter::LINEAR);

  dc_in.Init(hw.AudioSampleRate());
  dc_out.Init(hw.AudioSampleRate());

  hw.StartAdc();
  hw.StartAudio(AudioCallback);

  while (1) {
    hw.ProcessAnalogControls();
    hw.ProcessDigitalControls();

    // Footswitch handling: toggle bypass on FS2
    if (hw.switches[FunboxHardware::SW_2].FallingEdge()) {
      bypass = !bypass;
    }

    // Read 3-way toggles and apply behaviors
    {
      int new_t1 = map_three_way(hw.switches[FunboxHardware::SW_3].Pressed(),
                                 hw.switches[FunboxHardware::SW_4].Pressed());
      int new_t2 = map_three_way(hw.switches[FunboxHardware::SW_5].Pressed(),
                                 hw.switches[FunboxHardware::SW_6].Pressed());
      int new_t3 = map_three_way(hw.switches[FunboxHardware::SW_7].Pressed(),
                                 hw.switches[FunboxHardware::SW_8].Pressed());

      if (new_t1 != t_toggle1 || new_t2 != t_toggle2) {
        t_toggle1 = new_t1;
        t_toggle2 = new_t2;
        apply_model_selection_from_toggles();
      }
      if (new_t3 != t_toggle3) {
        t_toggle3 = new_t3;
        apply_wah_from_toggle3();
      }
    }

    // Write targets ONLY
    t_gain = gain.Process();
    t_level = level.Process();
    t_presence_db = mapEqDb(presence.Process());
    t_bass_db = mapEqDb(bass.Process());
    t_mid_db = mapEqDb(mid.Process());
    t_treble_db = mapEqDb(treble.Process());
    t_expression = expression.Process();

    hw.SetLed(FunboxHardware::LED_FS2, bypass ? 0.0f : 1.0f);
    hw.SetLed(FunboxHardware::LED_FS1, wah_enabled ? t_expression : 0.0f);

    hw.UpdateLeds();

    System::Delay(1);
  }
}
