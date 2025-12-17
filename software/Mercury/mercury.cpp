#include <RTNeural/RTNeural.h>

#include <q/fx/biquad.hpp>

#include "daisy_petal.h"
#include "daisysp.h"
#include "funbox.h"
#include "wavenet/wavenet_model.hpp"

// Model Weights (edit this file to add model weights trained with Colab script)
#include "model_data_nam.h"

using namespace daisy;
using namespace daisysp;
using namespace funbox;  // This is important for mapping the correct controls
                         // to the Daisy Seed on Funbox PCB

// Declare a local daisy_petal for hardware access
DaisyPetal hw;
Parameter gain, level, presence, bass, mid, treble, expression;
bool bypass;
int modelIndex = 0;
int m_currentModelindex = -1;
bool pswitch1[2], pswitch2[2], pswitch3[2], pdip[4];
int switch1[2], switch2[2], switch3[2], dip[4];
float nnLevelAdjust;

bool eqOn = true;

// Autowah
Autowah autowah;
bool autowah_enabled = false;

float knobValues[6];  // Moved to global
int toggleValues[3];
bool dipValues[4];

float dryMix, wetMix;
bool silence_output = false;
float setPopReduce, popReduce;

Led led1, led2;

struct NAMMathsProvider {
#if RTNEURAL_USE_EIGEN
  template <typename Matrix>
  static auto tanh(const Matrix& x) {
    // See: math_approx::tanh<3>
    const auto x_poly =
        x.array() * (1.0f + 0.183428244899f * x.array().square());
    return x_poly.array() * (x_poly.array().square() + 1.0f).array().rsqrt();
    // return x.array().tanh(); // Tried using Eigen's built in tanh(), also
    // works, failed on the same larger models as above custom tanh
  }
#elif RTNEURAL_USE_XSIMD
  template <typename T>
  static T tanh(const T& x) {
    return math_approx::tanh<3>(x);
  }
#endif
};

constexpr uint8_t NUM_FILTERS_NAM = 4;

const float minGain = -10.f;
const float maxGain = 10.f;

const float centerFrequencyNam[NUM_FILTERS_NAM] = {
    180.f, 1200.f, 4000.f, 8000.f};  // Experiment with these freqs and q values
const float q_nam[NUM_FILTERS_NAM] = {0.7f, 0.6f, 0.5f, 0.5f};

cycfi::q::peaking filter_nam[NUM_FILTERS_NAM] = {
    {0, centerFrequencyNam[0], 48000, q_nam[0]},
    {0, centerFrequencyNam[1], 48000, q_nam[1]},
    {0, centerFrequencyNam[2], 48000, q_nam[2]},
    {0, centerFrequencyNam[3], 48000, q_nam[3]}};

// NOTE NAM "Pico" (unnoficial model type)
// This is the same as the Nano models, except the number of channels on the
// first layer is 2 instead of 4 (and the input size on the 2nd layer is 2
// instead of 4 also)
using Dilations = wavenet::Dilations<1, 2, 4, 8, 16, 32, 64>;
using Dilations2 =
    wavenet::Dilations<128, 256, 512, 1, 2, 4, 8, 16, 32, 64, 128, 256, 512>;
wavenet::Wavenet_Model<float, 1,
                       wavenet::Layer_Array<float, 1, 1, 2, 2, 3, Dilations,
                                            false, NAMMathsProvider>,
                       wavenet::Layer_Array<float, 2, 1, 1, 2, 3, Dilations2,
                                            true, NAMMathsProvider>>
    rtneural_wavenet;

bool knobMoved(float old_value, float new_value) {
  float tolerance = 0.005;
  if (new_value > (old_value + tolerance) ||
      new_value < (old_value - tolerance)) {
    return true;
  } else {
    return false;
  }
}

void SelectModel() {
  if (m_currentModelindex != modelIndex) {
    rtneural_wavenet.load_weights(model_collection_nam[modelIndex].weights);
    static constexpr size_t N =
        1;  // number of samples sent through model at once
    rtneural_wavenet.prepare(N);  // This is needed, including this allowed the
                                  // led to come on before freezing
    rtneural_wavenet.prewarm();   // Note: looks like this just sends some 0's
                                  // through the model
    m_currentModelindex = modelIndex;
  }
}

void updateSwitch1or2() {
  if (toggleValues[0] == 0) {  // low gain models
    if (toggleValues[1] == 0) {
      modelIndex = 0;
      nnLevelAdjust = 1.3;
    } else if (toggleValues[1] == 1) {
      modelIndex = 1;
      nnLevelAdjust = 1.6;
    } else {
      modelIndex = 2;
      nnLevelAdjust = 1.1;
    }
  } else if (toggleValues[0] == 1) {  // med gain models
    if (toggleValues[1] == 0) {
      modelIndex = 3;
      nnLevelAdjust = 1.0;
    } else if (toggleValues[1] == 1) {
      modelIndex = 4;
      nnLevelAdjust = 1.0;
    } else {
      modelIndex = 5;
      nnLevelAdjust = 0.7;
    }
  } else {  // high gain models
    if (toggleValues[1] == 0) {
      modelIndex = 6;
      nnLevelAdjust = 0.9;
    } else if (toggleValues[1] == 1) {
      modelIndex = 7;
      nnLevelAdjust = 0.9;
    } else {
      modelIndex = 8;
      nnLevelAdjust = 0.9;
    }
  }
  silence_output = true;
  popReduce = 1.0;
  setPopReduce = 0.0;
  // SelectModel();
}

void updateSwitch3() {
  if (toggleValues[2] == 0) {
    autowah_enabled = false;
  } else if (toggleValues[2] == 2) {
    autowah_enabled = true;
  } else {
    autowah_enabled = false;
  }
}

void UpdateButtons() {
  // (De-)Activate bypass and toggle LED when left footswitch is let go, or
  // enable/disable amp if held for greater than 1 second // Can only
  // disable/enable amp when not in bypass mode
  if (hw.switches[Funbox::FOOTSWITCH_1].FallingEdge()) {
    bypass = !bypass;
    led1.Set(bypass ? 0.0f : 1.0f);
  }

  led1.Update();
  led2.Update();
}

void UpdateSwitches() {
  // Detect any changes in switch positions

  // 3-way Switch 1
  bool changed1 = false;
  for (int i = 0; i < 2; i++) {
    if (hw.switches[switch1[i]].Pressed() != pswitch1[i]) {
      pswitch1[i] = hw.switches[switch1[i]].Pressed();
      changed1 = true;
    }
  }
  if (changed1) {  // update_switches is for turning off preset
    if (pswitch1[0] == true) {
      toggleValues[0] = 0;
    } else if (pswitch1[1] == true) {
      toggleValues[0] = 2;
    } else {
      toggleValues[0] = 1;
    }
    updateSwitch1or2();
  }

  // 3-way Switch 2
  bool changed2 = false;
  for (int i = 0; i < 2; i++) {
    if (hw.switches[switch2[i]].Pressed() != pswitch2[i]) {
      pswitch2[i] = hw.switches[switch2[i]].Pressed();
      changed2 = true;
    }
  }
  if (changed2) {
    if (pswitch2[0] == true) {
      toggleValues[1] = 0;
    } else if (pswitch2[1] == true) {
      toggleValues[1] = 2;
    } else {
      toggleValues[1] = 1;
    }
    updateSwitch1or2();
  }

  // 3-way Switch 3
  bool changed3 = false;
  for (int i = 0; i < 2; i++) {
    if (hw.switches[switch3[i]].Pressed() != pswitch3[i]) {
      pswitch3[i] = hw.switches[switch3[i]].Pressed();
      changed3 = true;
    }
  }
  if (changed3) {
    if (pswitch3[0] == true) {
      toggleValues[2] = 0;
    } else if (pswitch3[1] == true) {
      toggleValues[2] = 2;
    } else {
      toggleValues[2] = 1;
    }
    updateSwitch3();
  }

  // Dip switches
  bool changed4 = false;
  for (int i = 0; i < 4; i++) {
    if (hw.switches[dip[i]].Pressed() != pdip[i]) {
      pdip[i] = hw.switches[dip[i]].Pressed();
      dipValues[i] =
          pdip[i];  // TODO Look into consolidating logic for dipValues, pdip,
                    // etc (this is for preset saving)
      changed4 = true;
      // Action for dipswitches handled in audio callback
    }
  }
  // Update if preset turned off
  if (changed4) {
    for (int i = 0; i < 4; i++) {
      dipValues[i] = pdip[i];  // TODO Check logic here
    }
  }
}

// This runs at a fixed rate, to prepare audio samples
static void AudioCallback(AudioHandle::InputBuffer in,
                          AudioHandle::OutputBuffer out, size_t size) {
  // hw.ProcessAllControls();
  hw.ProcessAnalogControls();
  hw.ProcessDigitalControls();

  UpdateButtons();
  UpdateSwitches();

  // Knob Processing ////////////////////
  knobValues[0] = gain.Process();
  knobValues[1] = level.Process();
  knobValues[2] = presence.Process();
  knobValues[3] = bass.Process();
  knobValues[4] = mid.Process();
  knobValues[5] = treble.Process();

  float vgain = knobValues[0];
  float vlevel = knobValues[1];
  float vpresence =
      knobValues[2] * 20.0 - 10.0;  // Make eq control range from -10 to +10 dB
  float vbass = knobValues[3] * 20.0 - 10.0;
  float vmid = knobValues[4] * 20.0 - 10.0;
  float vtreble = knobValues[5] * 20.0 - 10.0;

  // Order of effects is:
  //           Autowah -> Gain -> Neural Model -> Tone ->
  //

  // Bass, Mid, Treble, Presence
  filter_nam[0].config(vbass, centerFrequencyNam[0], 48000, q_nam[0]);
  filter_nam[1].config(vmid, centerFrequencyNam[1], 48000, q_nam[1]);
  filter_nam[2].config(vtreble, centerFrequencyNam[2], 48000, q_nam[2]);
  filter_nam[3].config(vpresence, centerFrequencyNam[3], 48000, q_nam[3]);

  float input_arr[1] = {0.0};  // Neural Net Input

  // Update autowah parameter once per block from expression
  if (autowah_enabled) {
    // 0 is heel (up), 1 is toe (down)
    float vexpression = expression.Process();
    autowah.SetWah(vexpression);
  }

  for (size_t i = 0; i < size; i++) {
    // Process your signal here
    if (bypass) {
      out[0][i] = in[0][i];
      out[1][i] = in[0][i];
    } else {
      float ampOut = 0.0;
      float sig = in[0][i];

      if (autowah_enabled) {
        sig = autowah.Process(sig);
      }

      input_arr[0] = sig * vgain;

      if (silence_output) {  // Don't process while switching models, else bad
                             // sound
        fonepole(popReduce, setPopReduce, .0002f);
        if (popReduce < 0.0003 && setPopReduce == 0.0) {
          SelectModel();
          setPopReduce = 1.0;
        }
        if (popReduce > 0.99 && setPopReduce == 1.0) {
          popReduce = 1.0;
          silence_output = false;
        }
      }

      if (setPopReduce ==
          1.0)  // If the model is finished changing, process neural net
        ampOut = rtneural_wavenet.forward(input_arr[0]) * 0.4 *
                 nnLevelAdjust;  // TODO Try try sending a block at a time,
                                 // possible speed improvement

      // Apply 4 band EQ
      if (eqOn) {
        for (uint8_t i = 0; i < NUM_FILTERS_NAM; i++) {
          ampOut = filter_nam[i](ampOut);
        }
      }

      out[0][i] = out[1][i] = ampOut * vlevel * popReduce;
    }
  }
}

int main(void) {
  float samplerate;

  hw.Init(true);
  samplerate = hw.AudioSampleRate();

  setupWeightsNam();
  hw.SetAudioBlockSize(48);

  switch1[0] = Funbox::SWITCH_1_LEFT;
  switch1[1] = Funbox::SWITCH_1_RIGHT;
  switch2[0] = Funbox::SWITCH_2_LEFT;
  switch2[1] = Funbox::SWITCH_2_RIGHT;
  switch3[0] = Funbox::SWITCH_3_LEFT;
  switch3[1] = Funbox::SWITCH_3_RIGHT;
  dip[0] = Funbox::SWITCH_DIP_1;
  dip[1] = Funbox::SWITCH_DIP_2;
  dip[2] = Funbox::SWITCH_DIP_3;
  dip[3] = Funbox::SWITCH_DIP_4;

  pswitch1[0] = true;
  pswitch1[1] = true;
  pswitch2[0] = true;
  pswitch2[1] = true;
  pswitch3[0] = true;
  pswitch3[1] = true;
  pdip[0] = true;
  pdip[1] = true;
  pdip[2] = true;
  pdip[3] = true;

  setupWeightsNam();  // in the model data nam .h file
  // updateSwitch1or2();
  SelectModel();
  setPopReduce = 1.0;
  popReduce = 1.0;

  filter_nam[0].config(0.0, centerFrequencyNam[0], samplerate, q_nam[0]);
  filter_nam[1].config(0.0, centerFrequencyNam[1], samplerate, q_nam[1]);
  filter_nam[2].config(0.0, centerFrequencyNam[2], samplerate, q_nam[2]);

  // Autowah init
  autowah.Init(samplerate);
  autowah.SetDryWet(100.0f);
  autowah.SetLevel(1.0f);

  gain.Init(hw.knob[Funbox::KNOB_1], 0.1f, 2.5f, Parameter::LOGARITHMIC);
  level.Init(hw.knob[Funbox::KNOB_2], 0.0f, 1.0f, Parameter::LINEAR);
  presence.Init(hw.knob[Funbox::KNOB_3], 0.0f, 1.0f, Parameter::LINEAR);
  bass.Init(hw.knob[Funbox::KNOB_4], 0.0f, 1.0f, Parameter::LINEAR);
  mid.Init(hw.knob[Funbox::KNOB_5], 0.0f, 1.0f, Parameter::LINEAR);
  treble.Init(hw.knob[Funbox::KNOB_6], 0.0f, 1.0f, Parameter::LINEAR);
  expression.Init(hw.expression, 0.0f, 1.0f,
                  Parameter::LINEAR);  // TODO Make sure this is the correct way
                                       // to reference expression

  // Initialize the correct model
  modelIndex = 0;
  nnLevelAdjust = 1.0;  // TODO Use level adjust to get model volumes even

  // Init the LEDs and set activate bypass
  led1.Init(hw.seed.GetPin(Funbox::LED_1), false);
  led1.Update();
  bypass = true;

  led2.Init(hw.seed.GetPin(Funbox::LED_2), false);
  led2.Update();

  hw.StartAdc();
  hw.StartAudio(AudioCallback);

  while (1) {
    System::Delay(100);
  }
}