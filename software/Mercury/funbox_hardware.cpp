#include "funbox_hardware.h"

using namespace daisy;

// Hardware related defines.
// Switches
constexpr Pin SW_1_PIN = seed::D25;  // Footswitch 1
constexpr Pin SW_2_PIN = seed::D26;  // Footswitch 2
constexpr Pin SW_3_PIN = seed::D14;  // Switch 1  left
constexpr Pin SW_4_PIN = seed::D13;  // Switch 1  right
constexpr Pin SW_5_PIN = seed::D7;   // Switch 2  left
constexpr Pin SW_6_PIN = seed::D10;  // Switch 2  right
constexpr Pin SW_7_PIN = seed::D2;   // Switch 3  left
constexpr Pin SW_8_PIN = seed::D4;   // Switch 3  right
constexpr Pin SW_9_PIN = seed::D1;   // Dip Switch 1
constexpr Pin SW_10_PIN = seed::D3;  // Dip Switch 2
constexpr Pin SW_11_PIN =
    seed::D5;  // Dip Switch 3 (Funbox v2/v3, not available on v1 board)
constexpr Pin SW_12_PIN =
    seed::D6;  // Dip Switch 4 (Funbox v2/v3, not available on v1 board)

// Knobs
constexpr Pin PIN_EXPRESSION = seed::D15;
constexpr Pin PIN_KNOB_1 = seed::D16;
constexpr Pin PIN_KNOB_2 = seed::D19;
constexpr Pin PIN_KNOB_3 = seed::D17;
constexpr Pin PIN_KNOB_4 = seed::D20;
constexpr Pin PIN_KNOB_5 = seed::D18;
constexpr Pin PIN_KNOB_6 = seed::D21;

// LEDs
constexpr Pin PIN_LED_FS1 = seed::D22;
constexpr Pin PIN_LED_FS2 = seed::D23;

void FunboxHardware::Init(bool boost) {
  // Initialize the hardware.
  seed.Configure();
  seed.Init(boost);
  InitSwitches();
  InitLeds();
  InitAnalogControls();
  SetAudioBlockSize(48);
}

void FunboxHardware::DelayMs(size_t del) { seed.DelayMs(del); }

void FunboxHardware::SetHidUpdateRates() {
  for (size_t i = 0; i < KNOB_LAST; i++) {
    knob[i].SetSampleRate(AudioCallbackRate());
  }
  for (size_t i = 0; i < LED_LAST; i++) {
    leds[i].SetSampleRate(AudioCallbackRate());
  }
  expression.SetSampleRate(AudioCallbackRate());
}

void FunboxHardware::StartAudio(AudioHandle::InterleavingAudioCallback cb) {
  seed.StartAudio(cb);
}

void FunboxHardware::StartAudio(AudioHandle::AudioCallback cb) {
  seed.StartAudio(cb);
}

void FunboxHardware::ChangeAudioCallback(
    AudioHandle::InterleavingAudioCallback cb) {
  seed.ChangeAudioCallback(cb);
}

void FunboxHardware::ChangeAudioCallback(AudioHandle::AudioCallback cb) {
  seed.ChangeAudioCallback(cb);
}

void FunboxHardware::StopAudio() { seed.StopAudio(); }

void FunboxHardware::SetAudioBlockSize(size_t size) {
  seed.SetAudioBlockSize(size);
  SetHidUpdateRates();
}

size_t FunboxHardware::AudioBlockSize() { return seed.AudioBlockSize(); }

void FunboxHardware::SetAudioSampleRate(
    SaiHandle::Config::SampleRate samplerate) {
  seed.SetAudioSampleRate(samplerate);
  SetHidUpdateRates();
}

float FunboxHardware::AudioSampleRate() { return seed.AudioSampleRate(); }

float FunboxHardware::AudioCallbackRate() { return seed.AudioCallbackRate(); }

void FunboxHardware::StartAdc() { seed.adc.Start(); }

void FunboxHardware::StopAdc() { seed.adc.Stop(); }

void FunboxHardware::ProcessAnalogControls() {
  for (size_t i = 0; i < KNOB_LAST; i++) {
    knob[i].Process();
  }
  expression.Process();
}

float FunboxHardware::GetKnobValue(Knob k) {
  size_t idx;
  idx = k < KNOB_LAST ? k : KNOB_1;
  return knob[idx].Value();
}

float FunboxHardware::GetExpression() { return expression.Value(); }

void FunboxHardware::ProcessDigitalControls() {
  for (size_t i = 0; i < SW_LAST; i++) {
    switches[i].Debounce();
  }
}

void FunboxHardware::InitMidi() {
  MidiUartHandler::Config midi_config;
  midi_config.transport_config.rx = seed::D30;  // On Funbox v2 and v3 hardware
  midi_config.transport_config.tx = seed::D29;  // On Funbox v2 hardware only
  midi.Init(midi_config);
}

void FunboxHardware::SetLed(int ledID, float bright) {
  if (ledID >= 0 && ledID < LED_LAST) {
    leds[ledID].Set(bright);
  }
}

void FunboxHardware::UpdateLeds() {
  for (size_t i = 0; i < LED_LAST; i++) {
    leds[i].Update();
  }
}

void FunboxHardware::InitSwitches() {
  constexpr Pin pin_numbers[SW_LAST] = {
      SW_1_PIN, SW_2_PIN, SW_3_PIN, SW_4_PIN,  SW_5_PIN,  SW_6_PIN,
      SW_7_PIN, SW_8_PIN, SW_9_PIN, SW_10_PIN, SW_11_PIN, SW_12_PIN,
  };

  for (size_t i = 0; i < SW_LAST; i++) {
    switches[i].Init(pin_numbers[i]);
  }
}

void FunboxHardware::InitLeds() {
  constexpr Pin pin_numbers[LED_LAST] = {
      PIN_LED_FS1,
      PIN_LED_FS2,
  };

  for (size_t i = 0; i < LED_LAST; i++) {
    leds[i].Init(pin_numbers[i], false, AudioCallbackRate());
  }
}

void FunboxHardware::InitAnalogControls() {
  // Set order of ADCs based on CHANNEL NUMBER
  // KNOB_LAST + 1 because of Expression input
  AdcChannelConfig cfg[KNOB_LAST + 1];
  // Init with Single Pins
  cfg[KNOB_1].InitSingle(PIN_KNOB_1);
  cfg[KNOB_2].InitSingle(PIN_KNOB_2);
  cfg[KNOB_3].InitSingle(PIN_KNOB_3);
  cfg[KNOB_4].InitSingle(PIN_KNOB_4);
  cfg[KNOB_5].InitSingle(PIN_KNOB_5);
  cfg[KNOB_6].InitSingle(PIN_KNOB_6);
  // Special case for Expression
  cfg[KNOB_LAST].InitSingle(PIN_EXPRESSION);

  seed.adc.Init(cfg, KNOB_LAST + 1);
  // Make an array of pointers to the knob.
  for (int i = 0; i < KNOB_LAST; i++) {
    knob[i].Init(seed.adc.GetPtr(i), AudioCallbackRate());
  }
  expression.Init(seed.adc.GetPtr(KNOB_LAST), AudioCallbackRate());
}
