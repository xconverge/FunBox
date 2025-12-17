#include "funbox_hardware.h"

using namespace daisy;

#ifndef SAMPLE_RATE
// #define SAMPLE_RATE DSY_AUDIO_SAMPLE_RATE
#define SAMPLE_RATE 48014.f
#endif

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

// Encoder
constexpr Pin ENC_A_PIN = seed::D28;
constexpr Pin ENC_B_PIN = seed::D27;
constexpr Pin ENC_CLICK_PIN = seed::D24;

// Knobs
constexpr Pin PIN_EXPRESSION = seed::D15;
constexpr Pin PIN_KNOB_1 = seed::D16;
constexpr Pin PIN_KNOB_2 = seed::D19;
constexpr Pin PIN_KNOB_3 = seed::D17;
constexpr Pin PIN_KNOB_4 = seed::D20;
constexpr Pin PIN_KNOB_5 = seed::D18;
constexpr Pin PIN_KNOB_6 = seed::D21;

static constexpr I2CHandle::Config petal_led_i2c_config = {
    I2CHandle::Config::Peripheral::I2C_1,
    {Pin(PORTB, 8), Pin(PORTB, 9)},
    I2CHandle::Config::Speed::I2C_1MHZ};

enum LedOrder {
  LED_RING_1_R,
  LED_RING_1_G,
  LED_RING_1_B,
  LED_RING_5_R,
  LED_RING_5_G,
  LED_RING_5_B,
  LED_RING_2_R,
  LED_RING_2_G,
  LED_RING_2_B,
  LED_RING_6_R,
  LED_RING_6_G,
  LED_RING_6_B,
  LED_RING_3_R,
  LED_RING_3_G,
  LED_RING_3_B,
  LED_FS_1,
  LED_RING_4_R,
  LED_RING_4_G,
  LED_RING_4_B,
  LED_RING_7_R,
  LED_RING_7_G,
  LED_RING_7_B,
  LED_RING_8_R,
  LED_RING_8_G,
  LED_RING_8_B,
  LED_FS_2,
  LED_FS_3,
  LED_FS_4,
  LED_FAKE1,
  LED_FAKE2,
  LED_FAKE3,
  LED_FAKE4,
  LED_LAST,
};

static LedDriverPca9685<2, true>::DmaBuffer DMA_BUFFER_MEM_SECTION
    petal_led_dma_buffer_a,
    petal_led_dma_buffer_b;

void FunboxHardware::Init(bool boost) {
  // Set Some numbers up for accessors.
  // Initialize the hardware.
  seed.Configure();
  seed.Init(boost);
  InitSwitches();
  // InitEncoder();  // Comment out this since not used on Funbox?
  InitLeds();
  InitAnalogControls();
  SetAudioBlockSize(48);
  // seed.usb_handle.Init(UsbHandle::FS_INTERNAL);
  // InitMidi(); // Could initialize midi here, but for now leaving it up to the
  // individual effect whether midi is used or not
}

void FunboxHardware::DelayMs(size_t del) { seed.DelayMs(del); }

void FunboxHardware::SetHidUpdateRates() {
  for (size_t i = 0; i < KNOB_LAST; i++) {
    knob[i].SetSampleRate(AudioCallbackRate());
  }
  for (size_t i = 0; i < FOOTSWITCH_LED_LAST; i++) {
    footswitch_led[i].SetSampleRate(AudioCallbackRate());
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
  encoder.Debounce();
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

void FunboxHardware::ClearLeds() {
  // Using Color
  //    Color c;
  //    c.Init(Color::PresetColor::OFF);
  //    for(size_t i = 0; i < RING_LED_LAST; i++)
  //    {
  //        ring_led[i].SetColor(c);
  //    }
  for (size_t i = 0; i < RING_LED_LAST; i++) {
    SetRingLed(static_cast<RingLed>(i), 0.0f, 0.0f, 0.0f);
  }
  for (size_t i = 0; i < FOOTSWITCH_LED_LAST; i++) {
    SetFootswitchLed(static_cast<FootswitchLed>(i), 0.0f);
  }
}

void FunboxHardware::UpdateLeds() { led_driver_.SwapBuffersAndTransmit(); }

void FunboxHardware::SetRingLed(RingLed idx, float r, float g, float b) {
  uint8_t r_addr[RING_LED_LAST] = {LED_RING_1_R, LED_RING_2_R, LED_RING_3_R,
                                   LED_RING_4_R, LED_RING_5_R, LED_RING_6_R,
                                   LED_RING_7_R, LED_RING_8_R};
  uint8_t g_addr[RING_LED_LAST] = {LED_RING_1_G, LED_RING_2_G, LED_RING_3_G,
                                   LED_RING_4_G, LED_RING_5_G, LED_RING_6_G,
                                   LED_RING_7_G, LED_RING_8_G};
  uint8_t b_addr[RING_LED_LAST] = {LED_RING_1_B, LED_RING_2_B, LED_RING_3_B,
                                   LED_RING_4_B, LED_RING_5_B, LED_RING_6_B,
                                   LED_RING_7_B, LED_RING_8_B};

  led_driver_.SetLed(r_addr[idx], r);
  led_driver_.SetLed(g_addr[idx], g);
  led_driver_.SetLed(b_addr[idx], b);
}
void FunboxHardware::SetFootswitchLed(FootswitchLed idx, float bright) {
  uint8_t fs_addr[FOOTSWITCH_LED_LAST] = {LED_FS_1, LED_FS_2, LED_FS_3,
                                          LED_FS_4};
  led_driver_.SetLed(fs_addr[idx], bright);
}

void FunboxHardware::InitSwitches() {
  //    // button1
  //    button1.Init(seed.GetPin(SW_1_PIN), callback_rate_);
  //    // button2
  //    button2.Init(seed.GetPin(SW_2_PIN), callback_rate_);
  //
  //    buttons[BUTTON_1] = &button1;
  //    buttons[BUTTON_2] = &button2;
  constexpr Pin pin_numbers[SW_LAST] = {
      SW_1_PIN, SW_2_PIN, SW_3_PIN, SW_4_PIN,  SW_5_PIN,  SW_6_PIN,
      SW_7_PIN, SW_8_PIN, SW_9_PIN, SW_10_PIN, SW_11_PIN, SW_12_PIN,
  };

  for (size_t i = 0; i < SW_LAST; i++) {
    switches[i].Init(pin_numbers[i]);
  }
}

void FunboxHardware::InitEncoder() {
  encoder.Init(ENC_A_PIN, ENC_B_PIN, ENC_CLICK_PIN);
}

void FunboxHardware::InitLeds() {
  // LEDs are on the LED Driver.

  // Need to figure out how we want to handle that.
  uint8_t addr[2] = {0x00, 0x01};
  I2CHandle i2c;
  i2c.Init(petal_led_i2c_config);
  led_driver_.Init(i2c, addr, petal_led_dma_buffer_a, petal_led_dma_buffer_b);
  ClearLeds();
  UpdateLeds();
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
