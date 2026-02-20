#include "daisysp.h"
using namespace daisysp;

class TinyReverb {
 public:
  void Init(float sample_rate) {
    sr_ = sample_rate;
    dl_.Init();
    damp_.Init(sample_rate);
    Set(0.22f, 0.28f, 3200.0f);  // a bit stronger defaults
  }

  void Set(float mix, float feedback, float damp_hz) {
    mix_ = mix;      // 0..1 (we’ll keep it modest)
    fb_ = feedback;  // ~0.18..0.35
    damp_.SetFreq(damp_hz);

    tapL1_ = MsToSamps(11.3f);
    tapL2_ = MsToSamps(17.7f);
    tapL3_ = MsToSamps(29.5f);

    tapR1_ = MsToSamps(13.1f);
    tapR2_ = MsToSamps(19.9f);
    tapR3_ = MsToSamps(31.7f);

    tailL_ = MsToSamps(70.0f);
    tailR_ = MsToSamps(74.0f);
  }

  inline void Process(float in, float* wetL, float* wetR) {
    // slightly stronger early reflections (was 0.35/0.25/0.18)
    const float eL = 0.42f * dl_.Read(tapL1_) + 0.30f * dl_.Read(tapL2_) +
                     0.22f * dl_.Read(tapL3_);
    const float eR = 0.42f * dl_.Read(tapR1_) + 0.30f * dl_.Read(tapR2_) +
                     0.22f * dl_.Read(tapR3_);

    float tL = dl_.Read(tailL_);
    float tR = dl_.Read(tailR_);

    float fb = damp_.Process(0.5f * (tL + tR));
    dl_.Write(in + fb_ * fb);

    // stronger tail contribution (was 0.35f)
    const float tailGain = 0.55f;

    // apply mix_ here (previously unused!)
    *wetL = mix_ * (eL + tailGain * tL);
    *wetR = mix_ * (eR + tailGain * tR);
  }

 private:
  inline float MsToSamps(float ms) const { return ms * 0.001f * sr_; }

  float sr_ = 48000.0f;
  float mix_ = 0.22f;
  float fb_ = 0.28f;

  DelayLine<float, 4800> dl_;
  Tone damp_;

  float tapL1_, tapL2_, tapL3_;
  float tapR1_, tapR2_, tapR3_;
  float tailL_, tailR_;
};