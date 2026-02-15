// Lfo.h
#pragma once
#include "DaisyDuino.h"
#include "SynthHardware.h"

class VS_Lfo
{
public:
  void Init(float sample_rate);
  float Process(float note_freq);
  void UpdateParamsFromHardware(const SynthHardware &hw);

private:
  /* GLOBAL LFO CONTROLS */
  LfoType type_;
  float lfo_rate_;

  /* SIGNAL LFO */
  Oscillator osc_;
  float const LFO_F_MAX = 100;
  float const LFO_F_MIN = 0.01;
  float const FM_RATIO_MIN = 0.5;
  float const FM_RATIO_MAX = 4.f;

  /* RANDOM LFO */
  float const RND_F_MAX = 50;
  float const RND_F_MIN = 1.f;
  SmoothRandomGenerator smooth_rnd_;
  ClockedNoise stepped_rnd_;

  /* NOISE */
  Tone noise_color_, low_noise_2_;
  WhiteNoise white_noise_;
  float color_freq_ = 200.f;
  float gain_high_, gain_low_;
  float ProcessColoredNoiseSample();
};
