#pragma once
#include "DaisyDuino.h"
#include "SynthHardware.h"
#include "vs_osc.h"
#include "vs_lfo.h"

static inline float SoftClipTanh3(float x);

class Voice
{
public:
  void Init(float sample_rate);

  void NoteOn(byte inChannel, byte inNote, byte inVelocity);
  void NoteOff(byte inChannel, byte inNote, byte inVelocity);

  void ProcessBlock(float **out, size_t size);

  // called at control-rate from outside
  void UpdateParamsFromHardware(const SynthHardware &hw);

private:
  /* VCO */
  VS_Osc osc_;
  float current_freq_ = 0.0f;
  float current_vel_ = 1.0f;

  /* VCF */
  MoogLadder flt_;
  float base_cutoff_ = 1000.0f;
  float flt_drive_;
  float env_cutoff_depth_ = 0.0f;
  float lfo_cutoff_depth_ = 0.0f;
  float ComputeCutoff(float env, float lfo);

  /* LFO */
  VS_Lfo lfo_;

  /* ADSR */
  Adsr env_amp_;
  Adsr env_rel_;
  bool gate_ = false;
  AmpMode amp_mode_ = AMP_MODE_ADSR;
  float ComputeAmp(float env);
  // ADSR SHAPING PARAMS + HELPERS
  const float A_MIN = 0.002f, A_MAX = 2.f, A_CURVE = .7f;
  const float D_MIN = 0.003f, D_MAX = 1.5f, D_CURVE = .5f;
  const float R_MIN = 0.01f, R_MAX = 3.0f, R_CURVE = .5f;
  float MapKnobToTime(float knob, float t_min, float t_max, float curve);
};
