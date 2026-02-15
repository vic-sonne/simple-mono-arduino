#include "Voice.h"

void Voice::Init(float sample_rate)
{
  /* VCO */
  osc_.Init(sample_rate);

  /* VCF */
  flt_.Init(sample_rate);

  /* ADSR */
  env_amp_.Init(sample_rate);
  env_amp_.SetSustainLevel(1.f);
  env_amp_.SetTime(ADSR_SEG_ATTACK, A_MIN);
  env_amp_.SetTime(ADSR_SEG_DECAY, D_MIN);
  env_amp_.SetTime(ADSR_SEG_RELEASE, R_MIN);

  /* RELEASE-ONLY ENV (alternate amp mode) */
  env_rel_.Init(sample_rate);
  env_rel_.SetSustainLevel(1.f);
  env_rel_.SetTime(ADSR_SEG_ATTACK, A_MIN);
  env_rel_.SetTime(ADSR_SEG_DECAY, D_MIN);
  env_rel_.SetTime(ADSR_SEG_RELEASE, R_MIN);

  /* LFO */
  lfo_.Init(sample_rate);
}

void Voice::ProcessBlock(float **out, size_t size)
{
  for (size_t i = 0; i < size; i++)
  {
    float env = env_amp_.Process(gate_);
    float lfo = lfo_.Process(current_freq_);

    float cutoff = ComputeCutoff(env, lfo);
    flt_.SetFreq(cutoff);

    float sig = osc_.Process(current_freq_, env, lfo);
    float filt = flt_.Process(sig * flt_drive_);

    float amp = ComputeAmp(env);

    float s = SoftClipTanh3(filt * amp);
    out[0][i] = out[1][i] = s;
  }
}

float Voice::ComputeCutoff(float env, float lfo)
{
  const float maxModOct = 5.0f;
  float env_oct = env_cutoff_depth_ * maxModOct;
  float lfo_oct = lfo_cutoff_depth_ * maxModOct;
  float total_oct = (env * env_oct) + (lfo * lfo_oct);
  float cutoff = base_cutoff_ * exp2f(total_oct);
  if (cutoff < 20.f)
    cutoff = 20.f;
  if (cutoff > 18000.f)
    cutoff = 18000.f;
  return cutoff;
}

float Voice::ComputeAmp(float env)
{
  switch (amp_mode_)
  {
  case AMP_MODE_ADSR:
    return env;
  case AMP_MODE_DRONE:
    return 1.f;
  case AMP_MODE_RELEASE:
    return env_rel_.Process(gate_);
  default:
    // should never happen, but worst case, keeps amp to 0
    return 0;
  }
}

static inline float SoftClipTanh3(float x)
{
  // clamp to keep the approximation bounded + stable
  if (x > 3.f)
    x = 3.f;
  if (x < -3.f)
    x = -3.f;

  // tanh-ish approx: x * (27 + x^2) / (27 + 9x^2)
  float x2 = x * x;
  return x * (27.f + x2) / (27.f + 9.f * x2);
}

void Voice::NoteOn(byte inChannel, byte inNote, byte inVelocity)
{
  // Note Off can come in as Note On w/ 0 Velocity
  if (inVelocity == 0.f)
  {
    gate_ = false;
  }
  else
  {
    current_freq_ = mtof(inNote);
    gate_ = true;
  }
}

void Voice::NoteOff(byte inChannel, byte inNote, byte inVelocity)
{
  gate_ = false;
}

void Voice::UpdateParamsFromHardware(const SynthHardware &hw)
{
  /* VCO */
  osc_.UpdateParamsFromHardware(hw);
  /* VCF */
  base_cutoff_ = hw.GetPot(POT_CUTOFF);
  float reso = hw.GetPot(POT_RESO);
  flt_.SetRes(reso);
  flt_drive_ = 1 + reso * reso * 4;
  if (flt_drive_ > 3.f)
    flt_drive_ = 3.f;
  env_cutoff_depth_ = hw.GetPot(POT_ENV_CUTOFF_AMT);
  lfo_cutoff_depth_ = hw.GetPot(POT_LFO_CUTOFF_AMT);
  /* ADSR */
  env_amp_.SetSustainLevel(hw.GetPot(POT_SUSTAIN));
  float attack_s = MapKnobToTime(hw.GetPot(POT_ATTACK), A_MIN, A_MAX, A_CURVE);
  float decay_s = MapKnobToTime(hw.GetPot(POT_DECAY), D_MIN, D_MAX, D_CURVE);
  float release_s = MapKnobToTime(hw.GetPot(POT_RELEASE), R_MIN, R_MAX, R_CURVE);
  env_amp_.SetTime(ADSR_SEG_ATTACK, attack_s);
  env_amp_.SetTime(ADSR_SEG_DECAY, decay_s);
  env_amp_.SetTime(ADSR_SEG_RELEASE, release_s);
  env_rel_.SetTime(ADSR_SEG_RELEASE, release_s);
  /* VCA */
  amp_mode_ = hw.GetAmpMode();
  /* LFO */
  lfo_.UpdateParamsFromHardware(hw);
}

float Voice::MapKnobToTime(float knob, float t_min, float t_max, float curve)
{
  float shaped = powf(knob, curve);
  return t_min * powf(t_max / t_min, shaped);
}
