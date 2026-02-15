#pragma once
#include "DaisyDuino.h"

// VCO
#define OSC_PARAM_POT A0
#define OSC_ENV_AMT_POT A2
#define OSC_LFO_AMT_POT A1
#define OSC_TRI_SW D12
#define OSC_SQ_SW D13
// FILTER
#define CUTOFF_POT A5
#define RESO_POT A7
#define ENV_CUTOFF_AMT_POT A6
#define LFO_CUTOFF_AMT_POT A4
// ADSR ENV
#define ATTACK_POT A11
#define DECAY_POT A9
#define SUSTAIN_POT A10
#define RELEASE_POT A8
// AMP MODE
#define AMP_ADSR_MODE_SW D4
#define AMP_DRONE_MODE_SW D3
// LFO
#define LFO_RATE_POT A3
#define LFO_SIG_RAND_SW D11
#define LFO_SHAPE_1_SW D9
#define LFO_SHAPE_3_SW D10

enum PotId
{
  POT_OSC_PARAM,
  POT_ENV_OSC_AMT,
  POT_LFO_OSC_AMT,
  POT_CUTOFF,
  POT_RESO,
  POT_ENV_CUTOFF_AMT,
  POT_LFO_CUTOFF_AMT,
  POT_ATTACK,
  POT_DECAY,
  POT_SUSTAIN,
  POT_RELEASE,
  POT_LFO_RATE,
};

enum AmpMode
{
  AMP_MODE_ADSR,
  AMP_MODE_RELEASE,
  AMP_MODE_DRONE,
};

enum OscType
{
  OSC_TYPE_TRI,
  OSC_TYPE_SAW,
  OSC_TYPE_SQ,
};

enum LfoType
{
  LFO_TYPE_SIN,
  LFO_TYPE_TRI,
  LFO_TYPE_FM,
  LFO_TYPE_STEPPED,
  LFO_TYPE_SMOOTH,
  LFO_TYPE_NOISE,
};

static inline float Deadband01(float x, float db);
static inline float DeadbandBipolar(float x, float db);

class SynthHardware
{
public:
  void Init();
  void UpdateControls(); // call at control rate

  float GetPot(PotId id) const;
  AmpMode GetAmpMode() const;
  OscType GetOscType() const;
  LfoType GetLfoType() const;

  DaisyHardware &Raw() { return hw_; }

private:
  DaisyHardware hw_;
  float sample_rate_;

  /********             VCO             ********/
  // AnalogControl + Parameter for each pot
  AnalogControl osc_param_ctl_, env_osc_amt_ctl_, lfo_osc_amt_ctl_;
  Parameter osc_param_param_, env_osc_amt_param_, lfo_osc_amt_param_;
  // switches
  Switch osc_tri_sw_, osc_sq_sw_;
  // Helper
  void UpdateVCO();

  /********             VCF             ********/
  // AnalogControl + Parameter for each pot
  AnalogControl cutoff_ctl_, reso_ctl_, env_cutoff_amt_ctl_, lfo_cutoff_amt_ctl_;
  Parameter cutoff_param_, reso_param_, env_cutoff_amt_param_, lfo_cutoff_amt_param_;
  // Helper
  void UpdateVCF();

  /********             ADSR             ********/
  // AnalogControl + Parameter for each pot
  AnalogControl a_ctl_, d_ctl_, s_ctl_, r_ctl_;
  Parameter a_param_, d_param_, s_param_, r_param_;
  /********             AMP             ********/
  // switches
  Switch adsr_sw_, drone_sw_;
  // Helper
  void UpdateAMP();

  /********             LFO             ********/
  // AnalogControl + Parameter for each pot
  AnalogControl lfo_rate_ctl_;
  Parameter lfo_rate_param_;
  // switches
  Switch lfo_sig_rand_sw_, lfo_shape_1_sw_, lfo_shape_3_sw_;
  // Helper
  void UpdateLFO();

  // store last processed values
  float osc_param_, osc_env_amt_, osc_lfo_amt_;
  float cutoff_, reso_, cutoff_env_amt_, cutoff_lfo_amt_;
  float att_, dec_, sus_, rel_;
  float lfo_rate_;
  OscType osc_type_;
  AmpMode amp_mode_;
  LfoType lfo_type_;
};
