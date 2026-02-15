#include "SynthHardware.h"

void SynthHardware::Init()
{
    hw_ = DAISY.init(DAISY_SEED, AUDIO_SR_48K);
    sample_rate_ = DAISY.get_samplerate();

    /* VCO */
    osc_param_ctl_.Init(OSC_PARAM_POT, sample_rate_);
    osc_param_param_.Init(osc_param_ctl_, -1, 1.f, Parameter::LINEAR);
    env_osc_amt_ctl_.Init(OSC_ENV_AMT_POT, sample_rate_);
    env_osc_amt_param_.Init(env_osc_amt_ctl_, -1, 1.f, Parameter::LINEAR);
    lfo_osc_amt_ctl_.Init(OSC_LFO_AMT_POT, sample_rate_);
    lfo_osc_amt_param_.Init(lfo_osc_amt_ctl_, 0.f, 1.f, Parameter::LINEAR);
    osc_tri_sw_.Init(sample_rate_, true, OSC_TRI_SW, INPUT_PULLUP);
    osc_sq_sw_.Init(sample_rate_, true, OSC_SQ_SW, INPUT_PULLUP);

    /* VCF */
    cutoff_ctl_.Init(CUTOFF_POT, sample_rate_);
    cutoff_param_.Init(cutoff_ctl_, 20.f, 18000.f, Parameter::LOGARITHMIC);
    reso_ctl_.Init(RESO_POT, sample_rate_);
    reso_param_.Init(reso_ctl_, 0.f, 0.93f, Parameter::LINEAR);
    /* VCF MOD */
    env_cutoff_amt_ctl_.Init(ENV_CUTOFF_AMT_POT, sample_rate_);
    env_cutoff_amt_param_.Init(env_cutoff_amt_ctl_, -1.f, 1.f, Parameter::LINEAR);
    lfo_cutoff_amt_ctl_.Init(LFO_CUTOFF_AMT_POT, sample_rate_);
    lfo_cutoff_amt_param_.Init(lfo_cutoff_amt_ctl_, 0.f, 1.f, Parameter::LINEAR);

    /* ADSR */
    a_ctl_.Init(ATTACK_POT, sample_rate_);
    d_ctl_.Init(DECAY_POT, sample_rate_);
    s_ctl_.Init(SUSTAIN_POT, sample_rate_);
    r_ctl_.Init(RELEASE_POT, sample_rate_);
    a_param_.Init(a_ctl_, 0, 1.f, Parameter::LINEAR);
    d_param_.Init(d_ctl_, 0, 1.f, Parameter::LINEAR);
    s_param_.Init(s_ctl_, 0, 1.f, Parameter::LINEAR);
    r_param_.Init(r_ctl_, 0, 1.f, Parameter::LINEAR);

    /* AMP MODE */
    adsr_sw_.Init(sample_rate_, true, AMP_ADSR_MODE_SW, INPUT_PULLUP);
    drone_sw_.Init(sample_rate_, true, AMP_DRONE_MODE_SW, INPUT_PULLUP);

    /* LFO */
    lfo_rate_ctl_.Init(LFO_RATE_POT, sample_rate_);
    lfo_rate_param_.Init(lfo_rate_ctl_, 0.f, 1.f, Parameter::LINEAR);
    lfo_sig_rand_sw_.Init(sample_rate_, true, LFO_SIG_RAND_SW, INPUT_PULLUP);
    lfo_shape_1_sw_.Init(sample_rate_, true, LFO_SHAPE_1_SW, INPUT_PULLUP);
    lfo_shape_3_sw_.Init(sample_rate_, true, LFO_SHAPE_3_SW, INPUT_PULLUP);
}

void SynthHardware::UpdateControls()
{
    UpdateVCO();
    UpdateVCF();
    UpdateAMP();
    UpdateLFO();
}

void SynthHardware::UpdateVCO()
{
    osc_param_ = osc_param_param_.Process();
    osc_param_ = DeadbandBipolar(osc_param_, 0.05f);
    osc_env_amt_ = env_osc_amt_param_.Process();
    osc_env_amt_ = DeadbandBipolar(osc_env_amt_, 0.05f);
    osc_lfo_amt_ = lfo_osc_amt_param_.Process();
    osc_lfo_amt_ = Deadband01(osc_lfo_amt_, 0.02f);
    osc_lfo_amt_ *= osc_lfo_amt_;
    osc_tri_sw_.Debounce();
    osc_sq_sw_.Debounce();
    if (osc_tri_sw_.Pressed())
        osc_type_ = OSC_TYPE_TRI;
    else if (osc_sq_sw_.Pressed())
        osc_type_ = OSC_TYPE_SQ;
    else
        osc_type_ = OSC_TYPE_SAW;
}

void SynthHardware::UpdateVCF()
{
    cutoff_ = cutoff_param_.Process();
    reso_ = reso_param_.Process();
    cutoff_env_amt_ = env_cutoff_amt_param_.Process();
    cutoff_lfo_amt_ = lfo_cutoff_amt_param_.Process();
}

void SynthHardware::UpdateAMP()
{
    att_ = a_param_.Process();
    dec_ = d_param_.Process();
    sus_ = s_param_.Process();
    rel_ = r_param_.Process();
    adsr_sw_.Debounce();
    drone_sw_.Debounce();
    if (adsr_sw_.Pressed())
        amp_mode_ = AMP_MODE_ADSR;
    else if (drone_sw_.Pressed())
        amp_mode_ = AMP_MODE_DRONE;
    else
        amp_mode_ = AMP_MODE_RELEASE;
}

void SynthHardware::UpdateLFO()
{
    lfo_rate_ = lfo_rate_param_.Process();
    lfo_sig_rand_sw_.Debounce();
    lfo_shape_1_sw_.Debounce();
    lfo_shape_3_sw_.Debounce();
    if (lfo_sig_rand_sw_.Pressed() && lfo_shape_1_sw_.Pressed())
    {
        lfo_type_ = LFO_TYPE_SIN;
    }
    else if (lfo_sig_rand_sw_.Pressed() && lfo_shape_3_sw_.Pressed())
    {
        lfo_type_ = LFO_TYPE_FM;
    }
    else if (lfo_sig_rand_sw_.Pressed()) // lfo shape 2 selected
    {
        lfo_type_ = LFO_TYPE_TRI;
    }
    else if (!lfo_sig_rand_sw_.Pressed() && lfo_shape_1_sw_.Pressed())
    {
        lfo_type_ = LFO_TYPE_STEPPED;
    }
    else if (!lfo_sig_rand_sw_.Pressed() && lfo_shape_3_sw_.Pressed())
    {
        lfo_type_ = LFO_TYPE_NOISE;
    }
    else // random type 2 selected
    {
        lfo_type_ = LFO_TYPE_SMOOTH;
    }
}

static inline float Deadband01(float x, float db)
{
    if (x <= db)
        return 0.f;
    return (x - db) / (1.f - db);
}

static inline float DeadbandBipolar(float x, float db)
{
    float ax = fabsf(x);

    if (ax <= db)
        return 0.f;

    float sign = (x > 0.f) ? 1.f : -1.f;
    return sign * (ax - db) / (1.f - db);
}

float SynthHardware::GetPot(PotId id) const
{
    switch (id)
    {
    case POT_OSC_PARAM:
        return osc_param_;
    case POT_ENV_OSC_AMT:
        return osc_env_amt_;
    case POT_LFO_OSC_AMT:
        return osc_lfo_amt_;
    case POT_CUTOFF:
        return cutoff_;
    case POT_RESO:
        return reso_;
    case POT_ENV_CUTOFF_AMT:
        return cutoff_env_amt_;
    case POT_LFO_CUTOFF_AMT:
        return cutoff_lfo_amt_;
    case POT_ATTACK:
        return att_;
    case POT_DECAY:
        return dec_;
    case POT_SUSTAIN:
        return sus_;
    case POT_RELEASE:
        return rel_;
    case POT_LFO_RATE:
        return lfo_rate_;
    default:
        return 0;
    }
}

OscType SynthHardware::GetOscType() const
{
    return osc_type_;
}

AmpMode SynthHardware::GetAmpMode() const
{
    return amp_mode_;
}

LfoType SynthHardware::GetLfoType() const
{
    return lfo_type_;
}
