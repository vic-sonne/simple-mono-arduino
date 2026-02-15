#include "vs_lfo.h"

void VS_Lfo::Init(float sample_rate)
{
    osc_.Init(sample_rate);
    smooth_rnd_.Init(sample_rate);
    stepped_rnd_.Init(sample_rate);
    white_noise_.Init();
    noise_color_.Init(sample_rate);
    noise_color_.SetFreq(color_freq_);
    low_noise_2_.Init(sample_rate);
    low_noise_2_.SetFreq(color_freq_);
}

float VS_Lfo::Process(float note_freq)
{
    float freq = lfo_rate_;
    float samp = 0;
    switch (type_)
    {
    case LFO_TYPE_SIN:
        samp = osc_.Process();
        break;
    case LFO_TYPE_TRI:
        samp = osc_.Process();
        break;
    case LFO_TYPE_FM:
        freq = note_freq * lfo_rate_;
        if (freq > 20000.f)
            freq = 20000.f;
        osc_.SetFreq(freq);
        samp = osc_.Process();
        break;
    case LFO_TYPE_STEPPED:
        samp = stepped_rnd_.Process();
        break;
    case LFO_TYPE_SMOOTH:
        samp = smooth_rnd_.Process();
        break;
    case LFO_TYPE_NOISE:
        samp = ProcessColoredNoiseSample();
        break;
    default:
        break;
    }
    return samp;
}

float VS_Lfo::ProcessColoredNoiseSample()
{
    gain_low_ = max(0.f, 1 - lfo_rate_);
    gain_high_ = max(0.f, 1 + lfo_rate_ * 0.5f);
    float x = white_noise_.Process();
    float lp = noise_color_.Process(x);
    float lp2 = low_noise_2_.Process(lp);
    float hp = x - lp;
    return 0.5f * (gain_low_ * lp2 + gain_high_ * hp);
}

void VS_Lfo::UpdateParamsFromHardware(const SynthHardware &hw)
{
    float lfo_knob = hw.GetPot(POT_LFO_RATE);
    type_ = hw.GetLfoType();
    switch (type_)
    {
    case LFO_TYPE_SIN:
        osc_.SetWaveform(Oscillator::WAVE_SIN);
        lfo_rate_ = LFO_F_MIN * powf(LFO_F_MAX / LFO_F_MIN, lfo_knob);
        osc_.SetFreq(lfo_rate_);
        break;
    case LFO_TYPE_TRI:
        osc_.SetWaveform(Oscillator::WAVE_TRI);
        lfo_rate_ = LFO_F_MIN * powf(LFO_F_MAX / LFO_F_MIN, lfo_knob);
        osc_.SetFreq(lfo_rate_);
        break;
    case LFO_TYPE_FM:
        osc_.SetWaveform(Oscillator::WAVE_SIN);
        lfo_rate_ = (FM_RATIO_MIN * powf(FM_RATIO_MAX / FM_RATIO_MIN, lfo_knob));
        break;
    case LFO_TYPE_STEPPED:
        lfo_rate_ = RND_F_MIN * powf(RND_F_MAX / RND_F_MIN, lfo_knob * lfo_knob);
        stepped_rnd_.SetFreq(lfo_rate_);
        break;
    case LFO_TYPE_SMOOTH:
        lfo_rate_ = RND_F_MIN * powf(RND_F_MAX / RND_F_MIN, lfo_knob * lfo_knob);
        smooth_rnd_.SetFreq(lfo_rate_);
        break;
    case LFO_TYPE_NOISE:
        lfo_rate_ = 2.f * lfo_knob - 1.f;
        lfo_rate_ = lfo_rate_ * lfo_rate_ * lfo_rate_;
        break;
    }
}
