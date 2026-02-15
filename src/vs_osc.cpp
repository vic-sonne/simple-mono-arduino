#include "vs_osc.h"

#define OSC_PAIR 3

void VS_Osc::Init(float sample_rate)
{
    osc_.Init(sample_rate);
    saw_osc_.Init(sample_rate);
}

/**
 * audio-rate processing
 */
float VS_Osc::Process(float frequency, float env, float lfo)
{
    float samp;
    switch (osc_type_)
    {
    case OSC_TYPE_SQ:
        samp = ProcessSquare(frequency, env, lfo);
        break;
    case OSC_TYPE_TRI:
#if OSC_PAIR == 2
        samp = ProcessPair2Dgtl(frequency, env, lfo);
#else
        samp = ProcessPair3Dgtl(frequency, env, lfo);
#endif
        break;
    default:
#if OSC_PAIR == 2
        samp = ProcessPair2Anlg(frequency, env, lfo);
#else
        samp = ProcessPair3Anlg(frequency, env, lfo);
#endif
        break;
    }
    return samp;
}

float VS_Osc::ProcessSquare(float freq, float env, float lfo)
{
    osc_.SetSyncFreq(freq);
    float pwm_amt = pw_amt_ + env * env_osc_depth_ + lfo * lfo_osc_depth_;
    pwm_amt = fclamp(pwm_amt, 0.f, 1.f);
    osc_.SetPW(pwm_amt);
    return osc_.Process();
}

float VS_Osc::ProcessPair2Anlg(float freq, float env, float lfo)
{
    const float maxModOct = 5.f;
    const float maxSemi = 38.0f;
    float env_oct = (env * env_osc_depth_ * maxSemi) / 12.f;
    float lfo_oct = lfo_osc_depth_ * maxModOct * lfo;
    float frequency = freq * exp2f(env_oct + lfo_oct);
    saw_osc_.SetFreq(frequency);
    return saw_osc_.Process();
}

float VS_Osc::ProcessPair2Dgtl(float freq, float env, float lfo)
{
    if (osc_param_ > 0.f)
    {
        osc_.SetFreq(freq);
        const float maxModOct = 5.f;
        const float maxSemi = 38.0f;
        const float base_oct = sync_amt_ * maxModOct;
        const float env_oct = (env * env_osc_depth_ * maxSemi) / 12.f;
        const float lfo_oct = lfo_osc_depth_ * maxModOct * lfo;
        float total_oct = base_oct + env_oct + lfo_oct;
        total_oct = fclamp(total_oct, 0.f, maxModOct);
        const float ratio = exp2f(total_oct);
        osc_.SetSyncFreq(freq * ratio);
        return osc_.Process();
    }
    else if (osc_param_ < 0.f)
    {
        osc_.SetFreq(freq);
        osc_.SetSyncFreq(freq);
        float total_fold_amt = fold_amt_ + env * env_osc_depth_ + lfo * lfo_osc_depth_;
        total_fold_amt = fclamp(total_fold_amt, 0.f, 1.f);
        float smp = osc_.Process();
        return WaveFold(smp, total_fold_amt);
    }
    else
    {
        osc_.SetFreq(freq);
        osc_.SetSyncFreq(freq);
        return osc_.Process();
    }
}

float VS_Osc::ProcessPair3Anlg(float freq, float env, float lfo)
{
    float x = osc_param_;
    x += env * env_osc_depth_;
    x += lfo * lfo_osc_depth_;

    float half = (x > 0.0f) ? x : 0.0f;
    float full = fabsf(x);
    if (half > 1.0f)
        half = 1.0f;
    if (full > 1.0f)
        full = 1.0f;

    float harmonics = 0.5f + 0.5f * half;
    float timbre = full;
    float morph = 1.0f - 0.5f * full;

    saw_osc_.SetFreq(freq);
    saw_osc_.SetPW(morph);
    float s = saw_osc_.Process();

    float shaped = WaveShaper4(s, harmonics);
    float folded = WaveFold(shaped, timbre);

    return folded;
}

float VS_Osc::ProcessPair3Dgtl(float freq, float env, float lfo)
{
    const float maxModOct = 5.f;
    const float maxSemi = 38.0f;
    float env_oct = (env * env_osc_depth_ * maxSemi) / 12.f;
    float lfo_oct = lfo_osc_depth_ * maxModOct * lfo;
    float frequency = freq * exp2f(env_oct + lfo_oct);
    frequency = fclamp(frequency, 20, 18000);
    osc_.SetFreq(frequency);
    osc_.SetSyncFreq(frequency);
    return osc_.Process();
}

/**
 * waveshaping helpers / utilities
 */
static inline float SoftClipCubic(float x)
{
    // good sounding, cheap
    // clamp to avoid blowups if upstream drives too hard
    if (x > 1.5f)
        x = 1.5f;
    if (x < -1.5f)
        x = -1.5f;
    return x - (x * x * x) * 0.3333333f;
}

static inline float SatOneOver(float x)
{
    // x/(1+|x|): cheap saturator
    float ax = fabsf(x);
    return x / (1.0f + ax);
}

static inline float AsymBend(float x)
{
    // simple asymmetric bend:
    // push positives a bit harder than negatives
    float pos = SoftClipCubic(x * 1.2f);
    float neg = SoftClipCubic(x * 0.9f);
    return (x >= 0.0f) ? pos : neg;
}

static inline float Mix(float a, float b, float t)
{
    return a + (b - a) * t;
}

// shape in [0;1]
static inline float WaveShaper4(float x, float shape)
{
    // 0..3
    float s = shape * 3.0f;
    int i = (int)s;
    float f = s - (float)i;

    float y0, y1;
    switch (i)
    {
    default:
    case 0:
        y0 = x;
        y1 = SoftClipCubic(x);
        break;
    case 1:
        y0 = SoftClipCubic(x);
        y1 = SatOneOver(x);
        break;
    case 2:
        y0 = SatOneOver(x);
        y1 = AsymBend(x);
        break;
    }
    return Mix(y0, y1, f);
}

// amount in [0;1]
static inline float WaveFold(float smp, float amount)
{
    const float drive = 1.0f + amount * 12.0f;
    smp *= drive;
    // Fold into [-1, 1] using a triangle-wave folding map
    float y = fmodf(smp + 1.0f, 4.0f);
    if (y < 0.0f)
        y += 4.0f;
    y = (y < 2.0f) ? (y - 1.0f) : (3.0f - y);
    return y;
}

/**
 * control-rate updates
 */
void VS_Osc::UpdateParamsFromHardware(const SynthHardware &hw)
{
    osc_param_ = hw.GetPot(POT_OSC_PARAM);
    env_osc_depth_ = hw.GetPot(POT_ENV_OSC_AMT);
    lfo_osc_depth_ = hw.GetPot(POT_LFO_OSC_AMT);
    osc_type_ = hw.GetOscType();
    switch (osc_type_)
    {
    case OSC_TYPE_SQ:
        pw_amt_ = (osc_param_ + 1) * 0.5f;
        osc_.SetSync(false);
        osc_.SetWaveshape(1);
        break;
    case OSC_TYPE_SAW:
#if OSC_PAIR == 2
        UpdatePair2Anlg();
#else
        UpdatePair3Anlg();
#endif
        break;
    case OSC_TYPE_TRI:
#if OSC_PAIR == 2
        UpdatePair2Dgtl();
#else
        UpdatePair3Dgtl();
#endif
        break;
    }
}

void VS_Osc::UpdatePair2Anlg()
{
    // osc_param_ sweeps -1 to 1. on that interval, we want our waveshape
    // to go from 1 to 0
    saw_osc_.SetWaveshape(-((osc_param_ - 1) * 0.5f));
    if (osc_param_ >= 0.f)
    {
        // remap the range 0..1 to 1..0
        saw_osc_.SetPW(1 - osc_param_);
    }
    else
    {
        // used to remap sweep [0 to 0.5] to [0.5 to 1]
        // now we need to remap [-1 to 0] to [0.5 to 1]
        saw_osc_.SetPW(osc_param_ * 0.5f + 1.f);
    }
}

void VS_Osc::UpdatePair2Dgtl()
{
    osc_.SetWaveshape(0);
    osc_.SetSync(true);
    osc_.SetPW(.5f);
    if (osc_param_ > 0.f)
    {
        // used to remap 0.5..1 to 0..1
        // now need to remap 0..1 to 0..1
        sync_amt_ = osc_param_;
    }
    else if (osc_param_ < 0.f)
    {
        // now need to remap -1..0 to 1..0
        fold_amt_ = -osc_param_;
    }
    else
    {
        fold_amt_ = 0;
        sync_amt_ = 0;
    }
}

void VS_Osc::UpdatePair3Anlg()
{
    saw_osc_.SetWaveshape(1);
}

void VS_Osc::UpdatePair3Dgtl()
{
    // set to triangle shape
    osc_.SetWaveshape(0);
    osc_.SetPW(.5f);
}
