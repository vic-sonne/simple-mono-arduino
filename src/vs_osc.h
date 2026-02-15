#pragma once
#include "DaisyDuino.h"
#include "SynthHardware.h"

/**
 * waveshaping helpers
 */
static inline float SoftClipCubic(float x);
static inline float SatOneOver(float x);
static inline float AsymBend(float x);
static inline float WaveShaper4(float x, float shape);
static inline float WaveFold(float smp, float amount);

class VS_Osc
{
public:
    void Init(float sample_rate);
    float Process(float frequency, float env, float lfo);

    // called at control-rate from outside
    void UpdateParamsFromHardware(const SynthHardware &hw);

private:
    /* VCO */
    VariableShapeOscillator osc_;
    VariableSawOscillator saw_osc_;
    OscType osc_type_;
    float osc_param_;
    float env_osc_depth_, lfo_osc_depth_;
    float fold_amt_, sync_amt_, pw_amt_;
    float ProcessPair2Anlg(float freq, float env, float lfo);
    float ProcessPair2Dgtl(float freq, float env, float lfo);
    void UpdatePair2Anlg();
    void UpdatePair2Dgtl();
    float ProcessPair3Anlg(float freq, float env, float lfo);
    float ProcessPair3Dgtl(float freq, float env, float lfo);
    void UpdatePair3Anlg();
    void UpdatePair3Dgtl();
    float ProcessSquare(float freq, float env, float lfo);
};
