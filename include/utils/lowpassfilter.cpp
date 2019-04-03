#include "lowpassfilter.h" 
#include <math.h>
FirstOrderLowPass::FirstOrderLowPass(int time_constant_millis,
                                           int sample_period_millis)
    : state(FUSION_VECTOR3_ZERO),
      alpha(exp(-(static_cast<double>(sample_period_millis)) /
                (static_cast<double>(time_constant_millis)))) {}

FusionVector3 FirstOrderLowPass::FusionVector3LPFilter(FusionVector3 new_sample) {
    state = FusionVectorAdd(FusionVectorMultiplyScalar(state, alpha), FusionVectorMultiplyScalar(new_sample, 1 - alpha));
    return state;
}
