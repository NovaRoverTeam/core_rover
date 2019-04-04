#include "lowpassfilter.h"
#include <math.h>
FirstOrderLowPass::FirstOrderLowPass(int time_constant_millis,
                                           int sample_period_millis)
    : state(0),
      alpha(exp(-(static_cast<double>(sample_period_millis)) /
                (static_cast<double>(time_constant_millis)))) {}

double FirstOrderLowPass::Float3LPFilter(float new_sample) {
    state = alpha * state + (1 - alpha) * new_sample;
    return state;
}
