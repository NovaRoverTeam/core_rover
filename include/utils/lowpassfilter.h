// A basic low pass filter to be used to refine magnetometer estimates
#include "../Fusion/Fusion.h"

class FirstOrderLowPass{
    public: 
    FirstOrderLowPass(int timeConstantMil, int samplePeriodMil); 
    FusionVector3 FusionVector3LPFilter(FusionVector3 new_sample);
    private:
    FusionVector3 state; 
    float alpha;  
};
