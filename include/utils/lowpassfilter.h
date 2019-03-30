// A basic low pass filter to be used to refine magnetometer estimates


class FirstOrderLowPass{
    public: 
    FirstOrderLowPass(int timeConstantMil, int samplePeriodMil); 
    double ProcessSample(float sample);
    private:
    float state; 
    float alpha;  
};
