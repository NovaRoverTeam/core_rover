## A class that applies a PID controller to the steering of the nova rover 
# in order to keep motion smooth and 
import math
class steeringPID:
    def __init__(self, first_value,frequency):
        self.turn_est = first_value
        self.freq = frequency
        self.filteredTurn = lpf(1,frequency,first_value)
        self.kp = 0.1 
        self.kI = 0.03
        self.kd = 0.4
        self.kiMax = 1
        self.turn_est = 0 
        self.freq = 1
        self.derTerm = 0
        self.propTerm = 0.0
        self.intTerm = 0.0
    def ComputeTurn(self, raw_command):
        self.computeder(raw_command)
        self.filteredTurn.stateUpdate(raw_command)
        self.propTerm = self.filteredTurn.state
        self.computeInt(raw_command)
        self.turn_est = self.kd*self.derTerm + self.kp*self.propTerm + self.kI*self.intTerm
    
    def computeder(self,raw_value):
        return self.freq* (raw_value - self.turn_est)*-1

    def computeInt(self,raw_value):
        self.intTerm = self.intTerm + raw_value
        if self.intTerm*self.intTerm > self.kiMax:
            self.intTerm = 1
## Usage of a lpf here to reduce the likelihood of large spikes in the yaw estimate 
# Causing hard turning of the motors, and possibly damaging them.
class lpf:
    def __init__(self, timeConstant, freq, initialValue):
        self.timeConstant = timeConstant
        self.freq = freq
        self.state = initialValue
        self.a = math.exp(-self.timeConstant*self.freq)


    def stateUpdate(self,newValue):
        self.state = self.a*newValue + (1-self.a)*self.state

    