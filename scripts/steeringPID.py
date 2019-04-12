## A class that applies a PID controller to the steering of the nova rover 
# in order to keep motion smooth and 

class steeringPIDL:
    self.kp = 0.1 
    self.kI = 0.3
    self.kd = 0.01
    self.kiMax = 1
    self.turn_est = 0 
    self.freq = 1
    self.derTerm = 0
    self.propTerm = 0
    self.IntTerm = 0 

    def __init__(self, first_value,frequency):
        self.turn_est = first_value
        self.freq = frequency
    
    def ComputeTurn(self, raw_command):
        self.computeder(raw_command)
        self.propTerm = raw_command
        self.computeInt(raw_command)
        self.turn_est = self.kd*self.derTerm + self.kp*self.propTerm + self.kI*self.IntTerm
    
    def computeder(self,raw_value):
        return self.freq* (raw_value - self.turn_est)*-1

    def computeInt(self,raw_value):
        self.IntTerm = self.IntTerm + raw_value
        if self.IntTerm*self.IntTerm > self.kiMax:
            self.IntTerm = 1

    