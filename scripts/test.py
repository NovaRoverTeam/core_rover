## script to test the pid controller. 
from steeringPID import steeringPID
import time
def pdiSteer(controller, rawCommand):
    controller.ComputeTurn(rawCommand)
    print controller.turn_est 
    return controller.turn_est 
target = 1
state = 0
pdiSteering = steeringPID(0,2)
while(1):
    effort = pdiSteer(pdiSteering,target-state)
    state = state + effort
    print("effort was", effort)    
    print("State is:",state)
    time.sleep(0.1)