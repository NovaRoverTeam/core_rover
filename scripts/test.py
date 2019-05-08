## script to test the pid controller. 
import steeringPID
def pdiSteer(controller, rawCommand):
    controller.ComputeTurn(rawCommand)
    return controller.turn_est 
target = 1

pdiSteering = steeringPID(0,2)
while(true):
    print(pdiSteer(pdiSteering,target))