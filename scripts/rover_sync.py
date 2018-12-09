#!/usr/bin/env python

import rospy
from rover_sm import RoverStateMachine
from nova_common.msg import *
from nova_common.srv import *

# Class handling ROS behaviour of rover_sync node
class RoverSync:

  #--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
  # __init__():
  #    Initialise class.
  #--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--    
  def __init__(self):

    rospy.init_node('rover_sync')

    self.req_change_mode_server = rospy.Service(
      '/core_rover/req_change_mode', ChangeMode,
      self.handleReqChangeMode)
    
    self.req_change_mission_server = rospy.Service(
      '/core_rover/req_change_mission', ChangeMode,
      self.handleReqChangeMission)

    self.rover_sm = RoverStateMachine() # Initialise state machine

  #--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
  # handleReqChangeMission():
  #   Service server handler for handling requests for Mission change.
  #--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..-- 
  def handleReqChangeMission(self, req):

    # Change Mode to Standby
    res_mode = self.handleReqChangeMode(ChangeModeRequest('Standby'))

    if res_mode.success:
      rospy.set_param('/core_rover/Mission', req.mode)
      message = "Successfully changed Mission to " + req.mode + "."
    else:
      message = "Unable to change Mission. " + res_mode.message
      
    return ChangeModeResponse(res_mode.success, message)

  #--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
  # handleReqChangeMode():
  #   Service server handler for handling requests for rover mode change.
  #   The base station can request changes of the rover. The rover can 
  #   demand changes of the base station.
  #--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..-- 
  def handleReqChangeMode(self, req):

    if self.rover_sm.state == req.mode:
      return ChangeModeResponse(True, 'Already in ' + req.mode + ' Mode.')
  
    if    req.mode == 'Standby':
      success = self.rover_sm.setStandby()
    elif  req.mode == 'Drive':
      success = self.rover_sm.setDrive()
    elif  req.mode == 'Arm':
      success = self.rover_sm.setArm()
    elif  req.mode == 'Drill':
      success = self.rover_sm.setDrill()
    elif  req.mode == 'Auto':
      success = self.rover_sm.setAuto()

    if success:
      message = ('Successfully changed rover\'s Mode to ' 
        + self.rover_sm.state + '.')
    else:
      message = ('Unable to change rover\'s Mode to ' + req.mode 
        + '. Mode will remain as ' + self.rover_sm.state + '.')
        
    return ChangeModeResponse(success, message)

#--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
# main():
#    Main function.
#--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--
def main():
  rover_sync = RoverSync()
 
  rate = rospy.Rate(0.1)
  while not rospy.is_shutdown(): 

    rate.sleep()

#--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
# Initialiser.
#--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..-- 
if __name__ == '__main__':
  try:
    main()
  except rospy.ROSInterruptException:
    pass
