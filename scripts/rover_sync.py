#!/usr/bin/env python

import rospy
import rospkg
import roslaunch
import os
from rover_sm import RoverStateMachine
from nova_common.msg import *
from nova_common.srv import *

name = "None"
changeMission = False

# Class handling ROS behaviour of rover_sync node
class RoverSync:

  #--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
  # __init__():
  #    Initialise class.
  #--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--    
  def __init__(self, parent=None):
  
    rospy.init_node('rover_sync', disable_signals=False)

    self.req_change_mode_server = rospy.Service(
      '/core_rover/req_change_mode', ChangeMode,
      self.handleReqChangeMode)
    
    self.req_change_mission_server = rospy.Service(
      '/core_rover/req_change_mission', ChangeMission,
      self.handleReqChangeMission)

    self.rover_sm = RoverStateMachine() # Initialise state machine

  #--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
  # handleReqChangeMission():
  #   Service server handler for handling requests for Mission change.
  #--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..-- 
  def handleReqChangeMission(self, req):
	
    global name
    global changeMission 
    success = 0
       
    if    req.mission == 'STOP':
      #bash script here
      message = "STOPPING ALL NODES"
      success = 1
    else:
        if  req.mission == 'SENSORS':
          success = 1
          name = "sensors"
        elif  req.mission == 'ERT' or req.mission == 'EQP':
          success = 1
          name = "eqpORert"
        elif  req.mission == 'SCI':
          success = 1
          name = "sci"
        elif  req.mission == 'AUT':
          success = 1
          name = "aut"

        if success:
          changeMission = True
          rospy.set_param('/core_rover/Mission', req.mission)
          message = "Changing Mission to " + req.mission + "."
        else:
          message = "Unable to change Mission. " 
      
    return ChangeMissionResponse(success, message)

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
  
  global name
  global changeMission 
  rover_sync = RoverSync()
 
  rate = rospy.Rate(0.1)
  while not rospy.is_shutdown(): 

    if (name is not "None" and changeMission is True):
      run_id = rospy.get_param("/run_id")
      uuid = roslaunch.rlutil.get_or_generate_uuid(run_id, True)
      roslaunch.configure_logging(uuid)

      rospack = rospkg.RosPack() # Get the file path for nova_common
      path = rospack.get_path('nova_common')

      launch_file = [path + '/launch/{}.launch'.format(name)]

      launch = roslaunch.parent.ROSLaunchParent(uuid, launch_file)
      launch.start() # Start the launch file
      changeMission = False
      name = "None"
    rate.sleep()

#--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
# Initialiser.
#--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..-- 
if __name__ == '__main__':
  try:
    main()
  except rospy.ROSInterruptException:
    pass
