#!/usr/bin/env python

import rospy
from nova_common.msg import *
from nova_common.srv import *

class RoverManager:

  #--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
  # __init__():
  #
  #    Initialise class.
  #--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--    
  def __init__(self):

    rospy.init_node('rover_manager')

    self.initialise_state() # Set initial parameter server values

    self.drive_cmd_sub = rospy.Subscriber(
        '/core_rover/rover_manager/drive_cmd', DriveCmd,
         self.drive_cmd_cb)
         
    self.drive_cmd_pub = rospy.Publisher(
        '/core_rover/driver/drive_cmd', DriveCmd, 
         queue_size=1)

    self.radio_stat_sub = rospy.Subscriber(
        '/core_rover/radio_status', RadioStatus,
         self.radio_stat_cb)

    self.req_change_state_server = rospy.Service(
        '/core_rover/req_change_state', ChangeState,
         self.handle_req_change_state)

    # Connectivity status for ROS radio
    self.radio_connected = False


  #--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
  # handle_req_change_state():
  #
  #  Service server handler for handling requests for rover state change.
  #  The base station can request changes of the rover. The rover can 
  #  demand changes of the base station.
  #--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..-- 
  def handle_req_change_state(self, req):
  
    # TODO some conditions won't allow instantaneous state change
    rospy.set_param(req.state, req.value)    
    res = ChangeStateResponse(True, '')
    return res


  #--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
  # radio_stat_cb():
  #
  #    Callback for radio status messages to check connectivity.
  #--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--
  def radio_stat_cb(self, msg):
    radio_ros = rospy.get_param('RadioROS')
      
    if msg.radio_id is radio_ros: # If receiving message from ROS radio
     
      if msg.ssh_active and msg.n_wlan_cons > 0: # Record status
        self.radio_connected = True
      else:
        self.radio_connected = False


  #--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
  # drive_cmd_cb():
  #
  #    Callback for control msgs designated for Drive mode.
  #--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--
  def drive_cmd_cb(self, msg):
    mode = rospy.get_param('RoverMode')
    
    if (mode == 'Drive'): # Pass message on to driver node if all good
      self.drive_cmd_pub.publish(msg)


  #--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
  # initialise_state():
  #
  #    Set the relevant parameters in the server for the initial rover
  #    state, primarily the mode.
  #--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--
  def initialise_state(self):  
    rospy.set_param('RoverMode', 'Standby')


#--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
# Main():
#
#    Main function.
#--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--
def main():
  rover_manager = RoverManager()
  
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
