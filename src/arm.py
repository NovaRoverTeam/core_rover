
#!/usr/bin/env python
import rospy
import can
from std_msgs.msg import String
from nova_common.msg import * #motor_arm
from base_station.msg import *

bustype = 'socketcan_native'
channel = 'can0' #cantx is set virtual can, can set up by running the run_can.batch file in sim
bus = can.interface.Bus(channel=channel, bustype=bustype)  #Define the can bus interface to transmit on. 

def RightCallback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    rospy.loginfo(str(data.RPM))
#    msg = can.Message(arbitration_id=0x32, data=[0x32, 0x32], extended_id = False) 
#    can_bus_send(msg)

def LeftCallback(data):
   rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

def listener():

    rospy.init_node('arm', anonymous=True)

    rospy.Subscriber("/base_statoin/ljs_raw_ctrl", RawCtrl, RightCallback)
    rospy.Subscriber("/base_station/rjs_raw_ctrl", RawCtrl, LeftCallBack)

    rospy.spin()
    while(true):
    sock.recv()
    time.sleep(0.0001)

if __name__ == '__main__':
    listener()
