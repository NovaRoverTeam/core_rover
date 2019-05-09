#!/usr/bin/env python
import rospy
import math
import time
from sensor_msgs.msg import NavSatFix, MagneticField, Imu
from webots_ros.srv import set_float
from nova_common.msg import *
from nova_common.srv import *

class RoveyPosClass(object):

    def __init__(self, lat, lng, roll, pitch, yaw):
        self.latitude = lat
        self.longitude = lng  
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw
          
    def setCoords(self, lat, lng):
        self.latitude = lat
        self.longitude = lng 

    def setOrientation(self, roll, pitch, yaw):
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw  

def gpsCallback(gpsData):
    global rovey_pos
    lat = gpsData.latitude
    lng = gpsData.longitude
    rovey_pos.setCoords(lat,lng)
    
def rpyCallback(rpyData):
    global rovey_pos
    rovey_pos.setOrientation(rpyData.roll, rpyData.pitch, rpyData.yaw)
    
rovey_pos = RoveyPosClass(0,0,0,0,0)
    
def sensors_pub():

    global rovey_pos

    rospy.init_node('sensors_pub', anonymous=True)
    rate = rospy.Rate(2) # Loop rate in Hz
 
    gps_sub     = rospy.Subscriber("/nova_common/gps_data", NavSatFix, gpsCallback)
    rpy_sub     = rospy.Subscriber("/nova_common/RPY", RPY, rpyCallback)
    status_pub  = rospy.Publisher("/core_rover/auto_status", AutoStatus, queue_size=10)
    
    while not rospy.is_shutdown():

      status_msg = AutoStatus()
      status_msg.latitude  = rovey_pos.latitude
      status_msg.longitude = rovey_pos.longitude
      status_msg.bearing   = rovey_pos.yaw
      status_pub.publish(status_msg)   

      rate.sleep()

if __name__ == '__main__':
    sensors_pub()
