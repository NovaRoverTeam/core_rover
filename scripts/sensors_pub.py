#!/usr/bin/env python
import rospy
import math
import time
from sensor_msgs.msg import NavSatFix, MagneticField, Imu
from auto_classes import WaypointClass, RoveyPosClass
from webots_ros.srv import set_float
from nova_common.msg import *
from nova_common.srv import *

heading = 0
rovey_pos = RoveyPosClass(0,0,0,0,0)

def gpsCallback(gpsData):
    global rovey_pos
    lat = gpsData.latitude
    lng = gpsData.longitude
    rovey_pos.setCoords(lat,lng)
    
def rpyCallback(rpyData):
    global rovey_pos
    rovey_pos.setOrientation(rpyData.roll, rpyData.pitch, rpyData.yaw)

def compassCallback(compassData):
    global heading
    heading = compassData.approx_compass
    
def sensors_pub():

    global rovey_pos

    rospy.init_node('sensors_pub', anonymous=True)
    rate = rospy.Rate(2) # Loop rate in Hz
 
    gps_sub     = rospy.Subscriber("/nova_common/gps_data", NavSatFix, gpsCallback)
    rpy_sub     = rospy.Subscriber("/nova_common/RPY", RPY, rpyCallback)
    compass_sub = rospy.Subscriber("/nova_common/approx_compass", , compassCallback)
    status_pub  = rospy.Publisher("/core_rover/auto_status", AutoStatus, queue_size=10)
    
    while not rospy.is_shutdown():

      status_msg = AutoStatus()
      status_msg.latitude  = rovey_pos.latitude
      status_msg.longitude = rovey_pos.longitude
      status_msg.roll      = rovey_pos.roll
      status_msg.pitch     = rovey_pos.pitch
      status_msg.bearing   = rovey_pos.yaw
      status_msg.approx_heading = heading
      status_pub.publish(status_msg)   

      rate.sleep()

if __name__ == '__main__':
    sensors_pub()
