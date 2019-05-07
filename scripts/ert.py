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

def distanceBetween(lat1, lng1, lat2, lng2):
    distance = math.sqrt((lat2-lat1)**2 + (lng2-lng1)**2)
    return distance
    
def turnDirection(beta, orientation):
    if beta < 180:
        if (orientation < (beta+180)) & (orientation > beta):
            rospy.logdebug("turn left")
            turn = (orientation-beta) * -1
        elif orientation < beta:
            rospy.logdebug("turn right")
            turn = beta-orientation
        else:
            rospy.logdebug("turn right")
            turn = (360-orientation)+beta
    else:
        if (orientation > (beta-180)) & (orientation < beta):
            rospy.logdebug("turn right")
            turn = beta-orientation
        elif orientation > beta:
            rospy.logdebug("turn left")
            turn = (orientation-beta) * -1
        else:
            rospy.logdebug("turn left")
            turn = ((360-beta)+orientation) * -1
            
    rospy.loginfo("turn: %s", turn)        
    return turn

def angleBetween(lat1, lng1, lat2, lng2):    
    lat1 = math.radians(lat1)
    lat2 = math.radians(lat2)

    longDiff = math.radians(lng2 - lng1)
    y = math.sin(longDiff) * math.cos(lat2)
    x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(longDiff)

    beta = math.atan2(y, x)
    beta = math.degrees(beta)
    beta = (beta+360) % 360

    return beta

def gpsCallback(gpsData):
    global rovey_pos
    lat = gpsData.latitude
    lng = gpsData.longitude
    rovey_pos.setCoords(lat,lng)
    #rospy.logdebug("lat: %s, long: %s", lat, lng)

def rpyCallback(rpyData):
    global rovey_pos
    rovey_pos.setOrientation(rpyData.roll, rpyData.pitch, rpyData.yaw)
    
rovey_pos = RoveyPosClass(0,0,0,0,0)
    
def ert():

    global rovey_pos

    rospy.init_node('auto', anonymous=True)
    rate = rospy.Rate(2) # Loop rate in Hz
 
    #how to get north direction from world info? or gps ref point?

    gps_sub     = rospy.Subscriber("/nova_common/gps_data", NavSatFix, gpsCallback)
    rpy_sub     = rospy.Subscriber("/nova_common/RPY", RPY, rpyCallback)
    status_pub  = rospy.Publisher("/core_rover/auto_status", AutoStatus, queue_size=10)

    waypointlat = float(input("Input lat pls:"))
    waypointlon = float(input("Input lon pls:"))
    time.sleep(5)
    while not rospy.is_shutdown():

    orientation = rovey_pos.yaw

    if (True):

        beta =angleBetween(rovey_pos.latitude, rovey_pos.longitude, waypointlat, waypointlon)
        distance = 110000 * distanceBetween(rovey_pos.latitude, rovey_pos.longitude, waypointlat, waypointlon)
        turn = turnDirection(beta, orientation)

        rospy.loginfo("fish")
        rospy.loginfo("beta: %s", beta)
        rospy.loginfo("distance: %s", distance)
        rospy.loginfo("orientation: %s", orientation)

    status_msg = AutoStatus()
    status_msg.latitude  = rovey_pos.latitude
    status_msg.longitude = rovey_pos.longitude
    status_msg.bearing   = orientation
    status_pub.publish(status_msg)   

    rate.sleep()

if __name__ == '__main__':
    auto()
