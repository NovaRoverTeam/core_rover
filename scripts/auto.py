#!/usr/bin/env python
import rospy
import math
import time
from sensor_msgs.msg import NavSatFix, MagneticField, Imu
from webots_ros.srv import set_float
from nova_common.msg import *
from nova_common.srv import *

#--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
# DesPosClass:
#    Creates a class for the GPS coords given by the competition.
#    This is updated by subscribing to the navigation node   
#--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..-- 
class WaypointClass(object):

    def __init__(self, lat, lng):
        self.latitude = lat
        self.longitude = lng  

    def setCoords(self, lat, lng):
        self.latitude = lat
        self.longitude = lng

#--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
# RoveyPosClass:
#    Creates a class for the location of the rover
#--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..-- 
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

#--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
# angleBetween():
#    Calculates the bearing in degrees between the location of two objects, similar
#    to a Cartesian plane
#--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..-- 
def angleBetween(lat1, lng1, lat2, lng2):    
    lat1 = math.radians(lat1)
    lat2 = math.radians(lat2)

    longDiff = math.radians(lng2 - lng1)
    y = math.sin(longDiff) * math.cos(lat2)
    x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(longDiff)

    beta = math.atan2(y, x)
    beta = math.degrees(bearing)
    beta = (bearing+360) % 360

    return beta
        
#--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
# distanceBetween():
#    Calculates the direct distance between two objects
#--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..-- 
def distanceBetween(lat1, lng1, lat2, lng2):
    distance = math.sqrt((lat2-lat1)**2 + (lng2-lng1)**2)
    return distance
    
#--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
# turnDirection():
#    Calculates the direction and number of degrees the rover has to
#    turn to face the destination
#--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--     
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

#--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
# gpsCallback():
#    Callback for the location of the rover
#--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--      
def gpsCallback(gpsData):
    global rovey_pos
    lat = gpsData.latitude
    lng = gpsData.longitude
    rovey_pos.setCoords(lat,lng)
    #rospy.logdebug("lat: %s, long: %s", lat, lng)

#--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
# rpyCallback():
#    Callback for roll, pitch, yaw from IMU
#--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--  
def rpyCallback(rpyData):
    global rovey_pos
    rovey_pos.setOrientation(rpyData.roll, rpyData.pitch, rpyData.yaw)

#--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
# waypointCallback():
#    Callback for the waypoint coords passed from navigation node
#--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--  
def waypointCallback(waypointData):
    global waypoint
    lat = waypointData.latitude
    lng = waypointData.longitude
    waypoint.setCoords(lat,lng)
    print(lat)

#--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
# getMode(): Retrieve Mode from parameter server.
#--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..-- 
#def getMode():
 #   return rospy.get_param('/core_rover/Mode')

 #--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
# Global variables
#--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..-- 
rovey_pos = RoveyPosClass(0,0,0,0,0)
waypoint = WaypointClass(0, 0)

#--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
# auto():
#    Main function
#--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--
def auto():

    global rovey_pos
    global des_pos
    global auto_engaged
    
    rospy.init_node('auto', anonymous=True)
    rate = rospy.Rate(2) # Loop rate in Hz
 
    #how to get north direction from world info? or gps ref point?

    gps_sub     = rospy.Subscriber("/nova_common/gps_data", NavSatFix, gpsCallback)
    rpy_sub     = rospy.Subscriber("/nova_common/RPY", RPY, rpyCallback)
    waypoint_sub = rospy.Subscriber("/core_rover/navigation/waypoint_coords", NavSatFix, waypointCallback)
    drive_pub   = rospy.Publisher("/core_rover/driver/drive_cmd", DriveCmd, queue_size=10)
    status_pub  = rospy.Publisher("/core_rover/auto_status", AutoStatus, queue_size=10)
    
    while not rospy.is_shutdown():

        orientation = rovey_pos.yaw

        if True:
        
            beta =0  #angleBetween(rovey_pos.latitude, rovey_pos.longitude, waypoint.latitude, waypoint.longitude)
            distance = distanceBetween(rovey_pos.latitude, rovey_pos.longitude, waypoint.latitude, waypoint.longitude)
            turn = turnDirection(beta, orientation)
            
            rospy.loginfo("fish")
            rospy.loginfo("beta: %s", beta)
            rospy.loginfo("distance: %s", distance)
            rospy.loginfo("orientation: %s", orientation)
            
            #rpm_limit   = rospy.get_param('rpm_limit')
           # steer_limit = rospy.get_param('steer_limit')
            
            drive_msg = DriveCmd()
            drive_msg.rpm       = 0
            drive_msg.steer_pct = turn * 0.3
            drive_pub.publish(drive_msg)
            
            #if distance < :
            #   desPos.set_coords(route[1][0], route[1][1])   

        # TODO adjust rate that AutoStatus is published
        status_msg = AutoStatus()
        status_msg.latitude  = rovey_pos.latitude
        status_msg.longitude = rovey_pos.longitude
        status_msg.bearing   = orientation
        status_pub.publish(status_msg)   

        rate.sleep()

#--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
# Initialiser
#--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--
if __name__ == '__main__':
    auto()

