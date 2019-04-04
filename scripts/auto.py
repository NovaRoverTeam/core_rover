#!/usr/bin/env python
import rospy
import math
import time
from sensor_msgs.msg import NavSatFix, MagneticField, Imu
from std_msgs.msg import Float32
from webots_ros.srv import set_float
from nav_msgs.msg import Odometry
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

    def __init__(self, lat, lng, x, y, z):
        self.latitude = lat
        self.longitude = lng  
        self.x = x
        self.y = y
        self.z = z
          
    def setCoords(self, lat, lng):
        self.latitude = lat
        self.longitude = lng 

    def setOrientation(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z  

#--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
# bearingInDegrees():
#    Calculates the direction an object is pointing in relation to the
#    north vector    
#--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..-- 
def bearingInDegrees(x, y):
    rad = math.atan2(y,x)
    bearing = (rad)/math.pi*180 -180

    if bearing < 0:
        bearing = bearing + 360.0
    return bearing

#--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
# angleBetween():
#    Calculates the bearing between the location of two objects, similar
#    to a Cartesian plane
#--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..-- 
def angleBetween(lat1, lng1, lat2, lng2):    
    #dLon = (lng2 - lng1)

    #y = math.sin(dLon) * math.cos(lat2)
    #x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dLon
    
    #beta = math.atan2(y,x)
    
    beta = math.atan2(lat2-lat1, lng2-lng1)
    beta = (beta*180)/math.pi
    if beta < 0:
            beta = beta + 360
    elif beta > 360:
            beta = beta - 360
            
    #beta = 360 - beta
    
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
# odometryCallback():
#    Callback for the orientation of the rover
#--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--      
def odometryCallback(odometryData):
    global rovey_pos
    x = imuData.orientation.x
    y = imuData.orientation.y
    z = imuData.orientation.z
    w = imuData.orientation.w       # previously, this was z; need to check if a bug
    roll, pitch,yaw =  quaternion_to_euler(x,y,z,w)
    rovey_pos.setOrientation(roll,pitch,yaw)

def quaternion_to_euler(x, y, z, w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    X = math.degrees(math.atan2(t0, t1))

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    Y = math.degrees(math.asin(t2))

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    Z = math.degrees(math.atan2(t3, t4))

    return X, Y, Z

#--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
# compassCallback():
#    Callback for the orientation of the rover
#--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--  
def compassCallback(compassData):
    global rovey_pos
    x = compassData.magnetic_field.x
    z = compassData.magnetic_field.z
    rovey_pos.setOrientation(x,z)
    #rospy.logdebug("x: %s, z: %s", x, z)

#--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
# headingCallback():
#    Callback for the orientation of the rover
#--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--  
def headingCallback(headingData):
    global orientation
    orientation = headingData.data 
    #rospy.logdebug("x: %s, z: %s", x, z)

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
orientation = 0
#--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
# auto():
#    Main function
#--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--
def auto():

    global rovey_pos
    global des_pos
    global auto_engaged
    global orientation
    
    rospy.init_node('auto', anonymous=True)
    rate = rospy.Rate(2) # Loop rate in Hz
 
    #how to get north direction from world info? or gps ref point?

    gps_sub     = rospy.Subscriber("/nova_common/gps_data", NavSatFix, gpsCallback)
    imu_sub     = rospy.Subscriber("/rtimulib_node/imu", Imu, imuCallback)
    compass_sub = rospy.Subscriber("/nova_common/MagnetometerFiltered", MagneticField, compassCallback)
        
    waypoint_sub = rospy.Subscriber("/core_rover/navigation/waypoint_coords", NavSatFix, waypointCallback)
    heading_sub = rospy.Subscriber("/nova_common/heading", Float32, headingCallback)
    drive_pub   = rospy.Publisher("/core_rover/driver/drive_cmd", DriveCmd, queue_size=10)
    status_pub  = rospy.Publisher("/core_rover/auto_status", AutoStatus, queue_size=10)
    
    while not rospy.is_shutdown():

        orientation = rovey_pos.z
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
        #status_msg.bearing   = orientation
        status_pub.publish(status_msg)   

        rate.sleep()

#--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
# Initialiser
#--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--
if __name__ == '__main__':
    auto()
