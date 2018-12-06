#!/usr/bin/env python
import rospy
import math
import time
from sensor_msgs.msg import NavSatFix, MagneticField
from webots_ros.srv import set_float
from nova_common.msg import *
from nova_common.srv import *


class DesPosClass(object):

    #--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
    # __init__():
    #   
    #    Initialise class.
    #--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--   

    def __init__(self, lat, lng):
        self.latitude = lat
        self.longitude = lng  

    #--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
    # set_coords
    #   
    #    Update location
    #--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--   
        
    def set_coords(self, lat, lng):
        self.latitude = lat
        self.longitude = lng


class RoveyPosClass(object):

    #--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
    # __init__():
    #   
    #    Initialise class.
    #--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--   
    
    def __init__(self, lat, lng, x, z):
        self.latitude = lat
        self.longitude = lng  
        self.x = x
        self.z = z
        
    #--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
    # set_coords():
    #   
    #    Update location
    #--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--   
    
    def set_coords(self, lat, lng):
        self.latitude = lat
        self.longitude = lng 

    #--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
    # set_orientation():
    #   
    #    Update the direction the object is facing
    #--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--
    
    def set_orientation(self, x, z):
        self.x = x
        self.z = z  

        
#--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
# Bearing_In_Degrees():
#   
#    Calculates the direction an object is pointing in relation to the
#    north vector    
#--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..-- 
def Bearing_In_Degrees(x, z):
    rad = math.atan2(x,z)
    radnorth = math.atan2(1,0) # north vector 1,0,0
    bearing = (rad-radnorth)/math.pi*180 

    if bearing < 0:
        bearing = bearing + 360.0
    return bearing;


#--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
# Angle_Between():
#   
#    Calculates the bearing between the location of two objects, similar
#    to a Cartesian plane
#--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..-- 
def Angle_Between(lat1, lng1, lat2, lng2):
    beta = math.atan2(lng2-lng1, lat2-lat1)
    beta = (beta*180)/math.pi
    beta = beta +90
    if beta < 0:
		    beta = beta + 360
    return beta
    
#--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
# Distance_Between():
#   
#    Calculates the direct distance between two objects
#--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..-- 
def Distance_Between(lat1, lng1, lat2, lng2):
    distance = math.sqrt((lat2-lat1)**2 + (lng2-lng1)**2)
    return distance
    

#--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
# Turn_Direction():
#   
#    Calculates the direction and number of degrees the rover has to
#    turn to face the destination
#--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--     
def Turn_Direction(beta, orientation):
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
            
    rospy.logdebug("turn: %s", turn)        
    return turn


#--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
# gps_callback():
#   
#    Callback for the location of the rover
#--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--      
def gps_callback(gpsData):
    global roveyPos
    lat = gpsData.latitude
    lng = gpsData.longitude
    roveyPos.set_coords(lat,lng)
    #rospy.logdebug("lat: %s, long: %s", lat, lng)


#--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
# compass_callback():
#   
#    Callback for the orientation of the rover
#--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--  
def compass_callback(compassData):
    global roveyPos
    x = compassData.magnetic_field.x
    z = compassData.magnetic_field.z
    roveyPos.set_orientation(x,z)
    #rospy.logdebug("x: %s, z: %s", x, z)


#--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
# handle_start_auto():
#
#  Service server handler for starting autonomous mission.
#--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..-- 
def handle_start_auto(req):
    global desPos
    global auto_engaged

    # Set the desired latitude and longitude from the service request
    desPos.set_coords(req.latitude, req.longitude)
    auto_engaged = True

    res = StartAutoResponse()
    res.success = True
    return res


roveyPos = RoveyPosClass(0,0,0,0)
desPos = DesPosClass(0, 0)
auto_engaged = False   # Flag variable for enabling autonomous mode

#--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
# auto():
#   
#    Main function
#--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--
def auto():

    global roveyPos
    global desPos
    global auto_engaged
    
    rospy.init_node('auto', anonymous=True)
    rate = rospy.Rate(2) # Loop rate in Hz
    	
    #route = [ [22.5, 24.2], [-10, 0]]
    #how to get north direction from world info? or gps ref point?
    
    gps_sub = rospy.Subscriber("/pioneer3at/gps/values", NavSatFix, gps_callback)
    compass_sub = rospy.Subscriber("/pioneer3at/compass/values", MagneticField, compass_callback)
    pub = rospy.Publisher('/core_rover/driver/drive_cmd', DriveCmd, queue_size=10)

    server = rospy.Service('/core_rover/start_auto', StartAuto, handle_start_auto)
    
    while not rospy.is_shutdown():
        
        if auto_engaged is True:
            beta = Angle_Between(roveyPos.latitude, roveyPos.longitude, desPos.latitude, desPos.longitude)
            distance = Distance_Between(roveyPos.latitude, roveyPos.longitude, desPos.latitude, desPos.longitude)
            orientation = Bearing_In_Degrees(roveyPos.x, roveyPos.z)
            
            rospy.logdebug("beta: %s", beta)
            rospy.logdebug("distance: %s", distance)
            rospy.logdebug("orientation: %s", orientation)
            
            turn = Turn_Direction(beta, orientation)

            rpm_limit   = rospy.get_param('RPM_limit')
            steer_limit = rospy.get_param('steer_limit')
            
            msg = DriveCmd()
            msg.rpm       = rpm_limit*10
            msg.steer_pct = steer_limit*10*turn/180
            pub.publish(msg)
            
            #if distance < 3:
            #   desPos.set_coords(route[1][0], route[1][1])    

        rate.sleep()

    route = [ [22.5, 24.2], [-10, 0]]
    
    desPos = DesPosClass(route[0][0], route[0][1])
    
    gps_sub     = rospy.Subscriber("/pioneer3at/gps/values", NavSatFix, gps_callback)
    compass_sub = rospy.Subscriber("/pioneer3at/compass/values", MagneticField, compass_callback)
    drive_pub   = rospy.Publisher("/core_rover/driver/drive_cmd", DriveCmd, queue_size=10)
    status_pub  = rospy.Publisher("/core_rover/auto_status", AutoStatus, queue_size=10)
    
    while not rospy.is_shutdown():
        
        beta = Angle_Between(roveyPos.latitude, roveyPos.longitude, desPos.latitude, desPos.longitude)
        distance = Distance_Between(roveyPos.latitude, roveyPos.longitude, desPos.latitude, desPos.longitude)
        orientation = Bearing_In_Degrees(roveyPos.x, roveyPos.z)
        turn = Turn_Direction(beta, orientation)
        
        rospy.loginfo("cat")
        rospy.loginfo("beta: %s", beta)
        rospy.loginfo("distance: %s", distance)
        rospy.loginfo("orientation: %s", orientation)
        
        rpm_limit   = rospy.get_param('RPM_limit')
        steer_limit = rospy.get_param('steer_limit')
        
        drive_msg = DriveCmd()
        drive_msg.rpm       = rpm_limit*10
        drive_msg.steer_pct = steer_limit*10*turn/180
        drive_pub.publish(drive_msg)
        
        status_msg = AutoStatus()
        status_msg.latitude  = roveyPos.latitude
        status_msg.longitude = roveyPos.longitude
        status_msg.bearing   = orientation
        status_pub.publish(status_msg)
        
        #if distance < 3:
        #   desPos.set_coords(route[1][0], route[1][1])    


#--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
#   Initialiser
#--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--
if __name__ == '__main__':
    auto()


