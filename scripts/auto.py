#!/usr/bin/env python
import rospy
import math
import time
from sensor_msgs.msg import NavSatFix, MagneticField
from webots_ros.srv import set_float
from nova_common.msg import *
from nova_common.srv import *

# Class to keep track of desired rover position
class DesPosClass(object):

    #--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
    # __init__():
    #    Initialise class.
    #--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--   
    def __init__(self, lat, lng):
        self.latitude = lat
        self.longitude = lng  

    #--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
    # setCoords():
    #    Update location
    #--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--  
    def setCoords(self, lat, lng):
        self.latitude = lat
        self.longitude = lng

# Class to keep track of rover position
class RoveyPosClass(object):

    #--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
    # __init__():
    #    Initialise class.
    #--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--  
    def __init__(self, lat, lng, x, z):
        self.latitude = lat
        self.longitude = lng  
        self.x = x
        self.z = z
        
    #--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
    # setCoords():
    #    Update location
    #--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--   
    def setCoords(self, lat, lng):
        self.latitude = lat
        self.longitude = lng 

    #--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
    # setOrientation():
    #    Update the direction the object is facing
    #--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--
    def setOrientation(self, x, z):
        self.x = x
        self.z = z  

#--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
# bearingInDegrees():
#    Calculates the direction an object is pointing in relation to the
#    north vector    
#--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..-- 
def bearingInDegrees(x, z):
    rad = math.atan2(x,z)
    radnorth = math.atan2(1,0) # north vector 1,0,0
    bearing = (rad-radnorth)/math.pi*180 

    if bearing < 0:
        bearing = bearing + 360.0
    return bearing

#--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
# angleBetween():
#    Calculates the bearing between the location of two objects, similar
#    to a Cartesian plane
#--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..-- 
def angleBetween(lat1, lng1, lat2, lng2):
    beta = math.atan2(lng2-lng1, lat2-lat1)
    beta = (beta*180)/math.pi
    beta = beta +90
    if beta < 0:
		    beta = beta + 360
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
            
    rospy.logdebug("turn: %s", turn)        
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
# compassCallback():
#   
#    Callback for the orientation of the rover
#--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--  
def compassCallback(compassData):
    global rovey_pos
    x = compassData.magnetic_field.x
    z = compassData.magnetic_field.z
    rovey_pos.setOrientation(x,z)
    #rospy.logdebug("x: %s, z: %s", x, z)

#--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
# getMode(): Retrieve Mode from parameter server.
#--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..-- 
def getMode():
    return rospy.get_param('/core_rover/Mode')

#--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
# handleStartAuto():
#  Service server handler for starting autonomous mission.
#--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..-- 
def handleStartAuto(req):
    global des_pos
    global auto_engaged

    if getMode() == 'Auto':
        # Set the desired latitude and longitude from the service request
        des_pos.setCoords(req.latitude, req.longitude)
        auto_engaged = True

        # TODO Create and use state machine for autonomous mission
        # Reset state machine here

        return StartAutoResponse(True, 
            "Successfully started Auto mission.")
    else:
        return StartAutoResponse(False, 
            "Unable to start mission, must be in Auto mode.")


rovey_pos = RoveyPosClass(0,0,0,0)
des_pos = DesPosClass(0, 0)
auto_engaged = False   # Flag variable for enabling autonomous mode

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
    	
    #route = [ [22.5, 24.2], [-10, 0]]
    #how to get north direction from world info? or gps ref point?

    server = rospy.Service('/core_rover/start_auto', StartAuto, handleStartAuto)

    gps_sub     = rospy.Subscriber("/pioneer3at/gps/values", NavSatFix, gpsCallback)
    compass_sub = rospy.Subscriber("/pioneer3at/compass/values", MagneticField, compassCallback)
    drive_pub   = rospy.Publisher("/core_rover/driver/drive_cmd", DriveCmd, queue_size=10)
    status_pub  = rospy.Publisher("/core_rover/auto_status", AutoStatus, queue_size=10)
    
    while not rospy.is_shutdown():

        orientation = bearingInDegrees(rovey_pos.x, rovey_pos.z)

        if auto_engaged is True:
        
            beta = angleBetween(rovey_pos.latitude, rovey_pos.longitude, des_pos.latitude, des_pos.longitude)
            distance = distanceBetween(rovey_pos.latitude, rovey_pos.longitude, des_pos.latitude, des_pos.longitude)
            turn = turnDirection(beta, orientation)
            
            rospy.loginfo("cat")
            rospy.loginfo("beta: %s", beta)
            rospy.loginfo("distance: %s", distance)
            rospy.loginfo("orientation: %s", orientation)
            
            rpm_limit   = rospy.get_param('rpm_limit')
            steer_limit = rospy.get_param('steer_limit')
            
            drive_msg = DriveCmd()
            drive_msg.rpm       = rpm_limit*10
            drive_msg.steer_pct = steer_limit*10*turn/180
            drive_pub.publish(drive_msg)
            
            #if distance < 3:
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

