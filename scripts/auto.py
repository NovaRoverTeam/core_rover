#!/usr/bin/env python
import rospy
import math
import time
from sensor_msgs.msg import NavSatFix, MagneticField
from webots_ros.srv import set_float
from nova_common.msg import DriveCmd

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
            rospy.loginfo("turn left")
            turn = (orientation-beta) * -1
        elif orientation < beta:
            rospy.loginfo("turn right")
            turn = beta-orientation
        else:
            rospy.loginfo("turn right")
            turn = (360-orientation)+beta
    else:
        if (orientation > (beta-180)) & (orientation < beta):
            rospy.loginfo("turn right")
            turn = beta-orientation
        elif orientation > beta:
            rospy.loginfo("turn left")
            turn = (orientation-beta) * -1
        else:
            rospy.loginfo("turn left")
            turn = ((360-beta)+orientation) * -1
            
    rospy.loginfo("turn: %s", turn)        
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
    #rospy.loginfo("lat: %s, long: %s", lat, lng)

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
    #rospy.loginfo("x: %s, z: %s", x, z)


roveyPos = RoveyPosClass(0,0,0,0)

#--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
# auto():
#   
#    Main function
#--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--
def auto():

    global roveyPos
    
    rospy.init_node('auto', anonymous=True)
    	
    route = [ [30, -75], [-10, 0]]
    #how to get north direction from world info? or gps ref point?
    
    desPos = DesPosClass(route[0][0], route[0][1])
    
    gps_sub = rospy.Subscriber("/pioneer3at/gps/values", NavSatFix, gps_callback)
    compass_sub = rospy.Subscriber("/pioneer3at/compass/values", MagneticField, compass_callback)
    pub = rospy.Publisher('/core_rover/driver/drive_cmd', DriveCmd, queue_size=10)
    
    while True:
        rospy.loginfo("cat")
        beta = Angle_Between(roveyPos.latitude, roveyPos.longitude, desPos.latitude, desPos.longitude)
        distance = Distance_Between(roveyPos.latitude, roveyPos.longitude, desPos.latitude, desPos.longitude)
        orientation = Bearing_In_Degrees(roveyPos.x, roveyPos.z)
        
        rospy.loginfo("beta: %s", beta)
        rospy.loginfo("distance: %s", distance)
        rospy.loginfo("orientation: %s", orientation)
        
        turn = Turn_Direction(beta, orientation)
        
        msg = DriveCmd()
        msg.rpm = 10
        msg.steer_pct = turn/180
        pub.publish(msg)
        
        #if distance < 3:
        #   desPos.set_coords(route[1][0], route[1][1])    

        time.sleep(1)

#--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
#   Initialiser
#--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--
if __name__ == '__main__':
    auto()

