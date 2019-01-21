#!/usr/bin/env python
import rospy
from sensor_msgs.msg import NavSatFix
from webots_ros.srv import set_float
from nova_common.msg import *
from nova_common.srv import *

#--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
# DesPosClass:
#    Creates a class for the GPS coords given by the competition.
#    This is updated by the GUI    
#--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..-- 
class DesPosClass(object):

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

    def __init__(self, lat, lng):
        self.latitude = lat
        self.longitude = lng  

    def setCoords(self, lat, lng):
        self.latitude = lat
        self.longitude = lng 

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


 #--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
# Global variables
#--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..-- 
rovey_pos = RoveyPosClass(0,0)
des_pos = DesPosClass(0, 0)
auto_engaged = False   # Flag variable for enabling autonomous mode

#--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
# navigation():
#    Main function
#--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--
def navigation():
    global rovey_pos
    global des_pos
    global auto_engaged

    rospy.init_node('navigation', anonymous=True)
    rate = rospy.Rate(2) # Loop rate in Hz

    server = rospy.Service('/core_rover/start_auto', StartAuto, handleStartAuto)

    gps_sub = rospy.Subscriber("/pioneer3at/gps/values", NavSatFix, gpsCallback)
    waypoint_pub  = rospy.Publisher("/core_rover/navigation/waypoint_coords", NavSatFix, queue_size=10)

    while not rospy.is_shutdown():

        if auto_engaged is True:

            #add code to generate waypoints somewhere in here. 
            # also are you publishing waypoints one at a time? if so, gotta write something so 
            # the navigation node knows when to send the next waypoint. maybe using rover position?

            waypoint_msg = NavSatFix()
            waypoint_msg.latitude = des_pos.latitude
            waypoint_msg.longitude = des_pos.longitude
            waypoint_pub.publish(waypoint_msg)

        rate.sleep()

if __name__ == '__main__':
    navigation()
