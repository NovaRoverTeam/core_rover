#!/usr/bin/env python
import rospy, math
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
# wayPoint(lng_current_pos,lat_current_pos,lng_destination,lat_destination,no_of_waypoints):
#  Function to generate list of waypoints, based on current and destination GPS coordinates.
#--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--
def wayPoint(lng_current_pos,lat_current_pos,lng_destination,lat_destination,no_of_waypoints):
    lng_increment = (lng_destination-lng_current_pos)/no_of_waypoints
    lat_incrememnt = (lat_destination-lat_current_pos)/no_of_waypoints
    way_points_list = []
    for i in range(no_of_waypoints):
        way_points_list.append((lng_current_pos+(i+1)*lng_increment,lat_current_pos+(i+1)*lat_incrememnt))
    return way_points_list # list of tuples
 #--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
# Global variables
#--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--
rovey_pos = RoveyPosClass(0,0)
way_pos = DesPosClass(0, 0)
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
    dist_to_dest_thres = 3 #Distance to destination point threshold (metres)
    dist_to_way_thres = 4 #Distance to way point threshold (metres)
    rospy.init_node('navigation', anonymous=True)
    rate = rospy.Rate(2) # Loop rate in Hz

    server = rospy.Service('/core_rover/start_auto', StartAuto, handleStartAuto)

    gps_sub = rospy.Subscriber("/pioneer3at/gps/values", NavSatFix, gpsCallback)
    waypoint_pub  = rospy.Publisher("/core_rover/navigation/waypoint_coords", NavSatFix, queue_size=10)
    waypoint_list = wayPoint(rovey_pos.longitude,rovey_pos.latitude,des_pos.longitude,des_pos.latitude,4)

    while not rospy.is_shutdown():

        if auto_engaged is True:
            #add code to generate waypoints somewhere in here.
            # also are you publishing waypoints one at a time? if so, gotta write something so
            # the navigation node knows when to send the next waypoint. maybe using rover position?

            #The following implements distance of line formula. However, it converts this from degrees to metres.
            dist_to_dest = math.sqrt((rovey_pos.latitude - way_pos.latitude)**2 + (rovey_pos.longitude - way_pos.longitude)**2)/111000
            if (dist_to_dest<dist_to_dest_threshold):
                waypoint_list = iter(wayPoint(rovey_pos.longitude,rovey_pos.latitude,des_pos.longitude,des_pos.latitude,4))

            distance = math.sqrt((rovey_pos.latitude - way_pos.latitude)**2 + (rovey_pos.longitude - way_pos.longitude)**2)/111000
            if (dist_to_dest<dist_to_dest_threshold):
                try:
                    current_waypoint = next(waypoint_list)
                except StopIteration:
                    print("last waypoint")
                way_pos.longitude = current_waypoint[0]
                way_pos.latitude = current_waypoint[1]
                waypoint_msg = NavSatFix() #Initialize waypoint data structure.
                waypoint_msg.latitude = way_pos.latitude # Populate structure with lat and long
                waypoint_msg.longitude = way_pos.longitude
                waypoint_pub.publish(waypoint_msg) # Insert datastructure into waypoint publisher.

        rate.sleep()

if __name__ == '__main__':
    navigation()
