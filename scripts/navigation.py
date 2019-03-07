#!/usr/bin/env python
import rospy, math,numpy
from sensor_msgs.msg import NavSatFix
from webots_ros.srv import set_float
from nova_common.msg import *
from nova_common.srv import *
import time

 #--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
# wayPoint(lng_current_pos,lat_current_pos,lng_destination,lat_destination,no_of_waypoints):
#  Function to generate list of waypoints, based on current and destination GPS coordinates.
#--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--
def wayPoint(lng_current_pos,lat_current_pos,lng_destination,lat_destination,no_of_waypoints):
    lng_increment = (lng_destination-lng_current_pos)/float(no_of_waypoints)
    lat_incrememnt = (lat_destination-lat_current_pos)/float(no_of_waypoints)
    way_points_list = []
    for i in range(no_of_waypoints):
        way_points_list.append((lng_current_pos+(i+1)*lng_increment,lat_current_pos+(i+1)*lat_incrememnt,i))
    return iter(way_points_list) # list of tuples

def spiralSearch(current_pos,no_of_waypoints):
    theta = numpy.linspace(0,10,num=no_of_waypoints)
    dd_const = 2 # No of degrees(lat long) the rover moves outward as it rotates theta degrees
    r = 2*theta
    x=r*numpy.cos(theta) + current_pos.longitude
    y=r*numpy.sin(theta) + current_pos.latitude
    searchPath=list()
    for i in range(len(x)):
        searchPath[i]=DesPosClass(x[i],y[i])
    return searchPath

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
        waypoint_list = iter(wayPoint(rovey_pos.longitude,rovey_pos.latitude,des_pos.longitude,des_pos.latitude,4))
        new_destination = True
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
way_pos = DesPosClass(0, 0)
des_pos = DesPosClass(0, 0)
waypoint_list = wayPoint(1,1,0,0,4)
auto_engaged = False   # Flag variable for enabling autonomous mode
new_destination = False

#--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
# navigation():
#    Main function
#--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--
def navigation():
    #Globals
    global rovey_pos
    global des_pos
    global auto_engaged
    global waypoint_list
    global way_pos
    # Constants
    dist_to_dest_thres = 3 #Distance to destination point threshold (metres)
    dist_to_way_thres = 4 #Distance to way point threshold (metres)
    server = rospy.Service('/core_rover/start_auto', StartAuto, handleStartAuto)
    gps_sub = rospy.Subscriber("/pioneer3at/gps/values", NavSatFix, gpsCallback)
    waypoint_pub  = rospy.Publisher("/core_rover/navigation/waypoint_coords", NavSatFix, queue_size=10)
    rospy.init_node('navigation', anonymous=True)
    rate = rospy.Rate(2) # Loop rate in Hz
    # waypoint_list = wayPoint(rovey_pos.longitude,rovey_pos.latitude,des_pos.longitude,des_pos.latitude,4)
    # waypoint_list = wayPoint(0.000000001,0.00000000001,0,0,4)
    rospy.loginfo("navigation node started")
    #### TO BE DELETED **************************
    auto_engaged = True #Override
    new_destination = True
    #### TO BE DELETED ***************************
    while not rospy.is_shutdown():

        if auto_engaged is True:
            #add code to generate waypoints somewhere in here.
            # also are you publishing waypoints one at a time? if so, gotta write something so
            # the navigation node knows when to send the next waypoint. maybe using rover position?

            #The following implements distance of line formula. However, it converts this from degrees to metres.
            # dist_to_dest = math.sqrt((rovey_pos.latitude - way_pos.latitude)**2 + (rovey_pos.longitude - way_pos.longitude)**2)*111000
            # if (dist_to_dest>dist_to_dest_thres):
            #     waypoint_list = iter(wayPoint(rovey_pos.longitude,rovey_pos.latitude,des_pos.longitude,des_pos.latitude,4))

            distance_to_waypt = math.sqrt((rovey_pos.latitude - way_pos.latitude)**2 + (rovey_pos.longitude - way_pos.longitude)**2)*111000
            #rospy.loginfo("distance: %s",str(distance_to_waypt))
            if ((distance_to_waypt<dist_to_way_thres) or new_destination is True): #only send the next point once the robot is close enough to is current target
                new_destination = False
                try:
                    current_waypoint = next(waypoint_list)
                    way_pos.longitude = current_waypoint[0]
                    way_pos.latitude = current_waypoint[1]
                    rospy.loginfo(current_waypoint)
                    waypoint_msg = NavSatFix() #Initialize waypoint data structure.
                    waypoint_msg.latitude = way_pos.latitude # Populate structure with lat and long
                    waypoint_msg.longitude = way_pos.longitude
                    waypoint_pub.publish(waypoint_msg) # Insert datastructure into waypoint publisher.
                except StopIteration:
                    rospy.loginfo("End of navigation")
                    auto_engaged = False
        rate.sleep()

if __name__ == '__main__':
    navigation()
