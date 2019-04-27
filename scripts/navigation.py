#!/usr/bin/env python
import rospy, math,numpy
from sensor_msgs.msg import NavSatFix
from webots_ros.srv import set_float
from nova_common.msg import *
from nova_common.srv import *
import time

simulator = False
testing = True
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
    def __str__(self):
        return 'latitude: ' + str(self.latitude) + ' longitude: ' + str(self.longitude)
    def __repr__(self):
        return 'latitude: ' + str(self.latitude) + ' longitude: ' + str(self.longitude)

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
# getRoverMode(): Retrieve Mode from parameter server.
#--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--
def getRoverMode():
    return rospy.get_param('/core_rover/Mode', "Auto")

#--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
# getAutoMode(): Retrieve Mode from parameter server.
#--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--
def getAutoMode():
    return rospy.get_param('/core_rover/autonomous_mode')

 #--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
# wayPoint(lng_current_pos,lat_current_pos,lng_destination,lat_destination,no_of_waypoints):
#  Function to generate list of waypoints, based on current and destination GPS coordinates.
#--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--
def wayPoint(lng_current_pos,lat_current_pos,lng_destination,lat_destination,no_of_waypoints):
    lng_increment = (lng_destination-lng_current_pos)/float(no_of_waypoints)
    lat_incrememnt = (lat_destination-lat_current_pos)/float(no_of_waypoints)
    way_points_list = []
    for i in range(no_of_waypoints):
        waypoint_lng = lng_current_pos+(i+1)*lng_increment
        waypoint_lat = lat_current_pos+(i+1)*lat_incrememnt
        way_points_list.append(RoveyPosClass(waypoint_lat,waypoint_lng))
    return way_points_list# list of tuples

#--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
# spiralSearch(current_pos,no_of_waypoints,rang_min,rang_max): Generate waypoints for a spiral search.
# Takes current position of rover, the number of waypoints desired, and the range of spiral.
#--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--
def spiralSearch(current_pos,no_of_waypoints,rang_min,rang_max):
    theta = numpy.linspace(rang_min,rang_max,num=no_of_waypoints)
    dd_const = 0.0000001 # No of degrees(lat long) the rover moves outward as it rotates theta degrees
    r = dd_const*theta
    lng=r*numpy.cos(theta) + current_pos.longitude
    lat=r*numpy.sin(theta) + current_pos.latitude
    searchPath=list()
    for i in range(len(x)):
        searchPath.append(RoveyPosClass(lat[i],lng[i]))
    return searchPath
 #--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
# wayPoint(lng_current_pos,lat_current_pos,lng_destination,lat_destination,no_of_waypoints):
#  Function to generate list of waypoints, based on current and destination GPS coordinates.
#--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--
def wayPoint(lng_current_pos,lat_current_pos,lng_destination,lat_destination,no_of_waypoints):
    lng_increment = (lng_destination-lng_current_pos)/float(no_of_waypoints)
    lat_incrememnt = (lat_destination-lat_current_pos)/float(no_of_waypoints)
    way_points_list = []
    for i in range(no_of_waypoints):
        waypoint_lng = lng_current_pos+(i+1)*lng_increment
        waypoint_lat = lat_current_pos+(i+1)*lat_incrememnt
        way_points_list.append(RoveyPosClass(waypoint_lat,waypoint_lng))
    return way_points_list# list of tuples

#--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
# spiralSearch(current_pos,no_of_waypoints,rang_min,rang_max): Generate waypoints for a spiral search.
# Takes current position of rover, the number of waypoints desired, and the range of spiral.
#--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--
def spiralSearch(current_pos,no_of_waypoints,rang_min,rang_max):
    theta = numpy.linspace(rang_min,rang_max,num=no_of_waypoints)
    dd_const = 2 # No of degrees(lat long) the rover moves outward as it rotates theta degrees
    r = 2*theta
    lng=r*numpy.cos(theta) + current_pos.longitude
    lat=r*numpy.sin(theta) + current_pos.latitude
    searchPath=list()
    for i in range(len(theta)):
        searchPath.append(RoveyPosClass(lat[i],lng[i]))
    return searchPath
 #--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
# handleStartAuto():
#  Service server handler for starting autonomous mission.
#--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--
def handleStartAuto(req):
    if getRoverMode() == 'Auto':
        # Set the desired latitude and longitude from the service request
        initNavigation()
        return StartAutoResponse(True,
            "Successfully started Auto mission.")
    else:
        return StartAutoResponse(False,
            "Unable to start mission, must be in Auto mode.")
 #--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
# initNavigation():
#  Intitialise Navigation to Tennis Ball GPS location
#--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--
def initNavigation():
    global des_pos
    global auto_engaged
    global new_destination
    global waypoint_list
    global waypoint_iter
    global spiral_engaged
    spiral_engaged = False
    if testing:
        des_pos.setCoords(-37.91008843314037 ,145.1362295348945)
    else:
        des_pos.setCoords(req.latitude, req.longitude)
    auto_engaged = True
    waypoint_list = wayPoint(rovey_pos.longitude,rovey_pos.latitude,des_pos.longitude,des_pos.latitude,4)
    waypoint_iter = iter(waypoint_list)
    new_destination = True
    spiral_engaged = False
    rospy.loginfo('DESTINATION:' + str(des_pos))
    rospy.loginfo('WaypointList: ' + str(waypoint_list))

 #--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
# initSearch():
#  Intitialise Searching for Tennis Ball (Spiral)
#--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--
def initSearch():
    global waypoint_list
    global waypoint_iter
    global new_destination
    global spiral_engaged
    waypoint_list = spiralSearch(rovey_pos,25,0,10)
    waypoint_iter = iter(waypoint_list)
    new_destination = True
    spiral_engaged = True
    rospy.loginfo('Spiral Search Engaged!')
    rospy.loginfo('Spiral WaypointList: ' + str(waypoint_list))
 #--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
# Global variables
#--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--
rovey_pos = RoveyPosClass(0,0) #Object for GPS coords of current rover position.
way_pos = RoveyPosClass(0,0) #Object GPS coords of current waypoint.
des_pos = RoveyPosClass(0,0) #Object GPS coords given by the competition. This is updated by the GUI
waypoint_list = wayPoint(1,1,0,0,4)
waypoint_iter = iter(waypoint_list) #Iterable object for way points to be sent.
auto_engaged = False   # Flag variable for enabling autonomous mode
new_destination = False # Flag variable for sending the first way point, when a new destination is set.
spiral_engaged = False # Flag variable for spiral search.

#--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
# navigation():
#    Main function
#--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--
def navigation():
    #Globals
    global rovey_pos
    global des_pos
    global auto_engaged
    global spiral_engaged
    global waypoint_iter
    global way_pos
    global new_destination
    global simulator

    dist_to_dest_thres = 3 #Distance to destination point threshold (metres)
    dist_to_way_thres = 4 #Distance to way point threshold (metres)
    #### SIMULATOR ONLY START **************************
    # Since the simulator is not to scale, the rover will do donuts.
    if simulator:
        dist_to_dest_thres = 55500 #Distance to destination point threshold (metres)
        dist_to_way_thres = 55500 #Distance to way point threshold (metres)
    #### SIMULATOR ONLY START **************************

    server = rospy.Service('/core_rover/start_auto', StartAuto, handleStartAuto)
    #gps_sub = rospy.Subscriber("/pioneer3at/gps/values", NavSatFix, gpsCallback)
    gps_sub = rospy.Subscriber("/nova_common/gps_data", NavSatFix, gpsCallback)
    waypoint_pub  = rospy.Publisher("/core_rover/navigation/waypoint_coords", NavSatFix, queue_size=10)
    rospy.init_node('navigation', anonymous=True)
    rate = rospy.Rate(2) # Loop rate in Hz
    rospy.loginfo("navigation node started")
    #### DEBUG ONLY START **************************
    # auto_engaged = True #Override
    # new_destination = True
    # spiral_engaged = False
    if testing:
        time.sleep(2)
    #### DEBUG ONLY END ***************************
    initNavigation()
    while not rospy.is_shutdown():
	rospy.loginfo(rovey_pos)
        if True:
            if True:
                rospy.loginfo("dog") if testing
                #Waypoint Publisher
                #The following implements distance of line formula. However, it converts this from degrees to metres.
                distance_to_dest = math.sqrt((rovey_pos.latitude - des_pos.latitude)**2 + (rovey_pos.longitude - des_pos.longitude)**2)*111000
                distance_to_waypt = math.sqrt((rovey_pos.latitude - way_pos.latitude)**2 + (rovey_pos.longitude - way_pos.longitude)**2)*111000

                if ((distance_to_dest<dist_to_dest_thres) and spiral_engaged is not True):
                    # Switch to search state
                    # If close to GPS destination populate spiral search in waypoint iterator queue.
                    initSearch()
                # Rover proximity to current waypoint determines if next waypoint should be sent.
                if ((distance_to_waypt<dist_to_way_thres) or new_destination is True): #only send the next point once the robot is close enough to is current target
                    new_destination = False
                    try:
                        way_pos = next(waypoint_iter)
                        rospy.loginfo(way_pos)
                        rospy.loginfo("cat") if testing
                        waypoint_msg = NavSatFix() #Initialize waypoint data structure.
                        waypoint_msg.latitude = way_pos.latitude # Populate structure with lat and long
                        waypoint_msg.longitude = way_pos.longitude
                        waypoint_pub.publish(waypoint_msg) # Insert datastructure into waypoint publisher.
                    except StopIteration:
                        rospy.loginfo("End of navigation")
                        if spiral_engaged==True:
                            initNavigation()
                        else:
                            initSearch()
        rate.sleep()

if __name__ == '__main__':
    navigation()
