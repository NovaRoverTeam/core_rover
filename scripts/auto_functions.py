import rospy, math,numpy
from auto_classes import WaypointClass
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
    lat1 = math.radians(lat1)
    lat2 = math.radians(lat2)
    longDiff = math.radians(lng2 - lng1)
    y = math.sin(longDiff) * math.cos(lat2)
    x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(longDiff)
    bearing = math.atan2(y, x)
    bearing = math.degrees(bearing)
    bearing = (bearing+360) % 360
    return bearing

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
# getMode(): Retrieve Mode from parameter server.
#--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--
def getMode():
    return rospy.get_param('/core_rover/Mode','Standby')
#--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
# getAutoMode(): Retrieve Mode from parameter server.
#--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--
def getAutoMode():
    return rospy.get_param('/core_rover/autonomous_mode','Off')

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
        way_points_list.append(WaypointClass(waypoint_lat,waypoint_lng))
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
    for i in range(len(x)):
        searchPath.append(RoveyPosClass(lat[i],lng[i]))
    return searchPath
