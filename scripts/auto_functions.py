import rospy, math, numpy, copy
from auto_classes import WaypointClass, Vector2D, RoveyPosClass
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
# cosineRule(angle, sideA, sideB): Calculate the length of the third edge
# of a triangle given two other sides and the opposite angle.
#--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--
def cosineRule(angle, sideA, sideB):
    sideA2 = math.pow(sideA, 2)
    sideB2 = math.pow(sideB, 2)
    return math.sqrt(sideA2 + sideB2 - 2 * sideA * sideB * math.cos(angle))

#--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
# sineRule(sideA, sideB, angle): Calculate the angle between sides a and c
# of a triangle the lengths of all three sides and 
#--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--
def sineRule(sideA, sideB, sideC, angle):
    pass

#--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
# calculateInternalAngles(side_count): Calcualte the internal angle of
# each vertex of any regular polygon 
#--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--
def calculateInternalAngles(side_count):
    return (side_count-2)*180

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
    dd_const = 0.000001 # No of degrees(lat long) the rover moves outward as it rotates theta degrees
    r = dd_const*theta
    lng=r*numpy.cos(theta) + current_pos.longitude
    lat=r*numpy.sin(theta) + current_pos.latitude
    searchPath=list()
    for i in range(len(theta)):
        searchPath.append(WaypointClass(lat[i],lng[i]))
    return searchPath

#--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
# sectorSearch(center_pos, search_radius): Generate waypoints for a sector search.
# Takes center position of search path in lat lng and search radius in meters.
#--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--
def sectorSearch(center_pos, search_radius):
    k_segment_count = 6
    k_segment_angle = 360/k_segment_count
    k_meters_per_latlng = 111000
    k_latlng_radius = search_radius / k_meters_per_latlng
    search_path = []

    # Calculate first waypoint (after first leg)
    vector_from_center = Vector2D.makeFromBearingMagnitude(center_pos.yaw, k_latlng_radius)
    first_waypoint = WaypointClass.makeShiftedWaypoint(center_pos, vector_from_center)
    search_path.append(first_waypoint)
    
    # Calculate following waypoints until back to start
    new_waypoint = None
    while new_waypoint != first_waypoint:
        # Add waypoint after crossleg
        vector_from_center.rotate(k_segment_angle)
        search_path.append(WaypointClass.makeShiftedWaypoint(center_pos, vector_from_center))

        # Add waypoint after leg
        vector_from_center.invert()
        new_waypoint = WaypointClass.makeShiftedWaypoint(center_pos, vector_from_center)
        search_path.append(new_waypoint)

    print(search_path)
    return search_path