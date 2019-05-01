#!/usr/bin/env python
import rospy,math,time
from sensor_msgs.msg import NavSatFix, MagneticField
from webots_ros.srv import set_float
from nova_common.msg import *
from nova_common.srv import *
from auto_classes import WaypointClass, RoveyPosClass
from auto_functions import *
# from core_rover.srv import *
from transitions import Machine
simulator = True
#--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
# Class representation of Autonomous state machine
#--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
class AutonomousStateMachine():

    states = ['Off','Navigating','Searching','Detected','Finding','Complete']

    def __init__(self):
        #--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
        #Constructor
        #--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
        # Transitions between the states. 'trigger' is a function you can call
        # on the class object which triggers the transition to occur. The
        # functions listed in 'before' are functions within the class that will
        # run just after the transition takes place. Calling the function returns
        # the transition's success or lack thereof.
        transitions = [
        {   'source': ['Off','Navigating','Searching'],'dest':'Navigating','after':'Navigating','trigger':'Navigate'},
        {   'source': 'Navigating','dest':'Searching','after':'Searching','trigger':'Search'},
        {   'source': ['Navigating','Searching','Lost'],'dest':'Detected','after':'Detected','trigger':'Found_TB'},
        {   'source': 'Detected','dest':'Finding','after':'Finding','trigger':'Lost_TB'},
        {   'source': 'Detected','dest':'Complete','after':'Complete','trigger':'Complete'},
        {   'source': ['Navigating','Searching','Detected','Finding','Complete'],
            'dest':'Off','after':'Off','trigger':'Cancel'},
        ]

        # Initialize the state Machine
        self.mach = Machine(model=self,states=AutonomousStateMachine.states, initial='Off',
        transitions=transitions, ignore_invalid_triggers=True)
        # Initialise the global state parameter
        self.setMode('Off')
    #--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
    # Set autonomous mode ROS paramter
    #--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
    def setMode(self, mode):
        rospy.set_param('/core_rover/autonomous_mode', mode)
    #--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
    # Functions Called "After" transitions - Sets the ROS Param to the
    # correct mode.
    #--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
    def Navigating(self,req):
        ## Implement initNavigation() method from navigation.py here
        initNavigation(req)
        self.setMode('Navigating')
    def Searching(self):
        ## Implement initSearch() method from navigation.py here
        self.setMode('Searching')
    def Detected(self):
        self.setMode('Detected')
    def Finding(self):
        self.setMode('Finding')
    def Complete(self):
        self.setMode('Complete')
    def Off(self):
        self.setMode('Off')
###########################################
#END State Machine Class
###########################################

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
#    Callback for the orientation of the rover
#--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--
def compassCallback(compassData):
    global rovey_pos
    x = compassData.magnetic_field.x
    z = compassData.magnetic_field.z
    rovey_pos.setOrientation(x,z)
    #rospy.logdebug("x: %s, z: %s", x, z)

#--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
# auto_drive(publisher):
#    Takes current waypoint and rover position. Creates drive command.
#    Publishes drive command.
#--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--
def auto_drive(publisher,orientation):
    global rovey_pos, waypoint
    beta = angleBetween(rovey_pos.latitude, rovey_pos.longitude, waypoint.latitude, waypoint.longitude)
    distance = distanceBetween(rovey_pos.latitude, rovey_pos.longitude, waypoint.latitude, waypoint.longitude)
    turn = turnDirection(beta, orientation)
    rospy.loginfo("Logging Autonomous Parameters")
    rospy.loginfo("beta: %s", beta)
    rospy.loginfo("distance: %s", distance)
    rospy.loginfo("orientation: %s", orientation)
    rpm_limit   = rospy.get_param('rpm_limit')
    steer_limit = rospy.get_param('steer_limit')
    drive_msg = DriveCmd()
    drive_msg.rpm       = rpm_limit*10
    drive_msg.steer_pct = steer_limit*10*turn/180
    publisher.publish(drive_msg)
#--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
# initNavigation():
#  Intitialise Navigation to Tennis Ball GPS location
#--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--
def initNavigation(req):
   global des_pos,waypoint_list, waypoint_iter
   des_pos.setCoords(req.latitude, req.longitude)
   waypoint_list = wayPoint(rovey_pos.longitude,rovey_pos.latitude,des_pos.longitude,des_pos.latitude,4)
   waypoint_iter = iter(waypoint_list)
   rospy.loginfo('DESTINATION:' + str(des_pos))
   rospy.loginfo('WaypointList: ' + str(waypoint_list))
 #--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
# initSearch():
#  Intitialise Searching for Tennis Ball (Spiral)
#--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--
def initSearch():
    searchpath = spiralSearch(rovey_pos,25,0,10)
    global waypoint_list
    waypoint_list = searchpath
    global waypoint_iter
    waypoint_iter = iter(searchpath)
    # spiral_engaged = True
    rospy.loginfo('Spiral Search Engaged!')
    rospy.loginfo('Spiral WaypointList: ' + str(searchpath))
 #--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
# handleStartAuto():
#  Service server handler for starting autonomous mission.
#--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--
def handleStartAuto(req):
    global SM
    if getMode() == 'Auto':
        # Set the desired latitude and longitude from the service request
        SM.navigate(req)
        return StartAutoResponse(True,
            "Successfully started Auto mission.")
    else:
        return StartAutoResponse(False,
            "Unable to start mission, must be in Auto mode.")
 #--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
# Global variables
#--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--
rovey_pos = RoveyPosClass(0,0,0,0)
des_pos = WaypointClass(0,0) #Object GPS coords given by the competition. This is updated by the GUI
waypoint = WaypointClass(0, 0) #Object GPS coords of current waypoint.
waypoint_list = wayPoint(1,1,0,0,4)
waypoint_iter = iter(waypoint_list) #Iterable object for way points to be sent.
#--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
# auto_controller():
#    Main Autonomous Controller Initialise
#--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--
def auto_controller():
    #Set Global Variables:
    global rovey_pos, des_pos, waypoint
    # Set Constants:
    dist_to_dest_thres = 3 #Distance to destination point threshold (metres)
    dist_to_way_thres = 4 #Distance to way point threshold (metres)
    #### SIMULATOR ONLY START **************************
    # Since the simulator is not to scale, the rover will do donuts.
    if simulator:
        dist_to_dest_thres = 55500 #Distance to destination point threshold (metres)
        dist_to_way_thres = 55500 #Distance to way point threshold (metres)
    #### SIMULATOR ONLY START **************************

    #ROS Publisher Subscribers and Services
    server = rospy.Service('/core_rover/start_auto', StartAuto, handleStartAuto)
    gps_sub     = rospy.Subscriber("/pioneer3at/gps/values", NavSatFix, gpsCallback)
    compass_sub = rospy.Subscriber("/pioneer3at/compass/values", MagneticField, compassCallback)
    drive_pub   = rospy.Publisher("/core_rover/driver/drive_cmd", DriveCmd, queue_size=10)
    status_pub  = rospy.Publisher("/core_rover/auto_status", AutoStatus, queue_size=10)

    SM = AutonomousStateMachine() # Initialise state machine class
    rospy.init_node('autonomous_controller', anonymous=True) # Initialise Node
    rate = rospy.Rate(2) # Loop rate in Hz
    rospy.loginfo("Autonomous Controller Started")
    while not rospy.is_shutdown():
        orientation = bearingInDegrees(rovey_pos.x, rovey_pos.z)
        #Current rover distance from final destination, and current waypoint
        distance_to_dest = distanceBetween(rovey_pos.latitude, rovey_pos.longitude, des_pos.latitude, des_pos.longitude)*111000
        distance_to_waypt = distanceBetween(rovey_pos.latitude, rovey_pos.longitude, waypoint.latitude, waypoint.longitude)*111000
        if getMode() == 'Auto':
            if SM.state == 'Navigating':
                # Navigating
                if ((distance_to_waypt<dist_to_dest_thres)):
                    try:
                        waypoint = next(waypoint_iter)
                    except StopIteration:
                        rospy.loginfo("Waypoint List Exhausted - Restarting Navigation to GPS coordinate")
                        SM.Navigate()

                if ((distance_to_dest<dist_to_dest_thres)):
                    # Switch to search state
                    # If close to GPS destination populate spiral search in waypoint iterator queue.
                    rospy.loginfo('Rover close to GPS coordinate destination. Commencing Search')
                    SM.Search()
                auto_drive(drive_pub,orientation)
            if SM.state == 'Searching':
                # Spiral Searching
                rospy.loginfo('Spiral Searching for Tennis Ball')
                if ((distance_to_waypt<dist_to_dest_thres)):
                    try:
                        waypoint = next(waypoint_iter)
                    except StopIteration:
                        rospy.loginfo("Spiral Search Waypoint List Exhausted - Going back to GPS coordinate")
                        SM.Search()
                auto_drive(drive_pub,orientation)
            if SM.state == 'Detected':
                # Detected Tennis Ball
                rospy.loginfo('Detected Tennis Ball')
            if SM.state == 'Finding':
                # Lost the tennis ball after previous detection. Try to find again.
                rospy.loginfo('Lost the tennis ball after previous detection. Try to find again.')
            if SM.state == 'Complete':
                # Completed the task
                rospy.loginfo('Completed the task')

        # TODO adjust rate that AutoStatus is published
        status_msg = AutoStatus()
        status_msg.latitude  = rovey_pos.latitude
        status_msg.longitude = rovey_pos.longitude
        status_msg.bearing   = orientation
        status_pub.publish(status_msg)

        rate.sleep()
if __name__ == '__main__':
    auto_controller()
