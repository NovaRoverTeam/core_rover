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
testing = True
#--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
# Class representation of Autonomous state machine
#--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
class AutonomousStateMachine():
    dist_to_dest_thres = 3 #Distance to destination point threshold (metres)
    dist_to_way_thres = 4 #Distance to way point threshold (metres)
    distance_to_dest = None # In metres (default is empty)
    distance_to_waypt = None # In metres (default is empty)
    orientation = None # In degrees (default is empty)
    rovey_pos = RoveyPosClass(0,0,0,0,0)
    des_pos = WaypointClass(0,0) #Object GPS coords given by the competition. This is updated by the GUI
    waypoint = WaypointClass(0, 0) #Object GPS coords of current waypoint.
    waypoint_list = wayPoint(1,1,0,0,4)
    waypoint_iter = iter(waypoint_list) #Iterable object for way points to be sent.

    # ROS Publisher, Subscriber and Service Callbacks
    def gpsCallback(self,gpsData):
        '''Callback for the location of the rover'''
        lat = gpsData.latitude
        lng = gpsData.longitude
        self.rovey_pos.setCoords(lat,lng)
  
    def rpyCallback(self,rpyData):
        '''Callback for roll, pitch, yaw from IMU'''
        global rovey_pos
        self.rovey_pos.setOrientation(rpyData.roll, rpyData.pitch, rpyData.yaw)


    def handleStartAuto(self,req):
        '''Service server handler for starting autonomous mission.'''
        if getMode() == 'Auto':
            self.des_pos.setCoords(req.latitude, req.longitude) # Set the desired latitude and longitude from the service request
            self.toTraverse()
            return StartAutoResponse(True,"Successfully started Auto mission.")
        else:
            return StartAutoResponse(False,"Unable to start mission, must be in Auto mode.")

    states = ['Off','Traverse','Search','Destroy','Panning','Complete']

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
        # Self States - Occur when a state is running
        {   'source': 'Off', 'dest': None, 'after': 'Off', 'trigger':'run'},
        {   'source': 'Traverse', 'dest': None, 'after': 'Traverse', 'trigger':'run'},
        {   'source': 'Search', 'dest': None, 'after': 'Search', 'trigger':'run'},
        {   'source': 'Destroy', 'dest': None, 'after': 'Destroy', 'trigger':'run'},
        {   'source': 'Panning', 'dest': None, 'after': 'Panning', 'trigger':'run'},
        {   'source': 'Complete', 'dest': None, 'after': 'Complete', 'trigger':'run'},
        # Changes states - Occur to change between different stages of the task.
        {   'source': ['Off','Traverse','Search'],'dest':'Traverse','after':'startTraverse','trigger':'toTraverse'},
        {   'source': ['Traverse','Panning'],'dest':'Search','after':'startSearch','trigger':'toSearch'},
        {   'source': ['Traverse','Search','Panning'],'dest':'Destroy','after':'startDestroy','trigger':'toDestroy'},
        {   'source': 'Destroy','dest':'Panning','after':'startPanning','trigger':'toPanning'},
        {   'source': 'Destroy','dest':'Complete','after':'startComplete','trigger':'toComplete'},
        {   'source': ['Traverse','Search','Destroy','Panning','Complete'],
            'dest':'Off','after':'startOff','trigger':'Reset'},
        ]

        # Initialize the state Machine
        self.mach = Machine(model=self,states=AutonomousStateMachine.states, initial='Off',
        transitions=transitions, ignore_invalid_triggers=True)
            # Ros Publishers and Subscribers
        self.drive_pub   = rospy.Publisher("/core_rover/driver/drive_cmd", DriveCmd, queue_size=10)
        self.gps_sub     = rospy.Subscriber("/nova_common/gps_data", NavSatFix, self.gpsCallback)
        self.rpy_sub     = rospy.Subscriber("/nova_common/RPY", RPY, self.rpyCallback)
        # Initialise the global state parameter
        self.setMode('Off')
    #--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
    # Set autonomous mode ROS paramter
    #--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
    def setMode(self, mode):
        rospy.set_param('/core_rover/autonomous_mode', mode)

    #--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
    # auto_drive(publisher):
    #    Takes current waypoint and rover position. Creates drive command.
    #    Publishes drive command.
    #--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--
    def auto_drive(self):
        beta = angleBetween(self.rovey_pos.latitude, self.rovey_pos.longitude, self.waypoint.latitude, self.waypoint.longitude)
        distance = distanceBetween(self.rovey_pos.latitude, self.rovey_pos.longitude, self.waypoint.latitude, self.waypoint.longitude)
        turn = turnDirection(beta, self.orientation)
        rospy.loginfo("Logging Autonomous Parameters")
        rospy.loginfo("beta: %s", beta)
        rospy.loginfo("distance: %s", distance)
        rospy.loginfo("orientation: %s", self.orientation)
        rospy.loginfo("current pos: %s", self.waypoint)
        rpm_limit   = rospy.get_param('rpm_limit',10)
        steer_limit = rospy.get_param('steer_limit',10)
        drive_msg = DriveCmd()
        drive_msg.rpm       = 10
        drive_msg.steer_pct = 0.5*turn
        self.drive_pub.publish(drive_msg)
    def metricCalculation(self):
        self.orientation = self.rovey_pos.yaw
        #Current rover distance from final destination, and current waypoint
        self.distance_to_dest = distanceBetween(self.rovey_pos.latitude, self.rovey_pos.longitude, self.des_pos.latitude, self.des_pos.longitude)*111000
        self.distance_to_waypt = distanceBetween(self.rovey_pos.latitude, self.rovey_pos.longitude, self.waypoint.latitude, self.waypoint.longitude)*111000
    #--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
    # Functions Called "After" transitions - Sets the ROS Param to the
    # correct mode.
    #--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--

    def startTraverse(self):
        '''Intitialise Navigation to Tennis Ball GPS location'''
        self.metricCalculation()
        self.waypoint_list = wayPoint(self.rovey_pos.longitude,self.rovey_pos.latitude,self.des_pos.longitude,self.des_pos.latitude,4)
        self.waypoint_iter = iter(self.waypoint_list)
        self.waypoint = next(self.waypoint_iter)
        rospy.loginfo('DESTINATION:' + str(self.des_pos))
        rospy.loginfo('WaypointList: ' + str(self.waypoint_list))
        self.setMode('Traverse')
    def Traverse(self):
        # Run Traverse
        self.metricCalculation()
        if ((self.distance_to_waypt<self.dist_to_dest_thres)):
            try:
                self.waypoint = next(self.waypoint_iter)
            except StopIteration:
                rospy.loginfo("Waypoint List Exhausted - Restarting Navigation to GPS coordinate")
                self.toTraverse()
        if (self.distance_to_dest<self.dist_to_dest_thres):
            rospy.loginfo('Rover close to GPS coordinate destination. Commencing Search')
            self.toSearch()# Switch to search state
        self.auto_drive()
        self.setMode('Traverse')
    def startSearch(self):
        '''Intitialise Search for Tennis Ball (Spiral)'''
        self.metricCalculation()
        self.waypoint_list = spiralSearch(self.rovey_pos,25,0,8*math.pi)
        self.waypoint_iter = iter(self.waypoint_list)
        self.setMode('Search')
        rospy.loginfo('Spiral Search Engaged!')
        rospy.loginfo('Spiral WaypointList: ' + str(self.waypoint_list))

    def Search(self):
        '''Run Spiral Search'''
        self.metricCalculation()
        rospy.loginfo('Spiral Search for Tennis Ball')
        if ((self.distance_to_waypt<self.dist_to_dest_thres)):
            try:
                self.waypoint = next(self.waypoint_iter)
            except StopIteration:
                rospy.loginfo("Spiral Search Waypoint List Exhausted - Going back to GPS coordinate")
                self.toTraverse()
        self.auto_drive()
        self.setMode('Search')
    def startDestroy(self):
        '''Start tracking the tennis ball to mow it down. '''
        self.metricCalculation()
        self.setMode('Destroy')
    def Destroy(self):
        '''Mowing the tennis ball down'''
        self.metricCalculation()
        self.setMode('Destroy')
    def startPanning(self):
        self.metricCalculation()
        self.setMode('Panning')
    def Panning(self):
        self.metricCalculation()
        self.setMode('Panning')
    def Complete(self):
        self.metricCalculation()
        self.setMode('Complete')
    def Off(self):
        self.metricCalculation()
        self.setMode('Off')
###########################################
#END State Machine Class
###########################################

#--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
# auto_controller():
#    Main Autonomous Controller Initialise
#--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--
def auto_controller():
    global testing
    ''' Run Autonomous Controller Node '''
    SM = AutonomousStateMachine() # Initialise state machine class
    #ROS Publisher Subscribers and Services
    server = rospy.Service('/core_rover/start_auto', StartAuto, SM.handleStartAuto)

    status_pub  = rospy.Publisher("/core_rover/auto_status", AutoStatus, queue_size=10)

    rospy.init_node('auto_controller', anonymous=True) # Initialise Node
    rate = rospy.Rate(2) # Loop rate in Hz
    rospy.loginfo("Autonomous Controller Started")
    ## testing
    if testing:
        time.sleep(2)
        SM.des_pos = WaypointClass(-37.6617819, 145.3692175)
        SM.toTraverse()
    ## end testing
    while not rospy.is_shutdown():
        if getMode() == 'Auto' or testing:
            SM.run()
            rospy.loginfo('RUN ITER')
        # TODO adjust rate that AutoStatus is published
        status_msg = AutoStatus()
        status_msg.latitude  = SM.rovey_pos.latitude
        status_msg.longitude = SM.rovey_pos.longitude
        status_msg.bearing   = SM.orientation
        status_pub.publish(status_msg)
        rate.sleep() # Sleep until next iteration
if __name__ == '__main__':
    auto_controller()
