#!/usr/bin/env python

import rospy
# from core_rover.srv import *
from transitions import Machine
#--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
# Class representation of Autonomous state machine - this is for
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
        {   'source': 'Off','dest':'Navigating','after':'Navigating','trigger':'Navigate'},
        {   'source': 'Navigating','dest':'Searching','after':'Searching','trigger':'Search'},
        {   'source': 'Searching','dest':'Detected','after':'Detected','trigger':'Found_TB'},
        {   'source': 'Detected','dest':'Finding','after':'Finding','trigger':'Lost_TB'},
        {   'source': 'Lost','dest':'Detected','after':'Detected','trigger':'Found_TB'},
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
    def Navigating(self):
        self.setMode('Navigating')
    def Searching(self):
        self.setMode('Searching')
    def Detected(self):
        self.setMode('Detected')
    def Finding(self):
        self.setMode('Finding')
    def Complete(self):
        self.setMode('Complete')
    def Off(self):
        self.setMode('Off')
# class SM_Node():
#     SM = AutonomousStateMachine()
#     # State machine service callback
#     def SM_service_callback(self,req):
#         command = req.action
#         if (command=='get'):
#             prop = req.property
#             if not prop:
#                 return auto_smResponse(self.SM.state)
#             else:
#                 return auto_smResponse(str(self.SM.state==prop))
#         if (command == 'set'):
#             prop = req.property
#             if prop in self.SM.states:
#                 self.SM.setMode(prop)
#                 return auto_smResponse(str(True))
#             else:
#                 return auto_smResponse(str(False))
#
#     # Main Auto State Machine Node Function
#     def __init__(self):
#         #Initialise state machine class
#         #Init Node
#         rospy.init_node('auto_sm', anonymous=True)
#         rate = rospy.Rate(1) # Loop rate in Hz
#         rospy.loginfo("Autonomous State machine has Started")
#         # Initialise Servce
#         s = rospy.Service('auto_sm_service',auto_sm,self.SM_service_callback)
#         rospy.spin()
#
# if __name__ == '__main__':
#     SM_Node()
