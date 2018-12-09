#!/usr/bin/env python

import rospy
from transitions import Machine

# Class representation of state machine
class RoverStateMachine():

    states = ['Standby', 'Drive', 'Arm', 'Auto', 'Drill']

    def __init__(self):

        # Transitions between the states. 'trigger' is a function you can call
        # on the class object which triggers the transition to occur. The
        # functions listed in 'before' are functions within the class that will 
        # run just after the transition takes place. Calling the function returns
        # the transition's success or lack thereof.
        transitions = [
            {   'source': 'Standby', 'dest': 'Drive', 
                'trigger': 'setDrive', 'after':'drive',
                'conditions':['connected']},
            {   'source': 'Standby', 'dest': 'Arm', 
                'trigger': 'setArm', 'after':'arm',
                'conditions':['missionERTorEQP', 'connected']},
            {   'source': 'Standby', 'dest': 'Drill', 
                'trigger': 'setDrill', 'after':'drill',
                'conditions':['missionSCI', 'connected']},
            {   'source': 'Standby', 'dest': 'Auto', 
                'trigger': 'setAuto', 'after':'auto',
                'conditions':['missionAUT', 'connected']},

            {   'source': 'Drive', 'dest': 'Standby', 
                'trigger': 'setStandby', 'after':'standby'},
            {   'source': 'Drive', 'dest': 'Arm', 
                'trigger': 'setArm', 'after':'arm',
                'conditions':['missionERTorEQP', 'connected']},
            {   'source': 'Drive', 'dest': 'Drill', 
                'trigger': 'setDrill', 'after':'drill',
                'conditions':['missionSCI', 'connected']},

            {   'source': 'Arm', 'dest': 'Standby', 
                'trigger': 'setStandby', 'after':'standby'},
            {   'source': 'Arm', 'dest': 'Drive', 
                'trigger': 'setDrive', 'after':'drive',
                'conditions':['missionERTorEQP', 'connected']},

            {   'source': 'Drill', 'dest': 'Standby', 
                'trigger': 'setStandby', 'after':'standby'},
            {   'source': 'Drill', 'dest': 'Drive', 
                'trigger': 'setDrive', 'after':'drive',
                'conditions':['missionSCI', 'connected']},

            {   'source': 'Auto', 'dest': 'Standby', 
                'trigger': 'setStandby', 'after':'standby'}
        ]

        # Initialize the state machine
        self.mach = Machine(model=self, 
            states=RoverStateMachine.states, initial='Standby',
            transitions=transitions, ignore_invalid_triggers=True)

        # Initialise the global state parameter
        self.setMode('Standby')


    def setMode(self, mode):
        rospy.set_param('/core_rover/Mode', mode)

    def missionSCI(self):
        mission = rospy.get_param('Mission')
        return mission == 'SCI'

    def missionAUT(self):
        mission = rospy.get_param('Mission')
        return mission == 'AUT'

    def missionERTorEQP(self):
        mission = rospy.get_param('Mission')
        return (mission == 'ERT' or mission == 'EQP')

    def connected(self):
        connectivity = rospy.get_param('/core_rover/Connectivity')
        return connectivity != 'Full_Loss'


    def standby(self):
        self.setMode('Standby')

    def drive(self):
        self.setMode('Drive')

    def arm(self):
        self.setMode('Arm')

    def drill(self):
        self.setMode('Drill')

    def auto(self):
        self.setMode('Auto')
