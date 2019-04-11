#!/usr/bin/env python

import rospy
from core_rover.srv import *
rospy.init_node('auto_sm_test', anonymous=True)
def request(action,property):
    sm_service = rospy.ServiceProxy('auto_sm_service',auto_sm)
    return sm_service(action,property)

print(request('get','Off'))
print(request('set','Navigating'))
print(request('get','Off'))
print(request('get','Navigating'))
print(request('get',''))
rospy.spin()
