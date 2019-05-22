#!/usr/bin/env python

import rospy
from sensor_msgs.msg import NavSatFix
from nova_common.msg import *
from nova_common.srv import *

state_objective_latitude = -38.0
state_objective_longitude = 151.2093

NODE_NAME = "objective_node"
OBJECTIVE_GPS_TOPIC = "/planner/objective_gps"

def is_ready():
    return state_objective_latitude != None \
           and state_objective_longitude != None

def handle_receive_objective(req):
    global state_objective_latitude
    global state_objective_longitude
    state_objective_latitude = req.latitude
    state_objective_longitude = req.longitude
    return StartAutoResponse(True, "Input objective coordinates for planning subsystem.")

def objective_node():

    # node should update objective when notifi3 ed by server
    server = rospy.Service("/core_rover/start_auto", StartAuto, handle_receive_objective)

    # publish objective
    objective_publisher = rospy.Publisher(OBJECTIVE_GPS_TOPIC, NavSatFix, queue_size=1)

    rospy.init_node(NODE_NAME)
    publish_rate = rospy.Rate(2)
    while not rospy.is_shutdown():
        if is_ready():
            msg = NavSatFix()
            msg.header.stamp = rospy.get_rostime()
            msg.latitude = state_objective_latitude
            msg.longitude = state_objective_longitude
            rospy.loginfo("Objective being published: %f, %f", msg.latitude, msg.longitude)
            objective_publisher.publish(msg)
        else:
            rospy.loginfo("Objective node not ready to publish.")
        publish_rate.sleep()

if __name__ == "__main__":
    objective_node()
