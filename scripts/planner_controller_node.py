#!/usr/bin/env python

import rospy
import tf
from math import sin, cos, radians, atan2, sqrt, degrees
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from std_msgs.msg import String
from nova_common.msg import DriveCmd

NODE_NAME = "controller_node"
ROVER_ODOM_TOPIC = "/ekf/Odometry"
ROVER_GPS_TOPIC = "/ekf/gps"
WAYPOINT_TOPIC = "/core_rover/planner/next_waypoint_pose"
DRIVE_CMD_TOPIC = "/core_rover/planner/drive_cmd"

state_rover_pose = None
state_rover_gps = None
state_waypoint_pose = None

def is_ready():
    return state_rover_pose != None\
           and state_rover_gps != None\
           and state_waypoint_pose != None

def odom_callback(msg):
    state_rover_pose = msg

def gps_callback(msg):
    state_rover_gps = msg

def waypoint_callback(msg):
    state_waypoint_pose = msg

def construct_drive_cmd(current_pose, waypoint_pose):
    angle = angle_of_poses(current_pose, waypoint_pose)
    drive_msg = DriveCmd()
    drive_msg.rpm = 10
    orientation = get_orientation(current_pose)
    drive_msg.steer_pct = turn_direction(beta, orientation)

    return drive_msg

def get_orientation(pose):
    return degrees(tf.getYaw(pose.orientation))

def angle_of_poses(current_pose, waypoint_pose):
    current_x = current_pose.point.x
    current_y = current_pose.point.y

    waypoint_x = waypoint_pose.point.x
    waypoint_y = waypoint_pose.point.y

    current_x = radians(current_pose.point.x)
    waypoint_x = radians(waypoint_pose.point.x)

    y_diff = radians(waypoint_y - current_y)
    y = sin(y_diff) * cos(current_y)
    x = cos(current_x) * sin(waypoint_x) - sin(current_x) * cos(waypoint_x) * cos(y_diff)

    beta = atan2(y, x)
    beta = degrees(beta)
    beta = (beta+360) % 360
    
    return beta
     
def turn_direction(beta, orientation):
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
            
    rospy.loginfo("turn: %s", turn)        
    return turn

def controller_node():

    # node initalization
    rospy.init_node(NODE_NAME)

    # subscribe to rover's current pose and gps
    odom_subscriber = rospy.Subscriber(ROVER_ODOM_TOPIC, Odometry, odom_callback)
    gps_subscriber = rospy.Subscriber(ROVER_GPS_TOPIC, NavSatFix, gps_callback)

    # subscribe the the next waypoint from the planner
    waypoint_subscriber = rospy.Subscriber(WAYPOINT_TOPIC, Pose, waypoint_callback)

    # publish drive commands
    drive_cmd_publisher = rospy.Publisher(DRIVE_CMD_TOPIC, DriveCmd, queue_size=10)
    publish_rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        if is_ready():
            drive_cmd = construct_drive_cmd(state_rover_pose, state_waypoint_pose)
            drive_cmd_pubisher.publish(drive_cmd)

        publish_rate.sleep()

if __name__ == "__main__":
    controller_node()