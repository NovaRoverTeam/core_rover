import rospy
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Pose
from std_msgs.msg import String

NODE_NAME = "controller_node"
ROVER_ODOM_TOPIC = "/ekf/Odometry"
ROVER_GPS_TOPIC = "/ekf/gps"
WAYPOINT_TOPIC = "/planner/next_waypoint"
OBJECTIVE_TOPIC = "/planner/current_objective_pose"
DRIVE_CMD_TOPIC = "/nova_common/drive_cmd"

state_rover_pose = None
state_rover_gps = None
state_waypoint_pose = None
state_objective_gps = None

def is_ready():
    return state_rover_pose != None\
           and state_rover_gps != None\
           and state_waypoint_pose != None
           and state_objective_gps != None

def odom_callback(msg):
    state_rover_pose = msg

def gps_callback(msg):
    state_rover_gps = msg

def waypoint_callback(msg):
    state_waypoint_pose = msg

def objective_callback(msg):
    state_objective_gps = msg

def construct_drive_cmd(current_pose, waypoint_pose):
    # TODO
    return "STUB" 

def near_global_objective(rover_gps, objective_gps):
    # TODO
    return False

def controller_node():

    # node initalization
    rospy.init_node(NODE_NAME)

    # subscribe to rover's current pose and gps
    odom_subscriber = rospy.Subscriber(ROVER_ODOM_TOPIC, Odometry, odom_callback)
    gps_subscriber = rospy.Subscriber(ROVER_GPS_TOPIC, NavSatFix, gps_callback)

    # subscribe the the next waypoint from the planner, and the global objective
    waypoint_subscriber = rospy.Subscriber(WAYPOINT_TOPIC, Pose, waypoint_callback)
    objective_subscriber = rospy.Subscriber(OBJECTIVE_TOPIC, NavSatFix, objective_callback)

    # publish drive commands
    drive_cmd_publisher = rospy.Publisher(DRIVE_CMD_TOPIC, String, queue_size=10)
    publish_rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        if is_ready():
            # if we're reached vicinity of the global objective ...
            if near_global_objective(state_rover_gps, state_objective_gps):
                # ...  TODO: advise state machine to switch to spiral search
                pass

            # otherwise continue driving towards the next waypoint 
            else:
                drive_cmd = construct_drive_cmd(state_rover_pose, state_waypoint_pose)
                drive_cmd_pubisher.publish(drive_cmd)

        publish_rate.sleep()

if __name__ == "__main__":
    controller_node()