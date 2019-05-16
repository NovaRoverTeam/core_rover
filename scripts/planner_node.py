import rospy
import math
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import Pose, Point, Quaternion
from sensor_msgs.msg import NavSatFix

import numpy as np

from a_star import a_star

EPSILON = 10e-10

# constants involved in preprocessing occupancy grid before passing to A*
COST_OF_UNKNOWN = 1                     # convert unknowns (-1) to cost of 1
OBSTACLE_PROBABILITY_THRESHOLD = 85     # threshold over which cell is considered to contain unpassable obstacle
COST_OF_OBSTACLE = 10e10                # cost assigned to cells which are thought to contain obstacles

# node configuration
NODE_NAME = "planner"
ODOM_TOPIC = "/ekf/Odometry" # nav_msgs/Odommetry
MAP_TOPIC = "/rtabmap/grid_map" # nav_msgs/OccupancyGrid
OBJECTIVE_TOPIC = "/objective_pose" # geometry_msgs/Pose
WAYPOINT_TOPIC = "/planner/waypoints" # nav_msgs/NavSatFix

QUEUE_SIZE = 10
PUBLISH_FREQUENCY = 10
LOG_LEVEL = rospy.DEBUG

# node's internal state, all data required for planning
state_rover_pose = None
state_objective_pose = None
state_occupancy_grid = None

def is_ready():
    """
    Checks that the node has received sufficient data to begin planning -
    namely, the rover's current pose, and the occupancy grid.
    """
    return state_rover_pose is not None \
           and state_objective_pose is not None \
           and state_occupancy_grid is not None

def pose_to_gridcell(pose):
    """
    Converts a pose (x,y) tuple to a cell coordinate (x,y) in the occupancy
    grid. Assumes both the pose and the grid to be in the "map" frame.
    """
    origin_pose = state_occupancy_grid.info.origin

    origin_point = origin_pose.point
    point = pose.point
    
    x = int(point.x - origin_point.x)
    y = int(point.y - origin_point.y)

    return (x, y)

def preprocess_occupancy_grid(occupancy_grid):
    """
    Preprocesses the raw occupancy grid prior to passing it to the waypointing
    algorithm:
        (1) Reshape 1D array into a 2D matrix;
        (2) Set cost of cells with high obstacle probability to infinity; and
        (3) Apply KxK max convolution to 2D matrix.
    """
    map_info = occupancy_grid.info
    map_width = map_info.width
    map_height = map_info.height
    raw_grid = occupancy_grid.data

    # (1) reshape to 2D matrix
    matrix = np.reshape(raw_grid, (map_height, map_width))

    # (2) set costs of cells 
    matrix[ matrix <= -1 ] = COST_OF_UNKNOWN
    matrix[ matrix >= OBSTACLE_PROBABILITY_THRESHOLD ] = COST_OF_OBSTACLE

    # (3) TODO: apply max convolution

    return matrix

def gridcell_to_pose(cell):
    """
    Converts an (x,y) cell coordinate into a pose. Both are assumed to be i
    the "map" frame.
    """
    origin_point = state_occupancy_grid.info.origin.point
    origin_orientation = state_occupancy_grid.info.origin.orientation

    pose = Pose()
    pose.position.x = orgin_point.x - cell[0]
    pose.position.y = orgin_point.y - cell[1]
    pose.position.z = orgin_point.z
    pose.orientation = origin_orientation

    return pose

def odom_callback(msg):
    """
    Saves the most recent pose of the rover to internal state. Pose is assumed
    to be in the "map" frame.
    """
    # TODO: check if rover's pose is actually published in "map"
    state_rover_pose = msg

def map_callback(msg):
    """
    Saves the most recent occupancy grid to internal state.
    """
    state_occupancy_grid = msg

def objective_callback(msg):
    """
    Saves the current objective to the node's internal state.
    """
    state_objective_pose = msg

def planner():
    """
    The "main" function for the rosnode. Initializes the node and runs the
    main control loop.
    """
    # node initalization
    rospy.init_node(NODE_NAME, log_level=LOG_LEVEL)

    # subscribe to odom and grid_map
    odom_subscriber = rospy.Subscriber(ODOM_TOPIC, Odometry, odom_callback)
    grid_map_subscriber = rospy.Subscriber(MAP_TOPIC, OccupancyGrid, map_callback)
    objective_subscriber = rospy.Subscriber(OBJECTIVE_TOPIC, NavSatFix, objective_callback)

    # main control loop - generate waypoints over occupancy grid and publish
    waypoint_publisher = rospy.Publisher(WAYPOINT_TOPIC, Pose, queue_size=QUEUE_SIZE)
    publish_rate = rospy.Rate(PUBLISH_FREQUENCY)
    while not rospy.is_shutdown():
        if is_ready() or True:
            rospy.logdebug("Planner ready to publish.")
            start = pose_to_gridcell(state_rover_pose)
            end = latlon_to_gridcell(state_objective_pose)
            grid = preprocess_occupancy_grid(state_occupancy_grid)
            #waypoints = [gridcell_to_pose(wp) for wp in a_star(grid, start, end)]
            waypoints = [wp for wp in a_star(grid, start, end)]
            waypoint_publisher.publish(gridcell_to_pose(waypoints[0]))
        else:
            rospy.logdebug("Planner not ready to publish.")
        publish_rate.sleep()

if __name__ == "__main__":
    planner()
