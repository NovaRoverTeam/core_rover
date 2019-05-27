#!/usr/bin/env python

import rospy
import math
from nav_msgs.msg import Odometry, OccupancyGrid, MapMetaData
from geometry_msgs.msg import Pose, Point, Quaternion
from sensor_msgs.msg import NavSatFix
import numpy as np
from scipy.ndimage import maximum_filter

from a_star import a_star

EPSILON = 10e-10

# constants involved in preprocessing occupancy grid before passing to A*
COST_OF_UNKNOWN = 1                     # convert unknowns (-1) to cost of 1
OBSTACLE_PROBABILITY_THRESHOLD = 85     # threshold over which cell is considered to contain unpassable obstacle
COST_OF_OBSTACLE = 10e10                # cost assigned to cells which are thought to contain obstacles

# node configuration
NODE_NAME = "planner_node"
ODOM_TOPIC = "/ekf/Odometry/global" # nav_msgs/Odommetry
MAP_TOPIC = "/rtabmap/grid_prob_map" # nav_msgs/OccupancyGrid
OBJECTIVE_POSE_TOPIC = "/planner/objective_pose" # geometry_msgs/Pose
WAYPOINT_TOPIC = "/planner/next_waypoint_pose" # nav_msgs/Odometry
BASE_LINK_SIZE = 1.2 # actual size of base link + padding

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

    origin_point = origin_pose.position
    point = pose.pose.pose.position
    # todo (naverill) validate this
    # size of the resolution
    k = 50
    x = int((point.x - origin_point.x) * k)
    y = int((point.y - origin_point.y) * k)

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
    map_resolution = map_info.resolution
    raw_grid = occupancy_grid.data

    # (1) reshape to 2D matrix
    matrix = np.reshape(raw_grid, (map_height, map_width))

    # (2) set costs of cells
    matrix[ matrix <= -1 ] = COST_OF_UNKNOWN
    matrix[ matrix >= OBSTACLE_PROBABILITY_THRESHOLD ] = COST_OF_OBSTACLE

    matrix = grid_to_max_grid(matrix, resolution)

    # todo (naverill) handle downsampled map -> path conversion

    return matrix

def grid_to_max_grid(occupancy_grid, resolution):
    """
    Downsamples the grid by applying a max pool and then
    """
    # base_link / map_resolution  = step_size * 3
    # ceil ensures value of step_size is >= 1
    step_size = math.ceil(BASE_LINK_WIDTH / (resolution * 3))

    # prevent against redundant processing
    if step_size <= 1:
        return occupancy_grid

    # converts grid from size n^2 to (n // step_size)^2, retaining obstacle info
    down_sampled_grid = max_pooling(occupancy_grid, step_size)

    max_filtered_grid = max_filter(occupancy_grid, step_size)

    return max_filtered_grid

def max_filter(occupancy_grid, step_size):
    """
    Applies a maximum filter to the occupancy grid, equivalent to the size of
    base_link. Sets each cell value to be equal to the max of all
    step_size*step_size surrounding cells. Each grid cell is now an accurate
    measure of its traversability by the rover of size BASE_LINK_WIDTH
    """
    return maximum_filter(occupancy_grid, size=step_size, mode="constant", cval=COST_OF_UNKNOWN)

def max_pooling(mat, step_size, pad=False):
    '''Non-overlapping pooling on 2D or 3D data.
    ref : <https://stackoverflow.com/questions/42463172/how-to-perform-max-mean-pooling-on-a-2d-array-using-numpy>

    mat         : ndarray, input array to pool.
    step_size   : tuple of 2, kernel size in (ky, kx).
    pad         : bool, pad <mat> or not. If no pad, output has size
                    n//f, n being <mat> size, f being kernel size.
                    if pad, output has size ceil(n/f).

    Return <result>: max pooled matrix.
    '''

    m, n = mat.shape[:2]
    ky, kx=ksize

    _ceil=lambda x, y: int(numpy.ceil(x / float(y)))

    if pad:
        ny = _ceil(m, ky)
        nx = _ceil(n, kx)
        size = ( ny * ky, nx * kx) + mat.shape[2:]
        mat_pad = numpy.full(size, COST_OF_UNKNOWN) # np.nan
        mat_pad[:m, :n, ...] = mat
    else:
        ny = m//ky
        nx = n//kx
        mat_pad = mat[:ny * ky, :nx * kx, ...]

    new_shape = (ny, ky, nx, kx) + mat.shape[2:]

    result = numpy.nanmax(mat_pad.reshape(new_shape),axis=(1, 3))
    return result


def gridcell_to_pose(cell):
    """
    Converts an (x,y) cell coordinate into a pose. Both are assumed to be i
    the "map" frame.
    """
    origin_position = state_occupancy_grid.info.origin.position
    origin_orientation = state_occupancy_grid.info.origin.orientation

    pose = Pose()

    # size of the resolution
    k = 50
    pose.position.x = cell[0] * k + origin_position.x
    pose.position.y = cell[1] * k + origin_position.y
    pose.position.z = origin_point.z
    pose.orientation = origin_orientation

    return pose

def odom_callback(msg):
    """
    Saves the most recent pose of the rover to internal state. Pose is assumed
    to be in the "map" frame.
    """
    # TODO: check if rover's pose is actually published in "map"
    global state_rover_pose
    state_rover_pose = msg

def map_callback(msg):
    """
    Saves the most recent occupancy grid to internal state.
    """
    global state_occupancy_grid
    state_occupancy_grid = msg

def objective_callback(msg):
    """
    Saves the current objective to the node's internal state.
    """
    global state_objective_pose
    state_objective_pose = msg

def get_objective_grid(occupancy_grid, start_grid, end_grid):
    """
    Returns the right objective grid cell
    1. When the objective is inside the occupancy grid, Return as it is
    2. When the objective is outside the occupancy grid,
       Return the conjunction of start-objective line and the edge of the grid
    """
    x_max = len(occupancy_grid[0]) - 1
    y_max = len(occupancy_grid) - 1

    # if the objective is outside the occuoancy grid
    if end_grid[0] < 0 or end_grid[0] > x_max \
        or end_grid[1] < 0 or end_grid[1] > y_max:
        # the line connecting end_grid and start_grid will have conjunctions with
        # all 4 sides of the edge, only the one between start_grid and end_grid
        # it the right objective

        if end_grid[0] == start_grid[0]:
            if end_grid[1] > start_grid[1]:
                end_grid = (end_grid[0], y_max)
            else:
                end_grid = (end_grid[0], 0)
        elif end_grid[1] == start_grid[1]:
            if end_grid[0] > start_grid[0]:
                end_grid = (x_max, end_grid[1])
            else:
                end_grid = (0, end_grid[1])
        else:
            k = (end_grid[1] - start_grid[1]) / (end_grid[0] - start_grid[0])
            b = start_grid[1] - k * start_grid[0]

            grids = []
            # top grid
            grids.append((int((y_max - b) // k), int(y_max)))
            # left grid
            grids.append((0, int(b)))
            # bottom grid
            grids.append((int(-b // k), 0))
            # right grid
            grids.append((int(x_max), int(k * x_max + b)))

            for grid in grids:
                if min(grid[0], start_grid[0], end_grid[0]) != grid[0] \
                    and max(grid[0], start_grid[0], end_grid[0]) != grid[0]:
                    if min(grid[1], start_grid[1], end_grid[1]) != grid[1] \
                        and max(grid[1], start_grid[1], end_grid[1]) != grid[1]:
                        end_grid = grid
    return end_grid

def get_smooth_waypoint(start, waypoints):
    """
    Returns the waypoint in a list of waypoints
    This waypoint is the furthest that the rover can go without changing directions
    """
    # start with the second waypoint in the list
    # the second waypoint could represent the turning directions of the rover
    next_index = 0
    next_point = waypoints[next_index]

    smooth_waypoint = next_point

    # the increment in coordinates for the next point if they are on the same line
    increment_point = (0, 0)

    # staight up/down
    if smooth_waypoint[0] == start[0]:
        increment_point = (smooth_waypoint[0] - start[0], 0)
    elif smooth_waypoint[1] == start[1]:
        increment_point = (0, smooth_waypoint[1] - start[1])
    # diagonal
    else:
        increment_point = (smooth_waypoint[0] - start[0], smooth_waypoint[1] - start[1])

    # investigate if the next point is on the same line
    # increment the index by 1 each time
    next_index += 1
    while next_index < len(waypoints):
        next_point_on_line = \
        (smooth_waypoint[0] + increment_point[0], smooth_waypoint[1] + increment_point[1])
        if next_point_on_line == waypoints[next_index]:
            smooth_waypoint = waypoints[next_index]
            next_index += 1
            continue
        break

    return smooth_waypoint

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
    objective_subscriber = rospy.Subscriber(OBJECTIVE_POSE_TOPIC, Odometry, objective_callback)

    # main control loop - generate waypoints over occupancy grid and publish
    waypoint_publisher = rospy.Publisher(WAYPOINT_TOPIC, Pose, queue_size=QUEUE_SIZE)
    publish_rate = rospy.Rate(PUBLISH_FREQUENCY)

    while not rospy.is_shutdown():

        if is_ready():
            rospy.loginfo("Planner ready to publish.")

            start = pose_to_gridcell(state_rover_pose)
            end = pose_to_gridcell(state_objective_pose)
            grid = preprocess_occupancy_grid(state_occupancy_grid)

            # handle out-of-grid issue, get the right end grid
            end = get_objective_grid(grid, start, end)

            # call A* for waypoints
            waypoints = a_star(grid, start, end)

            if len(waypoints) == 0:
                rospy.loginfo("Not publishing - no waypoints returned by A*.")
            else:
                rospy.loginfo("Publishing waypoints retuend by A*.")
                waypoint = get_smooth_waypoint(start, waypoints)
                waypoint_publisher.publish(gridcell_to_pose(waypoint))

        else:
            rospy.loginfo("Planner not ready to publish.")

        publish_rate.sleep()

if __name__ == "__main__":
    planner()
