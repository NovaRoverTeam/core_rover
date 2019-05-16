import rospy
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import Pose, Point, Quaternion
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String

from a_star import a_star

# node configuration
NODE_NAME = "planner"
ODOM_TOPIC = "/ekf/Odometry" # nav_msgs/Odommetry
MAP_TOPIC = "/rtabmap/grid_map" # nav_msgs/OccupancyGrid
OBJECTIVE_TOPIC = "/objective" # nav_msgs/NavSatFix
WAYPOINT_TOPIC = "/planner/waypoints" # probably nav_msgs/NavSatFix[]

QUEUE_SIZE = 10
PUBLISH_FREQUENCY = 10
LOG_LEVEL = rospy.DEBUG

# node's internal state, all data required for planning
state_current_pose = None
state_current_objective = None
state_grid_origin_pose = None
state_occupancy_grid = None

def is_ready():
    """
    Checks that the node has received sufficient data to begin planning -
    namely, the rover's current pose, and the occupancy grid.
    """
    return state_current_pose is not None and \
           state_current_objective is not None and \
           state_grid_origin_pose is not None and \
           state_occupancy_grid is not None

def pose_to_gridcell(pose):
    """
    Converts a pose (x,y) tuple to a cell coordinate (x,y) in the occupancy
    grid. Assumes both the pose and the grid to be in the "map" frame.
    """
    if pose == start_grid_origin_pose:
        return (0, 0)
    origin_point = start_grid_origin_pose.point

    point = pose.point

    x = point.x - origin_point.x
    y = point.y - origin_point.y

    return (x, y)

def latlon_to_gridcell(latlon):
    """
    Converts a latlon (lat,lon) tuple to a cell coordinate (x,y) in the
    occupancy grid. Assumes the grid to be in the "map" frame.
    """
    # TODO
    return (0, 0)

def preprocess_occupancy_grid(raw_grid):
    """
    Preprocesses the raw occupancy grid prior to passing it to the waypointing
    algorithm:
        (1) Reshape 1D array into a 2D matrix;
        (2) Set cost of cells with high obstance probability to infinity; and
        (3) Apply KxK max convolution to 2D matrix.
    """
    # TODO
    return raw_grid

def gridcell_to_pose(cell):
    """
    Converts an (x,y) cell coordinate into a pose. Both are assumed to be i
    the "map" frame.
    """
    if cell == (0, 0):
        return state_grid_origin_pose
    
    origin_point = start_grid_origin_pose.point
    pose = Pose()

    pose.position.x = orgin_point.x - cell[0]
    pose.position.y = orgin_point.y - cell[1]
    pose.position.z = orgin_point.z

    pose.orientation = state_grid_origin_pose.orientation

    return pose

def odom_callback(msg):
    """
    Saves the most recent pose of the rover to internal state. Pose is assumed
    to be in the "map" frame.
    """
    # TODO: check if rover's pose is actually published in "map"
    state_current_pose = msg.pose
    state_current_twist = msg.twist

def map_callback(msg):
    """
    Saves the most recent occupancy grid to internal state.
    """
    state_grid_origin_pose = msg.info.origin
    state_occupancy_grid = msg.data

def objective_callback(msg):
    """
    Saves the current objective to the node's internal state.
    """
    state_objective_pose = gps_to_pose(msg.latitude, msg.longitude)

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
    waypoint_publisher = rospy.Publisher(WAYPOINT_TOPIC, String, queue_size=QUEUE_SIZE)
    publish_rate = rospy.Rate(PUBLISH_FREQUENCY)
    while not rospy.is_shutdown():
        if is_ready() or True:
            rospy.logdebug("Planner ready to publish.")
            start = pose_to_gridcell(state_current_pose)
            end = latlon_to_gridcell(state_current_objective)
            grid = preprocess_occupancy_grid(state_occupancy_grid)
            waypoints = [gridcell_to_pose(wp) for wp in a_star(grid, start, end)]
            # TODO: waypoint smoothing
            waypoint_publisher.publish(str(waypoints))
        else:
            rospy.logdebug("Planner not ready to publish.")
        publish_rate.sleep()

if __name__ == "__main__":
    try:
        planner()
    except rospy.ROSInterruptException:
        pass
