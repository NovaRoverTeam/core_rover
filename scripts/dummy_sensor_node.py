import rospy
from std_msgs.msg import Header
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

def generate_header(frame_id):
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = frame_id
    return header

def generate_odometry():
    odom = Odometry()
    odom.header = generate_header("dummy_header_odom")
    odom.pose.pose.position.x = 0
    odom.pose.pose.position.y = 1
    return odom

def generate_imu_data():
    imu = Imu()
    imu.header = generate_header("dummy_header_imu")

    imu.linear_acceleration.x = 1
    imu.linear_acceleration.y = 1
    imu.linear_acceleration.z = 1

    imu.angular_velocity.x = 2
    imu.angular_velocity.y = 1
    imu.angular_velocity.z = 3

    imu.orientation.x = 1
    imu.orientation.y = 1
    imu.orientation.z = 2
#   imu.orientation.w = 0

    return imu

def dummy_sensor():
    """
    robotic_pose_ekf subscribes to:
        * "odom", 2D pose: nav_msgs/Odometry
        * "imu_data", 3D orientation: sensor_msgs/Imu
        * "vo", 3D pose: nav_msgs/Odometry
    """

    odom_pub = rospy.Publisher("odom", Odometry, queue_size=10)
    imu_pub = rospy.Publisher("imu_data", Imu, queue_size=10)

    rospy.init_node("dummy_sensor")
    rate = rospy.Rate(100)

    while not rospy.is_shutdown():
        rospy.loginfo("dummy_sensor: publishing dummy sensor data")
        odom_pub.publish(generate_odometry())
        imu_pub.publish(generate_imu_data())
        rate.sleep()

if __name__ == "__main__":
    try:
        dummy_sensor()
    except rospy.ROSInterruptException:
        pass
