import rospy
from std_msgs.msg import Header
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix

def generate_header(frame_id):
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = frame_id
    return header

def generate_navsatfix():
    nsf = NavSatFix()
    nsf.header = generate_header("base_footprint")
    nsf.latitude = 35.0
    nsf.longitude = 35.0
    nsf.altitude = 1.0
    nsf.position_covariance = [1,1,1,1,1,1,1,1,1]
    return nsf

def generate_odometry():
    odom = Odometry()
    odom.header = generate_header("dummy_header_odom")
    odom.pose.pose.position.x = 0
    odom.pose.pose.position.y = 1
    return odom

def generate_imu_data():
    imu = Imu()
    imu.header = generate_header("dummy_header_imu")

    imu.linear_acceleration.x = 0
    imu.linear_acceleration.y = 0
    imu.linear_acceleration.z = 9.8 

    imu.angular_velocity.x = 0
    imu.angular_velocity.y = 0
    imu.angular_velocity.z = 0

    imu.orientation.x = 0.3
    imu.orientation.y = 0.1
    imu.orientation.z = 0.5

    return imu

def dummy_sensor():
    """
    utm_odometry subscribes to:
        * "/nova_common/gps_data", type: sensor_msgs/NavSatFix
    utm_odometry publishes to:
        * "vo",  type: nav_msgs/Odometry (yes, we are using VO to publish GPS)

    robotic_pose_ekf subscribes to:
        * "odom", type: nav_msgs/Odometry
        * "imu_data", type: sensor_msgs/Imu
        * "vo", type: nav_msgs/Odometry
    """

#    odom_pub = rospy.Publisher("odom", Odometry, queue_size=1000)
    imu_pub = rospy.Publisher("imu_data", Imu, queue_size=1000)
    #gps_pub = rospy.Publisher("/nova_common/gps_data", NavSatFix, queue_size=1000)

    rospy.init_node("dummy_sensor")
    rate = rospy.Rate(3)

    while not rospy.is_shutdown():
        rospy.loginfo("dummy_sensor: publishing dummy sensor data")
 #       odom_pub.publish(generate_odometry())
        imu_pub.publish(generate_imu_data())
        #gps_pub.publish(generate_navsatfix())
        rate.sleep()

if __name__ == "__main__":
    try:
        dummy_sensor()
    except rospy.ROSInterruptException:
        pass
