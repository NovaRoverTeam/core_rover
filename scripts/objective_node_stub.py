import rospy
from sensor_msgs.msg import NavSatFix

state_objective_latitude = None
state_objective_longitude = None

def is_ready():
    return True

def handle_receive_objective(req):
    state_objective_latitude = req.latitude
    state_objective_longitude = req.longitude

def objective_node():

    # node should update objective when notified by server
    state_objective_latitude = -37.8136
    state_objective_longitude = 144.9631

    # publish objective
    objective_publisher = rospy.Publisher("/core_rover/current_objective", NavSatFix, queue_size=1)

    rospy.init_node("objective_node")
    publish_rate = rospy.Rate(2)
    while not rospy.is_shutdown():
        if is_ready():
            rospy.loginfo("Publishing objective.")
            msg = NavSatFix()
            msg.header.stamp = rospy.get_rostime()
            msg.latitude = state_objective_latitude
            msg.longitude = state_objective_longitude
            objective_publisher.publish(msg)
        else:
            rospy.loginfo("Not publishing objective.")
        publish_rate.sleep()

if __name__ == "__main__":
    objective_node()