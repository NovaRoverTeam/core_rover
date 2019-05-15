import rospy
from sensor_msgs.msg import NavSatFix
from nova_common.msg import *
from nova_common.srv import *

state_objective_latitude = None
state_objective_longitude = None

def is_ready():
    return state_objective_latitude != None and\
           state_objecctive_longitude != None

def handle_receive_objective(req):
    state_objective_latitude = req.latitude
    state_objective_longitude = req.longitude

def objective_node():

    # node should update objective when notified by server
    server = Rospy.Service("/core_rover/start_auto", StartAuto, handle_receive_objective)

    # publish objective
    objective_publisher = rospy.Publisher("/core_rover/current_objective", NavSatFix, queue_size=1)

    rospy.init_node("objective_node")
    publish_rate = rospy.Rate(2)
    while not rospy.is_shutdown():
        if is_ready():
            msg = NavSatFix()
            msg.header.stamp = rospy.get_rostime()
            msg.latitude = state_objective_latitude
            msg.longitude = state_objective_longitude
            objective_publisher.publish(msg)
        else:
            rospy.logdebug("Objective node not ready to publish.")
        publish_rate.sleep()

if __name__ == "__main__":
    objective_node()
