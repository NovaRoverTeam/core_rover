#!/usr/bin/env python
import rospy
import math
import time
from sensor_msgs.msg import NavSatFix, MagneticField
from webots_ros.srv import set_float
from nova_common.msg import *
from nova_common.srv import *

x=0
z=0

def compassCallback(compassData):
    global x
    global z
    x = compassData.magnetic_field.x
    z = compassData.magnetic_field.y

def bearing():
    global x
    global z
    rospy.init_node('bearing', anonymous=True)
    rate = rospy.Rate(2) # Loop rate in Hz
    compass_sub = rospy.Subscriber("/nova_common/MagnetometerFiltered", MagneticField, compassCallback)
    while not rospy.is_shutdown():
				bearing = math.atan2(z,x)*180/math.pi
				#radnorth = math.atan2(-0.8370561,-0.40941954) # north vector 1,0,0
				#bearing = (rad)*(180/math.pi)

				if bearing < 0:
						bearing = bearing + 360.0
				elif bearing > 360:
						bearing = bearing - 360.0

				#bearing = bearing - 180
				rospy.loginfo("bearing: %s", bearing)

				rate.sleep()


if __name__ == '__main__':
    bearing()

