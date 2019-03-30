#!/usr/bin/env python
import rospy, math,numpy, ast

from webots_ros.srv import set_float
from nova_common.msg import *
from nova_common.srv import *
import time
import socket
import sys
import os
from std_msgs.msg import String

def X_Zone(x):
    zones = ('left','centre','right')
    centre_border = (426,853)
    x = int(x)
    if x<426:
        return 'left'
    if x>=426 and x<=853:
        return 'centre'
    if x>853:
        return 'right'
def Drive_Forward(w,h,threshold):
    try:
        w = float(w)
        h = float(h)
    except Exception() as me:
        print(me)
    avg = sum([w,h])/float(2)
    return avg<threshold
    
def DriverMsg(data,steer_limit,rpm_limit):
    steer = {'left':-steer_limit,'centre':0,'right':steer_limit}
    cmd = DriveCmd()
    cmd.rpm = 0
    direction = X_Zone(data[0])
    rospy.loginfo(Drive_Forward(data[2],data[3],200))
    cmd.steer_pct = steer[direction]
    if (direction == 'centre' and Drive_Forward(data[2],data[3],200)):
        cmd.rpm = rpm_limit
    else:
        cmd.rpm = 0     
    return cmd

def tennis_loc():
    server_address = '/home/nvidia/Documents/YOLO3-4-Py/mysocket'
	    # Make sure the socket does not already exist
    try:
        os.unlink(server_address)
    except OSError:
        if os.path.exists(server_address):
            raise
    sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    sock.bind(server_address)
    sock.listen(1)
    ball_pub  = rospy.Publisher("/core_rover/navigation/tennis_loc", DriveCmd, queue_size=10)
    rospy.init_node('tennis_loc', anonymous=True)
    rate = rospy.Rate(30) # Loop rate in Hz
    rospy.loginfo("tennis ball location node started")

    while not rospy.is_shutdown():
    	connection, client_address = sock.accept()
    	while True:
        	steer_limit = rospy.get_param('steer_limit')
        	rpm_limit   = rospy.get_param('rpm_limit')
            data = connection.recv(100) # 1 Byte per character
            if data:
                #rospy.loginfo(data)
                data_parsed = ast.literal_eval(data)
                #print(data_parsed)
                #array = Float32MultiArray(4,data_parsed)
                msg = DriverMsg(data_parsed,steer_limit,rpm_limit)
                rospy.loginfo('publishing: ' + str(msg) + str(data_parsed))
                ball_pub.publish(msg)
            else:
                connection.close()
                break;


if __name__ == '__main__':
    tennis_loc()
