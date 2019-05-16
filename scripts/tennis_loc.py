#!/usr/bin/env python
import rospy, math,numpy, ast

from webots_ros.srv import set_float
from nova_common.msg import *
from nova_common.srv import *
import time
import socket
import sys
import os
import subprocess
from std_msgs.msg import String

debug = False

#--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
# getAutoMode(): Retrieve Mode from parameter server.
#--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--
def getAutoMode():
    return rospy.get_param('/core_rover/autonomous_mode','On')
def X_Zone(x):
    zones = ('left','centre','right')
    centre_border = (240,480)
    x = int(x)
    if x<centre_border[0]:
        return 'left'
    if x>=centre_border[0] and x<=centre_border[1]:
        return 'centre'
    if x>centre_border[1]:
        return 'right'
def Drive_Forward(w,h,threshold):
    try:
        w = float(w)
        h = float(h)
    except Exception() as me:
        print(me)
    avg = sum([w,h])/float(2)
    #rospy.loginfo("avg: %d", avg)
    return avg<threshold

def DriverMsg(data,steer_limit,rpm_limit):
    steer = {'left':-steer_limit * 50,'centre':0,'right':steer_limit*50}
    cmd = DriveCmd()
    cmd.rpm = 0
    direction = X_Zone(data[0])
    forward_or_not = Drive_Forward(data[2],data[3],60)
    #rospy.loginfo(forward_or_not)
    cmd.steer_pct = steer[direction]
    if (direction == 'centre' and forward_or_not):
        cmd.rpm = rpm_limit * 50
    else:
        cmd.rpm = 0
    return cmd

def tennis_loc():
    global debug
    server_address = '/tmp/tennis_loc_socket'
	    # Make sure the socket does not already exist
    try:
        os.unlink(server_address)
    except OSError:
        if os.path.exists(server_address):
            raise
    sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    ball_pub  = rospy.Publisher("/core_rover/driver/drive_cmd2", DriveCmd, queue_size=10)
    status_pub = rospy.Publisher("/core_rover/navigation/tennis_stat", String, queue_size=10)
    rospy.init_node('tennis_loc', anonymous=True)
    rate_active = rospy.Rate(30) # Loop rate in Hz
    rate_inactive = rospy.Rate(1)
    rospy.loginfo("tennis ball location node started")
    autonomous_mode = 'Off'
    vision_process = subprocess.Popen(['python3', '/home/nvidia/Documents/NeuralNet/YOLO3-4-Py/V2_Foscam_2.py'])
    sock.bind(server_address)
    sock.listen(1)
    connection, client_address = sock.accept()
    sock.setblocking(0)
    steer_limit = rospy.get_param('steer_limit',0.3)
    rpm_limit   = rospy.get_param('rpm_limit',0.3)
    while not rospy.is_shutdown():
    	if getAutoMode() != 'Off': 
          data = connection.recv(100) # 1 Byte per character
          if data != '0':
              data_parsed = ast.literal_eval(data)
              #print(data_parsed)
              #array = Float32MultiArray(4,data_parsed)
              msg = DriverMsg(data_parsed,steer_limit,rpm_limit)
              if msg.rpm==0 and msg.steer_pct==0:
                  status_pub.publish("Complete")
                  #rospy.loginfo("Complete")
              else:
                  status_pub.publish("Found")
                  #rospy.loginfo("Found")
              if getAutoMode() == "Destroy" or debug:
                  ball_pub.publish(msg)
          else:
              status_pub.publish("Lost")
              #rospy.loginfo("Lost")
          rate_active.sleep()
    	else:
          rate_inactive.sleep()
    connection.close()
    vision_process.terminate()

if __name__ == '__main__':
    tennis_loc()
