#!/usr/bin/env python
import rospy
import can
from std_msgs.msg import String
from nova_common.msg import * #motor_arm
from base_station.msg import *
import time
bustype = 'socketcan_native'
channel = 'can0' #cantx is set virtual can, can set up by running the run_can.batch file in sim
bus = can.interface.Bus(channel=channel, bustype=bustype)  #Define the can bus interface to transmit on. 
global trig_r_init
trig_r_init = False
values = [0.0,0.0,0.0, 0.0, 0.0, 0.0,0.0] #[Ly, Lx, Ltw, Ry, Rx, Rtw,RClaw]
ids = [0x02, 0x03, 0x01, 0x04, 0x05, 0x06, 0x07]
def RightCallback(data):
    data_array = [data.axis_ly_val,data.axis_lx_val,data.trig_l_val,data.trig_r_val]
    global trig_r_init
    for i in range(0,len(data_array)):
		if(i==3):
			if(trig_r_init==False and data_array[i] == 0.0):
				data_array[i] = 0.435
			else:
				trig_r_init = True
			
		if(i == 2 or i==3):
			sub_data = data_array[i]-0.435
			if sub_data>0.0:
				sub_data = sub_data/(1-0.435)
			else:
				sub_data = sub_data/(0.435)
			data_array[i]=sub_data
		values[i+3] = data_array[i]**1.8 if data_array[i]>0 else -(abs(data_array[i])**1.8)
		rospy.loginfo(data_array[i])
#    msg = can.Message(arbitration_id=0x32, data=[0x32, 0x32], extended_id = False) 
#    can_bus_send(msg)

def LeftCallback(data):
    data_array = [data.axis_ly_val,data.axis_lx_val,data.trig_l_val]
    for i in range(0,len(data_array)):
		if(i == 2):
			sub_data = data_array[i]-0.435
			if sub_data>0.0:
				sub_data = sub_data/(1-0.435)
			else:
				sub_data = sub_data/(0.435)
			data_array[i]=sub_data
		values[i+3] = data_array[i]**1.8 if data_array[i]>0.0 else -(abs(data_array[i])**1.8)
		#rospy.loginfo(data_array[i])
   # rospy.loginfo(data_array[2])
def listener():

    rospy.init_node('arm', anonymous=True)

    rospy.Subscriber("/base_station/rjs_raw_ctrl", RawCtrl, RightCallback)
    rospy.Subscriber("/base_station/ljs_raw_ctrl", RawCtrl, LeftCallback)

 #   rospy.spin()
    while(True):
 #	    sock.recv()
#		rospy.loginfo("Forwards %.2f", values[3])
#		rospy.loginfo("Right %.2f", values[4])
#		rospy.loginfo("Twist %.2f", values[5])
#		rospy.loginfo(values[2])
		for i in range(0,len(values)):
			field = 0x3
			if values[i]<0:
				field = 0x4
			complete_id = (ids[i] << 4)+field
			#complete_id = format(complete_id, '#013b')
			value = int(abs(values[i])*4095)
			if value<20:
				value = 0  #Getting rid of off centre
			bit1 = value>>8&0xFF
			bit2 = value&0xFF#format(value&0xFF,"#010b")
			#rospy.loginfo(bin(bit2))
			msg = can.Message(arbitration_id=complete_id, data=[bit1, bit2], extended_id = False)
 			bus.send(msg)
			#rospy.loginfo("Id %s", complete_id)
			
		time.sleep(1)

if __name__ == '__main__':
    listener()
