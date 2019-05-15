#!/usr/bin/env python
import rospy
import can
from std_msgs.msg import String
from std_msgs.msg import Empty
from nova_common.msg import * #motor_arm
from base_station.msg import *
import time
bustype = 'socketcan_native'
channel = 'can1' #cantx is set virtual can, can set up by running the run_can.batch file in sim
bus = can.interface.Bus(channel=channel, bustype=bustype)  #Define the can bus interface to transmit on. 
global trig_r_init
trig_r_init = False
values = [0.0,0.0,0.0, 0.0, 0.0, 0.0,0.0] #[Ly, Lx, Ltw, Ry, Rx, Rtw,RClaw]
ids = [0x02, 0x03, 0x01, 0x04, 0x05, 0x06, 0x07]
resets = [0,0,0,0]
hbeat = False;
hbeat_cnt = 0;
max_hbeat = 3;
ignore_endstops = False;

def RightCallback(data):
    data_array = [-data.axis_ly_val,data.axis_lx_val,data.trig_l_val,data.trig_r_val]
    #rospy.set_param('base_station/drive_mode','RightDrive')
    #drive_mode = rospy.get_param('base_station/drive_mode')
    if rospy.get_param('base_station/drive_mode') == 'RightDrive':
         data_array = [0.0,0.0,0.435,0.435]
    for i in range(0,len(data_array)):
        if(i == 2 or i==3):
                if(resets[i] == 0 and data_array[i] !=0.0):
                      resets[i] = 1
                elif(resets[i] == 0 and data_array[i] ==0.0):
                      data_array[i] = 0.435

		sub_data = data_array[i]-0.435
		if sub_data>0.0:
			sub_data = sub_data/(1-0.435)
		else:
			sub_data = sub_data/(0.435)
		data_array[i]=sub_data
                if(i == 3):
                    data_array[i] = -sub_data
        values[i+3] = data_array[i]**1.8 if data_array[i]>0 else -(abs(data_array[i])**1.8)
	#rospy.loginfo(data_array[i])
        if data.but_b_trg == True:
            rospy.loginfo("Switching to joystick drive")
            rospy.set_param('base_station/drive_mode','RightDrive')
        if data.but_b_trg == True:
            if ignore_endstops==False:
                ignore_endstops = True
            else:
                ignore_endstops = False
            rospy.loginfo(ignore_endstops)


def LeftCallback(data):
    data_array = [-data.axis_ly_val,data.axis_lx_val,data.trig_l_val]
    for i in range(0,len(data_array)):
                  if(i == 2):
		        if(resets[1]==0 and data_array[i] !=0.0):
		              resets[1] = 1
		        elif(resets[1]==0 and data_array[i] ==0.0):
		              data_array[i] = 0.435

                        sub_data = data_array[i]-0.435
                        if sub_data>0.0:
                              sub_data = sub_data/(1-0.435)
                        else:
                              sub_data = sub_data/(0.435)
                        data_array[i]=sub_data
                  values[i] = data_array[i]**1.8 if data_array[i]>0.0 else -(abs(data_array[i])**1.8)
    if data.but_b_trg == True:
	rospy.loginfo("Switching to xbox drive")
	rospy.set_param('base_station/drive_mode','XboxDrive')

def HBeatCb():
  global hbeat
  global hbeat_cnt
  hbeat = True
  hbeat_cnt = 0

def listener():

    rospy.init_node('arm', anonymous=True)
    rospy.set_param('base_station/drive_mode','XboxDrive')
    rospy.Subscriber("/base_station/rjs_raw_ctrl", RawCtrl, RightCallback)
    rospy.Subscriber("/base_station/ljs_raw_ctrl", RawCtrl, LeftCallback)
    rospy.Subscriber("/heartbeat", Empty, HBeatCb)
    #rospy.spin()
    while(True):

 #	    sock.recv()
                global hbeat_cnt
                hbeat_cnt+=1
                if (hbeat_cnt > max_hbeat):
                    hbeat = False
		    for i in range(0,len(values)):
			field = 0x0
			complete_id = (ids[i] << 4)+field
			#complete_id = format(complete_id, '#013b')
			msg = can.Message(arbitration_id=complete_id, data=[], extended_id = False)
 			bus.send(msg)
			#rospy.loginfo("Id %s", field)
		else:
			for i in range(0,len(values)):
                                if (i==4 or i==6) and ignore_endstops == True:
                                     field = 0x5
                                     if values[i]<0:
                                         field = 0x6 
                                else:
				     field = 0x3
				     if values[i]<0:
					 field = 0x4
				complete_id = (ids[i] << 4)+field

				#complete_id = format(complete_id, '#013b')
				value = int(abs(values[i])*4095)
				if value<10:
					value = 0  #Getting rid of off centre
				bit1 = value>>8&0xFF
				bit2 = value&0xFF # format(value&0xFF,"#010b")
				#rospy.loginfo(bin(bit2))
				msg = can.Message(arbitration_id=complete_id, data=[bit1, bit2], extended_id = False)
	 			bus.send(msg)
				#rospy.loginfo("Id %s", field)
			
		time.sleep(0.1)

if __name__ == '__main__':
    listener()
