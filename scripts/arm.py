#!/usr/bin/env python
import rospy
import can
from std_msgs.msg import String
from std_msgs.msg import Empty
from nova_common.msg import * #motor_arm
import time
import Jetson.GPIO as GPIO
bustype = 'socketcan_native'
channel = 'can1' #cantx is set virtual can, can set up by running the run_can.batch file in sim
bus = can.interface.Bus(channel=channel, bustype=bustype)  #Define the can bus interface to transmit on. 

reset = False
values = [0.0,0.0,0.435, 0.0, 0.0, 0.435, 0.435] #[Ly, Lx, Lt, Ry, Rx, Rtw,RClaw]
ids = [0x02, 0x03, 0x01, 0x04, 0x05, 0x06, 0x07] 
resets = [0,0,0,0] #Array used to record resets on 0-1 joystick control 
hbeat = False;
hbeat_cnt = 0;
max_hbeat = 15;
ignore_endstops = False;
toggle_finger = False;
finger_debouncer = 3; #counter for extending finger debounce
prev_finger = False

def RightCallback(data):
    global finger_debouncer #Used to debounce button press for finger extension
    global toggle_finger
    global ignore_endstops #When true, endeffector and wrist movement will send commands to ignore endstops
    data_array = [-data.axis_ly_val,data.axis_lx_val,data.trig_l_val,data.trig_r_val] #array from joystick commands
    if rospy.get_param('base_station/drive_mode') == 'RightDrive': 
         data_array = [0.0,0.0,0.435,0.435]
    for i in range(0,len(data_array)):
        """if(i == 2 or i==3):  #if continuous rotation or end effector side switch
              if(resets[i] == 0 and data_array[i] !=0.0):
                   resets[i] = 1
              elif(resets[i] == 0 and data_array[i] ==0.0):
                    data_array[i] = 0.435
              sub_data = data_array[i]-0.435 #Centering 0-1 joystick commands
              if sub_data>0.0:
                sub_data = sub_data/(1-0.435)
              else:
                sub_data = sub_data/(0.435)
              data_array[i]=sub_data
              if(i==2):
                data_array[i] = -sub_data #Reverse endeffector closing """
        values[i+3] = data_array[i]**1.8 if data_array[i]>0 else -(abs(data_array[i])**1.8) #Scaling response for higher preceision at low range
    if data.but_b_trg == True: 
        ignore_endstops = True
    else:
        ignore_endstops = False
    
    if data.but_y_trg == True and finger_debouncer>4: # Extending finger with screwdriver
        toggle_finger = True
        rospy.loginfo("Switching to joystick drive")
        finger_debouncer = 0

def LeftCallback(data):
    global reset
    data_array = [-data.axis_ly_val,data.axis_lx_val,data.trig_l_val] 
    print(data_array)
    for i in range(0,len(data_array)):
        """if(i == 2): #If base rotation 
            if(resets[1]==0 and data_array[i] !=0.0):
                  resets[1] = 1
            elif(resets[1]==0 and data_array[i] ==0.0):
                  data_array[i] = 0.435
            sub_data = data_array[i]-0.435  #Centering 0-1 joystick commands
            if sub_data>0.0:
                  sub_data = sub_data/(1-0.435)
            else:
                  sub_data = sub_data/(0.435)
            data_array[i]=sub_data
            """
        values[i] = data_array[i]**1.8 if data_array[i]>0.0 else -(abs(data_array[i])**1.8) #Scaling response for higher preceision at low range
    if data.but_x_trg == True:  
      rospy.loginfo("Switching to xbox drive")
      rospy.set_param('base_station/drive_mode','XboxDrive')
    if data.but_b_trg == True:
      rospy.loginfo("0's")
      reset = True
    else:
      reset = False
def HBeatCb(data):
	  global hbeat
	  global hbeat_cnt
	  global ignore_endstops
	  hbeat = True
	  hbeat_cnt = 0

def listener():
    rospy.init_node('arm', anonymous=False)
    rospy.set_param('base_station/drive_mode','XboxDrive')
    rospy.Subscriber("/base_station/rjs_raw_ctrl", RawCtrl, RightCallback)
    rospy.Subscriber("/base_station/ljs_raw_ctrl", RawCtrl, LeftCallback)
    rospy.Subscriber("/heartbeat", Empty, HBeatCb)

    GPIO.setmode(GPIO.BOARD)
    GPIO.setwarnings(False)
    GPIO.setup(31,GPIO.OUT)
    while(True):
      global finger_debouncer
      global toggle_finger
      global hbeat_cnt
      global ignore_endstops
      global reset
      global values
      global prev_finger
      rospy.loginfo(ignore_endstops)
      hbeat_cnt+=1
      if False and ((hbeat_cnt > max_hbeat) or (reset == True)):
         hbeat = False
         values = [0.0,0.0,0.0, 0.0, 0.0, 0.0, 0.0]
         for i in range(0,len(values)):  #Sending stop commands to all
 	        field = 0x0
	        complete_id = (ids[i] << 4)+field #Embedding ID's with 0 command
	        msg = can.Message(arbitration_id=complete_id, data=[], extended_id = False) #Creating can message to send
	        bus.send(msg)
      else:
        for i in range(0,len(values)):
          if (i==4 or i==6) and (ignore_endstops == True): #If on ignore endstops mode
             field = 0x5
             if values[i]<0:
                field = 0x6
             value = int(abs(values[i])*4095) 
          else:  #If not on ignore end_stops mode or not an endeffector one
             field = 0x3
             if values[i]<0:
                field = 0x4
             value = int(abs(values[i])*4095)
          if(i==6):
            if toggle_finger == True:
              #field = 0x7
              toggle_finger = False #Finger toggle debouncer 
              if prev_finger == True:
                  GPIO.output(31,GPIO.LOW)
                  rospy.loginfo("Off")
                  prev_finger = False
              else:
                  GPIO.output(31,GPIO.HIGH)
                  prev_finger = True
                  rospy.loginfo("On")
          complete_id = (ids[i] << 4)+field #Embedding ID's with chosen directional PWM command
          if value<10:
            value = 0  #Getting rid of off centre
          bit1 = value>>8&0xFF
          bit2 = value&0xFF
          #rospy.loginfo(value) 
          msg = can.Message(arbitration_id=complete_id, data=[bit1, bit2], extended_id = False)
          bus.send(msg)
        if finger_debouncer<15:
          finger_debouncer+=1
      time.sleep(0.1)

if __name__ == '__main__':
    listener()
