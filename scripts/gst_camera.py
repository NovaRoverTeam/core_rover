#!/usr/bin/env python

import gi, os, sys, subprocess, StringIO
from enum import Enum
from PyQt5.QtWidgets import *


# Get specific versions
gi.require_version('Gst', '1.0')
gi.require_version('GstVideo', '1.0')
gi.require_version('Gtk', '3.0')

from gi.repository import Gst, GObject, Gtk
from gi.repository import GdkX11, GstVideo


# Initialise the GStreamer library with arguments
Gst.init(sys.argv)




# Number of feed types available
numFeedTypes = 6


# Create an enum of different feed types
# Enum Name = Index
class FeedType (Enum):
	FT_Stereo	= 0
	FT_Telescopic	= 1
	FT_FoscamBlack	= 2
	FT_FoscamWhite	= 3
	FT_Arm_1	= 4
	FT_Arm_2	= 5




# Create a message pipeline function, for outputting messages to the terminal
def on_message(bus, message, loop):
	mtype = message.type
	"""
	Gstreamer Message Types and how to parse
        https://lazka.github.io/pgi-docs/Gst-1.0/flags.html#Gst.MessageType
	"""
	if mtype == Gst.MessageType.EOS:
	  print("End of stream")
        
	elif mtype == Gst.MessageType.ERROR:
		err, debug = message.parse_error()
		print(err, debug)

	elif mtype == Gst.MessageType.WARNING:
		err, debug = message.parse_warning()
		print(err, debug)       
        
	return True 



# Stores a collection of all GST pipelines and buses
# A bus stores the messages from GST and connects the terminal to command
pipelines = [None] * numFeedTypes
buses = [None] * numFeedTypes




# DEFAULT VARIABLES ARE SET TO STEREOSCOPIC CAMS
# Image Format Variables
width = 720
height = 500
frame_rate = 30
img_format = "I420"

# Networking Variables
ip_end = 8
port = '5000'

# Camera Identification Variables
cur_feedType = FeedType.FT_Stereo
video_IDs = [1,2]
device_name = "Stereo Vision 2"
isUSB = True
cam_index = 0




# Returns the GST pipeline with updated variable values for Stereo Cam
def gst_pipeline_stereo():
	return "v4l2src device=/dev/video{} ! videoscale ! video/x-raw, width={}, height={}, framerate={}/1, format={} ! compositor name=comp sink_1::xpos={} ! jpegenc ! rtpjpegpay ! udpsink host=192.168.1.{} port={} v4l2src device=/dev/video{} ! videoscale ! video/x-raw, width={}, height={}, framerate={}/1, format={} ! comp.".format(video_IDs[0], width, height, frame_rate, img_format, width, ip_end, port, video_IDs[1], width, height, frame_rate, img_format)




# Returns the GST pipeline with updated variable values for Arm Cam
def gst_pipeline_arm():
	return "v4l2src device=/dev/video{} ! videoscale ! video/x-raw, width={}, height={}, framerate={}/1, format={} ! jpegenc ! rtpjpegpay ! udpsink host=192.168.1.{} port={}".format(video_IDs[0], width, height, frame_rate, img_format, ip_end, port)




# Returns the GST pipeline with updated variable values for Foscam (with IP)
def gst_pipeline_foscam(_foscamID):
	#return("gst-launch-1.0 rtspsrc location=rtsp://nova:rovanova@192.168.1.{}:88/videoMain ! autovideosink".format(_foscamID))
	return("rtspsrc location=rtsp://nova:rovanova@192.168.1.{}:88/videoMain ! decodebin ! videoscale ! video/x-raw, width=640, height=480 ! jpegenc ! rtpjpegpay ! udpsink host=192.168.1.{} port={} sync=false".format(_foscamID, ip_end, port))




# Get port value, for having multiple streams
# For example, one stream of stereoscopic cams could be port 5000, and another stream could be 5001.
# Only the last digit will be changed, so use [0] for the first stream and [1] for the second.
#port_input = str(raw_input('Enter the final digit of the port: '))
#port = port[:3] + port_input[0]










###################################
######### ACTUAL PROGRAM ##########
###################################

isRunning = True

print("Running GStreamer camera stream window. Type 'help' or 'h' for more information.")

# While program is running, watch for commands
while isRunning:
	# Get user command
  command = str(raw_input('\n' +
  '****************************\n' +
  "Enter a new command ('h' to get help): ")).lower()
	
  # Help command
  if command == 'h' or command == 'help':
    print(
  '\n****************************' +
  '\n    Program Command List    ' +
  '\n****************************' +
  "\nExit Program:\t'e', 'exit'" +
  "\nHelp:\t\t'h', 'help'" +
  "\nStart Feed:\t's', 'start'" +
  "\nStop Feed:\t'x', 'stop'" +
  '\n')
  
  # Exit command
  if command == 'e' or command == 'exit':
    isRunning = False
    print('Closing Program.')

  # Start / Stop Feed command
  if command in ('s',' start', 'x', 'stop'):
    # Get user input for camera feed type:
    feed_input = str(raw_input(
      '****************************\n' +
	    'Press (S) for Stereo Cam\n' +
	    'Press (T) for Telescopic Cam\n' +
	    'Press (B) for Black Foscam\n' +
	    'Press (W) for Black Foscam\n' +
	    'Press (1) for Arm Cam 1\n' +
	    'Press (2) for Arm Cam 2\n' +
	    '\t: ')).lower()
	    
	  # Print break
    print('****************************\n')

    # Change the feed type depending on user input
    if feed_input == 't':
	    # Change to Telescopic Camera feed
	    cur_feedType = FeedType.FT_Telescopic
	    device_name = 'HD USB Camera'
	    width = 640
	    height = 480
	    port = '5001'
	    isUSB = True
	    cam_index = 1

    elif feed_input == '1':
	    # Change to Arm Camera 1 feed
	    cur_feedType = FeedType.FT_Arm_1
	    device_name = ''
	    width = 640
	    height = 480
	    port = '5004'
	    isUSB = True
	    cam_index = 4

    elif feed_input == '2':
	    # Change to Arm Camera 2 feed
	    cur_feedType = FeedType.FT_Arm_2
	    device_name = ''
	    width = 640
	    height = 480
	    port = '5005'
	    isUSB = True
	    cam_index = 5

    elif feed_input == 'b':
	    # Change to Black Foscam feed
	    cur_feedType = FeedType.FT_FoscamBlack
	    device_name = 'Black Foscam'
	    width = 640
	    height = 480
	    port = 5002
	    isUSB = False
	    cam_index = 2

    elif feed_input == 'w':
	    # Change to White Foscam feed
	    cur_feedType = FeedType.FT_FoscamWhite
	    device_name = 'White Foscam'
	    width = 640
	    height = 480
	    port = 5003
	    isUSB = False
	    cam_index = 3
	    
    else:
      # Change to Stereo feed
      cur_feedType = FeedType.FT_Stereo
      device_name = "Stereo Vision 2"
      width = 720
      height = 500
      port = 5000
      isUSB = True
      cam_index = 0

  # Start Feed command
  if command in ('s',' start'):



    # Search /dev/ for video devices connected to rover
    bash_cmd = "ls /dev/ | grep video"
    output = subprocess.check_output(['bash','-c', bash_cmd])

    # Create list of video device names
    devs = []  
    s = StringIO.StringIO(output)
    for line in s:
	    devs.append(line.strip("\n"))

     # Create list of video device ids
    ids = []  
    for i in range(len(devs)):  
	    bash_cmd  = "cat /sys/class/video4linux/" + devs[i] + "/name "
	    output = subprocess.check_output(['bash','-c', bash_cmd])
	    ids.append(output.strip("\n"))



    # Creates the list of desired camera devices from the dev list
    devices = []
    for idx, val in enumerate(ids):
	    if val == device_name:
		    devices.append(devs[idx].replace('video',''))
    print('{} Device Index(es): {}\n'.format(device_name, devices))

    # Update camera IDs
    video_IDs = devices

    print('Attempting to stream data from {} at IP = 192.168.1.{}:{}\n'.format(device_name, ip_end, port))




    # Check for error in camera detection
    if len(devices) > 0 or not isUSB:
      # Get the appropriate GST pipeline command
      if cur_feedType == FeedType.FT_Stereo:
        gstCode = gst_pipeline_stereo()
      elif cur_feedType == FeedType.FT_Telescopic:
        gstCode = gst_pipeline_arm()
      elif cur_feedType == FeedType.FT_FoscamBlack:
        gstCode = gst_pipeline_foscam(53)
      elif cur_feedType == FeedType.FT_FoscamWhite:
        gstCode = gst_pipeline_foscam(52)
      elif cur_feedType == FeedType.FT_Arm_1:
        gstCode = gst_pipeline_arm()
      elif cur_feedType == FeedType.FT_Arm_2:
        gstCode = gst_pipeline_arm()
		
		
	    # Create pipeline
      pipeline = Gst.parse_launch(gstCode)
      bus = pipeline.get_bus()  

	    # Allow bus to emit messages to main thread
      bus.add_signal_watch()

	    # Add handler to specific signal
      bus.connect("message", on_message, None)
      
      # Add a new pipeline to the list
      pipelines[cam_index] = pipeline
      buses[cam_index] = bus
      
      
      # Start the pipeline in the playing state
      pipelines[cam_index].set_state(Gst.State.PLAYING)
		

    # If camera not plugged in, output error message
    else:
	    print("Camera source '{}' not found. Please check to see if it is correctly plugged in.".format(device_name))
	    
  # Stop Feed command
  if command in ('x', 'stop'):
    if pipelines[cam_index] != None:
      pipelines[cam_index].set_state(Gst.State.NULL)
      print('Stopping stream from {} at IP = 192.168.1.{}:{}\n'.format(device_name, ip_end, port))
    else:
      print('No stream playing at this feed.')
	
