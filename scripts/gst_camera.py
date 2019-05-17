#!/usr/bin/env python

import gi, os, sys, subprocess, StringIO, requests
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
numFeedTypes = 9



# Create an enum of different feed types
# Enum Name = Index
class FeedType (Enum):
	FT_Stereo_Dual			= 0
	FT_Stereo_Single 		= 1
	FT_Telescopic		 		= 2
	FT_FoscamBlack			= 3
	FT_FoscamWhite			= 4
	FT_Arm_1			   		= 5
	FT_Arm_2			    	= 6
	FT_Stereo_Dual_2		= 7
	FT_Stereo_Single_2 	= 8
	
	
# Create an enum for different quality types
# Enum Name = Input
class QualityType (Enum):
  QT_High     = 0
  QT_Medium   = 1
  QT_Low      = 2


# Returns the factor for multiplying the quality for width and height
def QualityFactor (qf):
  if qf == QualityType.QT_Medium:
    return 0.5
  elif qf == QualityType.QT_Low:
    return 0.25
  else:
    return 1
  



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
# Camera states store the whether each camera is streaming or not
pipelines = [None] * numFeedTypes
buses = [None] * numFeedTypes
cameraStates = [False] * numFeedTypes




# DEFAULT VARIABLES ARE SET TO STEREOSCOPIC CAMS
# Image Format Variables
width = 720
height = 500
frame_rate = 30
img_format = "I420"
cur_qualityType = QualityType.QT_High

# Networking Variables
ip_end = 8
port = '5000'

# Camera Identification Variables
cur_feedType = FeedType.FT_Stereo_Dual
video_IDs = [1,2]
device_name = "Stereo Vision 2"
isUSB = True
cam_index = 0

# End IP of foscam for changing foscam rotation
fos_end_ip = 53




# Returns the GST pipeline with updated variable values for Stereo Cam
def gst_pipeline_stereo(_idx):
  qual = QualityFactor(cur_qualityType)
  return "v4l2src device=/dev/video{} ! videoscale ! video/x-raw, width={}, height={}, framerate={}/1, format={} ! compositor name=comp sink_1::xpos={} ! jpegenc ! rtpjpegpay ! udpsink host=192.168.1.{} port={} v4l2src device=/dev/video{} ! videoscale ! video/x-raw, width={}, height={}, framerate={}/1, format={} ! comp.".format(video_IDs[_idx], int(width * qual), int(height * qual), frame_rate, img_format, width, ip_end, port, video_IDs[_idx + 1], width * qual, height * qual, frame_rate, img_format)




# Returns the GST pipeline with updated variable values for single USB Cameras
def gst_pipeline_single(_idx):
  qual = QualityFactor(cur_qualityType)
  return "v4l2src device=/dev/video{} ! videoscale ! video/x-raw, width={}, height={}, framerate={}/1, format={} ! jpegenc ! rtpjpegpay ! udpsink host=192.168.1.{} port={}".format(video_IDs[_idx], int(width * qual), int(height * qual), frame_rate, img_format, ip_end, port)




# Returns the GST pipeline with updated variable values for Foscam (with IP)
def gst_pipeline_foscam(_foscamID):
  qual = QualityFactor(cur_qualityType)
  return("rtspsrc location=rtsp://nova:rovanova@192.168.1.{}:88/videoMain ! decodebin ! videoscale ! video/x-raw, width={}, height={} ! jpegenc ! rtpjpegpay ! udpsink host=192.168.1.{} port={} sync=false".format(_foscamID, int(width * qual), int(height * qual), ip_end, port))




# Get port value, for having multiple streams
# For example, one stream of stereoscopic cams could be port 5000, and another stream could be 5001.
# Only the last digit will be changed, so use [0] for the first stream and [1] for the second.
#port_input = str(raw_input('Enter the final digit of the port: '))
#port = port[:3] + port_input[0]










###################################
######### ACTUAL PROGRAM ##########
###################################

isRunning = True

# While program is running, watch for commands
while isRunning:
  
  # Add header information:
  os.system('clear') # Clear window
  print("Running GStreamer camera stream window. Type 'help' or 'h' for more information.")
  
  # Get current streaming cameras
  camStreams = 'None'
  for idx, val in enumerate(cameraStates):
    if val == True:
      camStreams += ', {}'.format(idx)
      
  # If there are cameras streaming
  if camStreams != 'None':
    camStreams = camStreams[6:]
  
  print("Current Streaming Camera Indexes: [{}]".format(camStreams))
  
  print("Quality Settings: {}\nIPv4 Address: 192.168.1.{}".format(cur_qualityType, ip_end))


	# Get user command
  command = str(raw_input('\n' +
  '*******************************************\n' +
  "Enter a new command ('h' to get help): ")).lower()
	
	
	
	
  # Help command
  if command == 'h' or command == 'help':
    print(
  '\n*******************************************' +
  '\n    Program Command List    ' +
  '\n*******************************************' +
  "\nExit Program:\t'e', 'exit'" +
  "\nHelp:\t\t'h', 'help'" +
  "\nInformation:\t'.', 'info'" +
  "\nChange IP:\t'i', 'ip'" +
  "\nChange Quality:\t'q', 'quality'" +
  "\nStart Feed:\t's', 'start'" +
  "\nStop Feed:\t'x', 'stop'" +
  "\nAdjust Foscam:\t'f', 'foscam'" +
  '\n*******************************************')
  
  
  # Exit command
  if command == 'e' or command == 'exit':
    isRunning = False
    print('\nClosing Program. \n')
    
    
    
    
  # Information command
  if command == '.' or command == 'info':
    print('\nThis script is for setting up camera feeds across a network. The default IP is 192.168.1.X, where X can be defined using the IP menu. For each camera that is streamed, the stream can also be closed and the quality of the feed can be adjusted. Please note, when the quality or IP ending value is adjusted, the streams that are running will not automatically update. To update, close and start the stream again once the settings have been adjusted.')

    
    
    
  # IP command
  if command == 'i' or command == 'ip':
    try:
  		ip_input = int(raw_input(
  		'\n*******************************************\n' +
  		'Current IP: 192.168.1.{}\n'.format(ip_end) +
  		'Enter a new IP end value: '
  		))
  		
  		ip_end = ip_input
    except:
      print('Invalid number entry.')
  		
 	  #print break
    print('*******************************************\n')



      
  # Quality command
  if command == 'q' or command == 'quality':
  
    # Get user input for quality type:
    qual_input = str(raw_input(
      '\n*******************************************\n' +
	    'Press (H) for High\n' +
	    'Press (M) for Medium\n' +
	    'Press (L) for Low\n' +
	    '\t: ')).lower()
	    
	  # Print break
    print('****************************\n')
    
    # Change quality type depending on user input
    if qual_input == 'h':
      # High Quality
      cur_qualityType = QualityType.QT_High
    elif qual_input == 'm':
      # Medium Quality
      cur_qualityType = QualityType.QT_Medium
    elif qual_input == 'l':
      # Low Quality
      cur_qualityType = QualityType.QT_Low
    else:
      # Invalid Entry
      print('Invalid Quality Settings Entered.')
      continue

    print('Quality Settings Adjusted to {}'.format(cur_qualityType))



##################################
######### CHANGING FEED ##########
##################################

  # Start / Stop Feed command
  if command in ('s',' start', 'x', 'stop'):
    # Get user input for camera feed type:
    feed_input = str(raw_input(
      '\n*******************************************\n' +
	    'Press (D) for Dual Stereo Cam\n' +
	    'Press (S) for Single Stereo Cam\n' +
	    'Press (T) for Telescopic Cam\n' +
	    'Press (B) for Black Foscam\n' +
	    'Press (W) for White Foscam\n' +
	    'Press (1) for Arm Cam 1\n' +
	    'Press (2) for Arm Cam 2\n' +
	    'Press (3) for Arm Dual Stereo Cam\n' +
	    'Press (4) for Arm Single Stereo Cam\n' +
	    '\t: ')).lower()
	    
	  # Print break
    print('*******************************************')

    # Change the feed type depending on user input
    if feed_input == 't':
	    # Change to Telescopic Camera feed
	    cur_feedType = FeedType.FT_Telescopic
	    device_name = 'HD USB Camera'
	    width = 640
	    height = 480
	    port = '5001'
	    isUSB = True
	    cam_index = 2

    elif feed_input == '1':
	    # Change to Arm Camera 1 feed
	    cur_feedType = FeedType.FT_Arm_1
	    device_name = 'HD USB Camera'
	    width = 640
	    height = 480
	    port = '5004'
	    isUSB = True
	    cam_index = 5

    elif feed_input == '2':
	    # Change to Arm Camera 2 feed
	    cur_feedType = FeedType.FT_Arm_2
	    device_name = 'HD USB Camera'
	    width = 640
	    height = 480
	    port = '5005'
	    isUSB = True
	    cam_index = 6

    elif feed_input == 'b':
	    # Change to Black Foscam feed
	    cur_feedType = FeedType.FT_FoscamBlack
	    device_name = 'Black Foscam'
	    width = 720
	    height = 540
	    port = '5002'
	    isUSB = False
	    cam_index = 3

    elif feed_input == 'w':
	    # Change to White Foscam feed
	    cur_feedType = FeedType.FT_FoscamWhite
	    device_name = 'White Foscam'
	    width = 720
	    height = 540
	    port = '5003'
	    isUSB = False
	    cam_index = 4

    elif feed_input == 's':
	    # Change to Single Stereo feed
	    cur_feedType = FeedType.FT_Stereo_Single
	    device_name = "Stereo Vision 2"
	    width = 720
	    height = 500
	    port = '5000'
	    isUSB = True
	    cam_index = 1
	    
    elif feed_input == 'd':
		  # Change to Dual Stereo feed
      cur_feedType = FeedType.FT_Stereo_Dual
      device_name = "Stereo Vision 2"
      width = 720
      height = 500
      port = '5000'
      isUSB = True
      cam_index = 0
      
    elif feed_input == '4':
	    # Change to Arm Single Stereo feed
	    cur_feedType = FeedType.FT_Stereo_Single_2
	    device_name = "Stereo Vision 2"
	    width = 720
	    height = 500
	    port = '5006'
	    isUSB = True
	    cam_index = 8
	    
    elif feed_input == '3':
		  # Change to Arm Dual Stereo feed
      cur_feedType = FeedType.FT_Stereo_Dual_2
      device_name = "Stereo Vision 2"
      width = 720
      height = 500
      port = '5006'
      isUSB = True
      cam_index = 7
      
    else:
      # No value pressed
      continue




  # Start Feed command
  if command in ('s',' start'):

    # Search /dev/ for video devices connected to rover
    bash_cmd = "ls /dev/ | grep video"
    output = subprocess.check_output(['bash','-c', bash_cmd])

    # Create list of video devices (ie video1)
    devs = []  
    s = StringIO.StringIO(output)
    for line in s:
	    devs.append(line.strip("\n"))

     # Create list of video device names (ie HD USB Camera)
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
      if cur_feedType == FeedType.FT_Stereo_Dual:
        gstCode = gst_pipeline_stereo(0)
			elif cur_feedType == FeedType.FT_Stereo_Single:
        gstCode = gst_pipeline_single(0)
			elif cur_feedType == FeedType.FT_Telescopic:
			  gstCode = gst_pipeline_single(0)
			elif cur_feedType == FeedType.FT_FoscamBlack:
			  gstCode = gst_pipeline_foscam(53)
			elif cur_feedType == FeedType.FT_FoscamWhite:
			  gstCode = gst_pipeline_foscam(52)
			elif cur_feedType == FeedType.FT_Arm_1:
			  gstCode = gst_pipeline_single(1)
			elif cur_feedType == FeedType.FT_Arm_2:
				gstCode = gst_pipeline_single(2)
			elif cur_feedType == FeedType.FT_Stereo_Dual_2:
				gstCode = gst_pipeline_stereo(2)
			elif cur_feedType == FeedType.FT_Stereo_Single_2:
				gstCode = gst_pipeline_single(2)
		
		
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
      
      # Update the camera state variable
      cameraStates[cam_index] = True
		

    # If camera not plugged in, output error message
    else:
	    print("Camera source '{}' not found. Please check to see if it is correctly plugged in.".format(device_name))
	    
	    
	    
	    
	    
	    
	    
  # Stop Feed command
  if command in ('x', 'stop'):
    if pipelines[cam_index] != None:
      pipelines[cam_index].set_state(Gst.State.NULL)
      print('Stopping stream from {} at IP = 192.168.1.{}:{}\n'.format(device_name, ip_end, port))
      cameraStates[cam_index] = False
    else:
      print('No stream playing at this feed.')
	
	
	
	##################################
	######### ADJUST FOSCAM ##########
	##################################
	
	# Foscam adjustments
  if command in ('f', 'foscam'):

		# Run in a loop
		isFosMenu = True
	
		while isFosMenu:
			# Get user input for foscam feed type:
		  fos_input = str(raw_input(
		    '\n*******************************************\n' +
			  'Press (A) for Rotate Left\n' +
			  'Press (D) for Rotate Right\n' +
			  'Press (W) for Rotate Up\n' +
			  'Press (S) for Rotate Down\n' +
			  'Press (X) for Stop Rotation\n' +
			  'Press (I) for Zoom In\n' +
			  'Press (O) for Zoom Out\n' +
			  'Press (1) for Black Foscam\n' +
			  'Press (2) for White Foscam\n' +
			  'Press (C) to Cancel\n' +
			  '\t: ')).lower()
			  
			# Print break
		  print('*******************************************')
		  
		  
		  fosCommand = ""
		  
		  # Rotate Left
		  if fos_input == 'a':
		  	fosCommand = 'ptzMoveLeft'
		  
		  # Rotate Right
		  if fos_input == 'd':
		  	fosCommand = 'ptzMoveRight'
		  
		  # Rotate Up
		  if fos_input == 'w':
		  	fosCommand = 'ptzMoveUp'
		  	
		  # Rotate Down
		  if fos_input == 's':
		  	fosCommand = 'ptzMoveDown'
		  	
		  # Stop Rotation
		  if fos_input == 'x':
		  	fosCommand = 'ptzStopRun'
		  	
		  # Zoom In
		  if fos_input == 'i':
		  	fosCommand = 'zoomIn'
		  
		  # Zoom Out
		  if fos_input == 'o':
		  	fosCommand = 'zoomOut'
		  	
		  # Change to Black Foscam
		  if fos_input == '1':
		  	fos_end_ip = 53
		  	
		  # Change to White Foscam
		  if fos_input == '2':
		  	fos_end_ip = 52		  	
		  
		  # Run the command
		  if fosCommand != "":
		  
		  	# Send the request
		  	req = "http://192.168.1.{}:88/cgi-bin/CGIProxy.fcgi?cmd={}&usr=nova&pwd=rovanova".format(fos_end_ip, fosCommand)
		  	resp = requests.get(req)
		  	
		  	# Print the output
		  	print(resp.text)
		  
		  
		  # Cancel the Foscam Menu
		  if fos_input == 'c':
		  	isFosMenu = False
	
	
	
	
	
	# Waits for enter to be pressed
  if isRunning:
    raw_input('\nPress Enter to continue...\n')
    
    
    
    
    
  
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
