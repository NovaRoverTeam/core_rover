#############################################
######                                 ######
###### GStreamer and Camera Controller ######
######            (GaCC)               ######
######                                 ######
#############################################

# This script is used to access cameras on board
#   the rover, and is able to start and stop camera
#   streaming. It is created so that it can be
#   accessed by both the terminal command line, and
#   a Web UI service.




################################
###### FUNCTION DIRECTORY ######
################################

# LoadCameraData ()
#   --> Reads the camera data csv file and imports the camera settings
#       that can be used later.
# GetCameraData (selectField)
#   --> Returns all camera data from the csv file, unless the field parameter
#       is given, then only that field is returned as a list
# GetIPAddress ()
#   --> Returns the current IP address as a string
# SetIPAddress (newIP)
#   --> Takes in a full address (x.x.x.x) or just the last number (x) 
#       and changes the IP to send the camera streams to.
# GetQualityType ()
#   --> Returns the current Quality type as a string
# SetQualityType (newQual) 
#   --> Takes in either 'h', 'm' or 'l' and updates the quality of the cameras
# IsCameraStreaming ()
#   --> Returns whether or not the current camera is streaming



###############################
###### IMPORTED PACKAGES ######
###############################

#!/usr/bin/env python

import gi, os, sys, subprocess, StringIO, requests, time
from enum import Enum
from PyQt5.QtWidgets import *

# Get specific versions
gi.require_version('Gst', '1.0')
gi.require_version('GstVideo', '1.0')
gi.require_version('Gtk', '3.0')

# Import the GStreamer objects
from gi.repository import Gst, GObject, Gtk
from gi.repository import GdkX11, GstVideo

# Initialise the GStreamer library with arguments
Gst.init(sys.argv)



#######################
###### VARIABLES ######
#######################

# Quality type of stream.
# For high, the default camera size is used
# For medium, half the camera dimensions are used
# For low, a quarter of the camera dimensions are used
class QualityType (Enum):
  QT_High   = 0
  QT_Medium = 1
  QT_Low    = 2


# Current variables
cur_qualityType = QualityType.QT_High
cur_IPAddress = '127.0.0.1'
cameras = {} # Dictionary of camera information (by IDs), with dictionaries as data
cameraPathIndex = {} # Dictionary to find camera based on a path ID
cur_streamingType = False # If True, TCP. If False, UDP

# Refresh Variables
lastRefreshTime_P = 0 # The last timestamp that the system scanned for cameras (PATH DEPENDANT)
lastRefreshTime_N = 0 # The last timestamp that the system scanned for cameras (NAME DEPENDANT)
refreshRate = 1.0 # The time it takes before the cameras need to be scanned again

# GStreamer variables
pipelines = {}
buses = {}
cameraStates = {}

# Temp Camera Variables
# DO NOT TOUCH THESE OR CHANGE THE NAMES
# They serve simply as storing temperary data while the program does not refresh
# Instead of checking camera data every row, this allows for the same data to be
#   relayed within a specific time. Assuming, it does not change within that time
camsConnected = [] # List of connected cameras by path
cams_connected = [] # List of connected cameras by name




#######################
###### GSTREAMER ######
#######################

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


# Return the GStreamer code for cameras
def GetGSTCodeSingle (path, width, height, frameRate, imgFormat, port):
  # Get quality factor
  if cur_qualityType == QualityType.QT_High:
    qual = 1.0
  elif cur_qualityType == QualityType.QT_Medium:
    qual = 0.5
  elif cur_qualityType == QualityType.QT_Low:
    qual = 0.25
  
  if cur_streamingType:
    encoding = 'theoraenc ! oggmux ! tcpserversink'
  else:
    encoding = 'decodebin ! jpegenc ! rtpjpegpay ! udpsink'
  
  # Return code
  return "v4l2src device={} ! videoscale ! video/x-raw, width={}, height={}, framerate={}/1, format={} ! {} host={} port={}".format(path, int(int(width) * qual), int(int(height) * qual), frameRate, imgFormat, encoding, cur_IPAddress, port)






#######################
###### FUNCTIONS ######
#######################

# Get the quality type
def GetQualityType ():
  if cur_qualityType == QualityType.QT_High:
    return 'High'
  elif cur_qualityType == QualityType.QT_Medium:
    return 'Medium'
  elif cur_qualityType == QualityType.QT_Low:
    return 'Low'
  else:
    return 'Invalid Quality'

# Change the quality type
def SetQualityType (newQual):
  global cur_qualityType

  newQual = newQual.lower()
  
  # Update the quality type based on the input
  if (newQual == 'h' or newQual == 'high'):
    cur_qualityType = QualityType.QT_High
  elif (newQual == 'm' or newQual == 'medium'):
    cur_qualityType = QualityType.QT_Medium
  elif (newQual == 'l' or newQual == 'low'):
    cur_qualityType = QualityType.QT_Low
    
  # If invalid input, don't change the quality type and return an error
  else:
    return None
  
  # Return the new quality type, as changed by the input
  return cur_qualityType




# Get the IP Address
def GetIPAddress ():
  return cur_IPAddress

# Change the IP Address
def SetIPAddress (newIP):
  global cur_IPAddress
  
  # Check if the IP is a full address
  mask = newIP.split('.')
  # If it is just the end of the IP
  if len(mask) == 1:
    # Check for a valid IP end
    try:
      if int(mask[0]) >= 0 and int(mask[0]) <= 255:
        oldMask = cur_IPAddress.split('.')
        cur_IPAddress = oldMask[0] + '.' + oldMask[1] + '.' + oldMask[2] + '.' + mask[0]
      else:
        return None
    except:
      return None
  
  # In case the input is the whole IP address
  elif len(mask) == 4:
    cur_IPAddress = newIP
    
  # If invalid input, don't change the IP address
  else:
    return None
  
  # Return the new IP address, as changed by the input
  return cur_IPAddress



# Get whether or not it is streaming UDP or TCP
def GetStreamingType ():
  if cur_streamingType:
    return 'TCP'
  else:
    return 'UDP'

# Switches to the alternative streaming type
def ToggleStreamingType ():
  global cur_streamingType
  cur_streamingType = not cur_streamingType



# Gets the list of connected cameras by the USB hub path it is connected to
def GetCamerasByPath ():
  # Make sure camera data is loaded
  LoadCameraData()
  
  global camsConnected, lastRefreshTime_P

  # Check if time allows for refreshing the camera data
  # This ensures that the same code is not repeated without reason
  if time.time() > lastRefreshTime_P + refreshRate:
    # Use the command to find a list of connected cameras
    camsConnected = []
    
    # Only if cameras are connected.
    try:
      bash_cmd = "ls /dev/v4l/by-path | grep platform-3530000.xhci-usb-"
      output = subprocess.check_output(['bash','-c', bash_cmd])
      
      # Get a list of the cameras by taking each line as a separate device
      listedCams = output[:-2].replace('platform-3530000.xhci-usb-','').replace('-video-index','').split('\n')

      # Loop through each camera
      for i in range(0, len(listedCams)):
        # Only take the middle number (eg, from 0:2.4.2:1.0 take only 2.4.2)
        split = listedCams[i].split(':')
        camIndex = split[1]
        
        # Check if camera exists
        if camIndex in cameraPathIndex.keys():
          camsConnected.append([camIndex, cameraPathIndex[camIndex]])
        # Otherwise give an error
        else:
          camsConnected.append([camIndex, 'UNKNOWN CAMERA'])
    
    # Otherwise just return no cameras
    except:
      pass
    
    # Save the new timestamp
    lastRefreshTime_P = time.time()
  
  # Return the final list
  return camsConnected




# Loads the camera information from the CSV file
def LoadCameraData ():
  global cameras
  cameras = {} # Clear data
  # Reads the camera CSV file
  with open('cameras.csv', 'r') as file:
    content = file.readlines()
    attributes = content[0].replace('\n','').split(",")
    
    # Loop through each camera line, after the header line
    if (len(content) > 1):
      for camera in content[1:]:
        # Create a dictionary for the data
        data_dict = {}
        data = camera.replace('\n','').split(",")
        for i in range(0, len(data)):
          data_dict[attributes[i]] = data[i]
        
        # Add the dictionary to the camera array
        cameras[data_dict['ID']] = data_dict
  
        # Create the path index dictionary and map it to the camera
        cameraPathIndex[data_dict['Path']] = data_dict['ID']




# Returns the camera data from the saved dictionary
def GetCameraData (selectField = ''):
  LoadCameraData () # Load the data from CSV file
  # If outputting all data
  if selectField == '':
    return cameras
  # If only selecting names of cameras
  else:
    return [cameras[cam][selectField] for cam in cameras.keys()]





# Gets a list of all connected cameras
def GetConnectedCameras ():

  global cams_connected, lastRefreshTime_N

  # Check if time allows for refreshing the camera data
  # This ensures that the same code is not repeated without reason
  if time.time() > lastRefreshTime_N + refreshRate:
  
    # Search /dev/ for video devices connected to rover
    bash_cmd = "ls /dev/ | grep video"
    output = subprocess.check_output(['bash','-c', bash_cmd])

    # Create list of video devices (ie video1)
    devs = []  
    s = StringIO.StringIO(output)
    for line in s:
      dev = line.strip("\n")
      # Remove the default camera on the Jetson
      if dev != "video0":
        devs.append(dev)

     # Create list of video device names (ie HD USB Camera)
    ids = []  
    for i in range(len(devs)):  
      bash_cmd  = "cat /sys/class/video4linux/" + devs[i] + "/name "
      output = subprocess.check_output(['bash','-c', bash_cmd])
      ids.append(output.strip("\n"))

    # Create a set of cameras with names
    cams_connected = zip(devs, ids)
    lastRefreshTime_N = time.time()
  
  return cams_connected


# Takes in a camera and checks to see if it is connected to the system
def IsCameraConnected (cameraID):
  # Get the path and camera connection variables
  pathCameras = GetCamerasByPath()
  pathCameraNames = [cam[1] for cam in pathCameras]
  connectedCameras = GetConnectedCameras()
  connectedCameraNames = [cam[1] for cam in connectedCameras]
  
  # Check if the camera is connected
  connected = False
  if cameraID in pathCameraNames: #or camera['CameraName'] in connectedCameraNames:
    connected = True
    
  return connected


# Returns whether or not a camera is currently streaming
def IsCameraStreaming (cameraID):
  # Check if the camera stream state exists
  if cameraID in cameraStates.keys():
    return cameraStates[cameraID]
  else:
    return False






# Starts a camera streaming
def StartStreaming (cameraID):

  # Check if the camera is not streaming
  if IsCameraStreaming(cameraID) == False:
    # Get the current camera dictionary
    cameraInfo = cameras[cameraID]
    
    # Get path directory
    if True:
      path = '/dev/v4l/by-path/platform-3530000.xhci-usb-0:{}:1.0-video-index0'.format(cameraInfo['Path'])
    else:
      path = '/dev/video{}'.format(0)
    
    # Get the GST code
    gstCode = GetGSTCodeSingle(path,cameraInfo['Width'],cameraInfo['Height'],cameraInfo['FrameRate'],cameraInfo['Format'],cameraInfo['Port'])
    
    # Create pipeline
    pipeline = Gst.parse_launch(gstCode)
    bus = pipeline.get_bus()  

    # Allow bus to emit messages to main thread
    bus.add_signal_watch()

    # Add handler to specific signal
    bus.connect("message", on_message, None)
    
    # Add a new pipeline to the list
    pipelines[cameraID] = pipeline
    buses[cameraID] = bus
         
    # Start the pipeline in the playing state
    pipeline.set_state(Gst.State.PLAYING)
    cameraStates[cameraID] = True
  
  # Otherwise, stop the camera stream
  else:
    StopStreaming(cameraID)




# Stops the camera streaming
def StopStreaming (cameraID):
  
  # Check to see if the camera is streaming
  if IsCameraStreaming(cameraID):
    # Stop the pipeline from streaming
    pipelines[cameraID].set_state(Gst.State.NULL)
    cameraStates[cameraID] = False


# Stops all camera streaming
def StopAllStreams ():
  # Loop through every camera that is streaming
  for camera in pipelines.keys():
    StopStreaming(camera)




# OTHER COMMANDS
# v4l2-ctl -d /dev/video0 --list-formats
# ls /dev/v4l/by-path

