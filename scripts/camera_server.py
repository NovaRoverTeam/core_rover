#!/usr/bin/env python

import rospy

import gi
gi.require_version('Gst', '1.0')
gi.require_version('GstBase', '1.0')
gi.require_version('Gtk', '3.0')
from gi.repository import GObject, Gst, GstBase, Gtk, GObject
import signal
import subprocess
import os
import StringIO
import sys
from enum import Enum

from nova_common.srv import * # Import custom msg
from nova_common.msg import *

# Enumerator for gstreamer camera status
class StreamStatus(Enum):
  GST_STATE_FAILURE             = 0
  GST_STATE_SUCCESS             = 1
  GST_STATE_ASYNC               = 2
  GST_STATE_CHANGE_NO_PREROLL   = 3  
  
loop_hz = 1  # Rate of main loop, in Hz
des_con = [] # Global vars, set whether we want a stream on or not
cur_con = []

# Class handling ROS behaviour of rover_sync node
class CameraServer:

  #--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
  # __init__():
  #    Initialise class.
  #--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--    
  def __init__(self):

    Gst.init(None)
    rospy.init_node('camera_server')

    _ = rospy.Service('/core_rover/connect_stream', 
      ToggleStream, self.handleConnectStream)      

    rospy.Subscriber('/base_station/gimbal_cmd', GimbalCmd, 
      self.gimbalCb, queue_size=10)

    self.pub = rospy.Publisher('/core_rover/camera_status', CameraStatus, queue_size=1)

    self.setupSig()
    self.initParams()

    self.cur_con = [False]*self.n_devs     # Current streams  

    # TODO Get destinations by service call or radio state machine
    # Important for choosing backup radio
    self.dests = [self.def_dest]*self.n_devs 
   
    self.updateDevList() # Get available devices  
    self.streams = [None]*self.n_devs # Initialise stream array
      
    # Initialise default streams
    for i in range(self.n_devs): 
      if self.des_con[i] is True:
        self.startStream(i)
        
  #--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
  # initParams():
  #    Grab initial parameters from ROS param server.
  #--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--   
  def initParams(self):
  
    self.port     = rospy.get_param("~port")      # Base port num for UDP
    self.n_devs   = rospy.get_param("~n_cams")    # Number of cameras
    self.def_dest = rospy.get_param("~def_dest") # Default host IP addr
    
    self.ids     = self.getCamParam("id")      # List of devs/IPs
    self.usbs    = self.getCamParam("usb")     # Whether USB or IP
    self.names   = self.getCamParam("name")    # Camera names
    self.des_con = self.getCamParam("default") # Camera names

  #--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
  # setupSig():
  #    Set up custom sigint handler.
  #--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--   
  def setupSig(self):

    def signal_handler(sig, frame):
      rospy.loginfo("Forcing gstreamer shutdown.")
      rospy.signal_shutdown("SIGINT") # Shut down ROS   
      sys.exit(1)      
          
    signal.signal(signal.SIGINT, signal_handler) # Register sigint handler

  #--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
  # updateDevList():
  #
  #    Update list of available video devices, in the order specified in
  #    the launch file, allowing identification of each camera.
  #--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--
  def updateDevList(self):

    try:
      # Search /dev/ for video devices connected to rover
      bash_cmd = "ls /dev/ | grep video"
      output = subprocess.check_output(['bash','-c', bash_cmd])
      
      devs = [] # Create list of video device names  
      s = StringIO.StringIO(output)
      for line in s:
        devs.append(line.strip("\n"))
        
      ids = [] # Create list of video device ids  
      for i in range(len(devs)):  
        bash_cmd  = "cat /sys/class/video4linux/" + devs[i] + "/name "
        #bash_cmd += "| grep -oP '\(\K[^\)]+'" # Comment for intgr. cam
        output = subprocess.check_output(['bash','-c', bash_cmd])
        ids.append(output.strip("\n"))
        
      dev_ordered = [None]*self.n_devs # Order the names of available devs
        
      for i in range(self.n_devs): #
        try:
          dev_i = ids.index(self.ids[i]) # Get index of where device is
          dev_str = "/dev/" + devs[dev_i]  
          dev_ordered[i] = dev_str
        except ValueError: # Device with desired id is not available
          pass      
          
      rospy.loginfo(dev_ordered)
      self.dev_list = dev_ordered
      
    except subprocess.CalledProcessError: # unable to grep dev names
      self.dev_list = [None]*self.n_devs
      
  #--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
  # createWebcamStream():
  #
  #    Creates a new stream to the given host and port, using a camera 
  #    found at device path "dev".
  #--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--
  def createWebcamStream(self, i):

    description = ("v4l2src device=" + self.dev_list[i]
      + " ! image/jpeg,width=640, height=480,framerate=30/1 ! rtpjpegpay"
      + " ! udpsink host=" + self.dests[i] + " port=" + str(self.port + i))
    
    return Gst.parse_launch(description)

  #--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
  # createIPCamStream():
  #
  #    Creates a new stream to the given host and port, using a camera 
  #    found at IP address for camera i.
  #--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--
  def createIPCamStream(self, i):
	
    description = ("rtspsrc location=rtsp://nova:rovanova@" + self.ids[i] 
      + ":88/videoMain ! decodebin ! jpegenc"
      + " ! rtpjpegpay ! udpsink host=" + self.dests[i] 
      + " port=" + str(self.port + i) + " sync=false")

    return Gst.parse_launch(description) 

  #--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
  # createStereoCamStream():
  #
  #    Creates a new stream to the given host and port, using the stereo 
  #    cameras found at device path "dev".
  #--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--
  def createStereoCamStream(self, i):
	
	# Check if both stereo cameras exist
	if self.dev_list[i+1] is not None:
	    description = ("v4l2src device=" + self.dev_list[i]
	      + " ! videoscale ! video/x-raw, width=720, height=500, framerate=30/1,"
	      + " format=I420 ! compositor name=comp sink_1::xpos=720 ! jpegenc ! "
	      + " rtpjpegpay" + " ! udpsink host=" + self.dests[i] + " port=" + str(self.port + i)
	      + " v4l2src device=" + self.dev_list[i+1] + " ! videoscale ! video/x-raw, width=720,"
	      + " height=500, framerate=30/1, format=I420 ! comp.")
	    
    	    return Gst.parse_launch(description)
	# If only one works, use standard webcam stream
	else:
		return createWebcamStream(i)
    
  #--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
  # getCamParam():
  #
  #    Convenience function for grabbing multiple launch file params.
  #--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..-- 
  def getCamParam(self, prm):

    params = []
    for i in range(self.n_devs):
      params.append(rospy.get_param("~cam_" + str(i) + "/" + prm))
    return params

  #--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
  # startStream():
  #
  #  Grabs the correct device name for the camera, creates  
  #--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..-- 
  def startStream(self, i):
    
    if self.usbs[i] is True: # If USB device

      self.updateDevList() # Recompute device list
      if self.dev_list[i] is not None:
        self.streams[i] = self.createStereoCamStream(i)

    else: # If IP device
      self.streams[i] = self.createIPCamStream(i)

    self.streams[i].set_state(Gst.State.PLAYING)

  #--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
  # handleConnectStream():
  #
  #  Service server handler for connecting a camera stream.
  #--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..-- 
  def handleConnectStream(self, req):
      
      self.des_con[req.cam_id] = req.on
        
      return ToggleStreamResponse(True, "yep switched to " 
        + str(self.des_con[req.cam_id]))   

  #--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
  # refreshCamera():
  #
  #  Checks the camera periodically to see if we need to set it up.
  #--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..-- 
  def refreshCamera(self, i): 
                 
    if self.des_con[i] is True: # If requested to stream this cam
    
      if self.streams[i] is None: # If no stream, create one
        self.startStream(i)          
      
      else: # If stream already exists, check status         
        current, _, _ = self.streams[i].get_state(Gst.CLOCK_TIME_NONE)
        
        # If stream has failed
        if StreamStatus(current) is not StreamStatus.GST_STATE_SUCCESS:          
          if self.cur_con[i] is True: 
            rospy.loginfo("Camera " + str(i) + " (" + self.names[i] 
                          + ") disconnected. Attempting to reconnect.") 
                          
          self.streams[i].set_state (Gst.State.NULL); # Stop stream                         
          self.startStream(i) # Restart stream
          
          self.cur_con[i] = False    # Record that the stream is inactive
          
        # If stream fine, give status message
        elif self.cur_con[i] is False:         
          
          rospy.loginfo("Camera " + str(i) + " (" + self.names[i] 
                        + ") successfully connected.") 
          self.cur_con[i] = True     # Record that the stream is now active
    
    elif self.cur_con[i] is True: # If this camera is streaming but should stop
    
      self.streams[i].set_state (Gst.State.NULL); # Stop stream
      self.streams[i] = None                      # Remove stream
      self.cur_con[i] = False                     # Record disconnection

      rospy.loginfo("Camera " + str(i) + " (" + self.names[i] 
                        + ") disconnected.") 
      
    self.pubCamStat(i) # Publish camera status

  #--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
  # gimbalCb():
  #
  #  Respond to a gimbal command request.
  #--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..-- 
  def gimbalCb(self, msg):  

    try:
      bash_cmd = ("curl -s http://" + self.ids[msg.cam_id] 
        + ":88/cgi-bin/CGIProxy.fcgi?cmd=" + msg.cgi_cmd 
        + "\&usr=nova\&pwd=")
      subprocess.check_output(['bash','-c', bash_cmd])

    except subprocess.CalledProcessError:
      pass

  #--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
  # pubCamStat():
  #
  #  Publish a camera status message for a camera.
  #--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..-- 
  def pubCamStat(self, i):  

    msg = CameraStatus() # Populate camera status message with info
    msg.cam_id = i

    # Is the camera physically plugged in? Report with camera status
    msg.plugged = (self.dev_list[i] is not None)  
      
    # Is the camera streaming? Report with camera status  
    msg.streaming = self.cur_con[i]
    
    self.pub.publish(msg)  

#--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
# main():
#
#    Main function.
#--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..-- 
def main():

  camera_server = CameraServer()
  
  rate = rospy.Rate(loop_hz)  
  
   
  while not rospy.is_shutdown():  
      
    for i in range(camera_server.n_devs):    
      camera_server.refreshCamera(i)
      
    rate.sleep()
    
  
#--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
# Initialiser.
#--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..-- 
if __name__ == '__main__':
  try:
    start = main()   
    
       
  except rospy.ROSInterruptException:
    pass
