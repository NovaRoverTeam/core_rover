#!/usr/bin/env python

import rospy
import pygst
pygst.require("0.10")
import gst
import pygtk
import gtk
import signal
import subprocess
import StringIO
import sys
from enum import Enum

# Enumerator for gstreamer camera status
class StreamStatus(Enum):
  GST_STATE_FAILURE             = 0
  GST_STATE_SUCCESS             = 1
  GST_STATE_ASYNC               = 2
  GST_STATE_CHANGE_NO_PREROLL   = 3  


#--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
# get_vid_devs():
#
#    Get a list of available video devices, in the order specified in
#    the launch file, allowing identification of each camera.
#--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--
def get_vid_devs(id_list):

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
      bash_cmd  = "cat /sys/class/video4linux/" + devs[i]
      bash_cmd += "/name | grep -oP '\(\K[^\)]+'"
      output = subprocess.check_output(['bash','-c', bash_cmd])
      ids.append(output.strip("\n"))
      
    n_ids = len(id_list) # Number of known devices
    dev_ordered = [None]*n_ids # Order the names of available devs
      
    for i in range(n_ids): #
      try:
        dev_i = ids.index(id_list[i]) # Get index of where device is
        dev_str = "/dev/" + devs[dev_i]  
        dev_ordered[i] = dev_str
      except ValueError: # Device with desired id is not available
        pass      
        
    print(dev_ordered)
    return dev_ordered
    
  except subprocess.CalledProcessError: # unable to grep dev names
    return [None]*len(id_list)
    


#--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
# create_stream():
#
#    Creates a new stream to the given host and port, using a camera 
#    found at device path "dev".
#--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--
def create_stream(host, port, dev):

  pipeline = gst.Pipeline("pipe") # Create a gstreamer pipeline
  
  # Add video source to pipeline
  elem_vid_src = gst.element_factory_make("v4l2src", "vid_src")
  elem_vid_src.set_property("device", dev)
  pipeline.add(elem_vid_src) 
  
  # Set video quality info and add to pipeline
  caps = gst.caps_from_string("image/jpeg,width=640, height=480,framerate=30/1")
  filter = gst.element_factory_make("capsfilter", "filter")
  filter.set_property("caps", caps)
  pipeline.add(filter)  
  
  # Add jpeg encoder and payload packer to pipeline
  elem_jpeg_enc = gst.element_factory_make("rtpjpegpay", "jpeg_enc")
  pipeline.add(elem_jpeg_enc)
  
  # Add the udp streamer to pipeline
  elem_udp = gst.element_factory_make("udpsink", "udp")
  elem_udp.set_property("host", host)
  elem_udp.set_property("port", port)
  pipeline.add(elem_udp)
  
  elem_vid_src.link(filter) # Link the elements in the pipeline together
  filter.link(elem_jpeg_enc)
  elem_jpeg_enc.link(elem_udp)
  
  return pipeline
  
  
#--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
# get_cam_param():
#
#    Convenience function for grabbing multiple launch file params.
#--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..-- 
def get_cam_param(prm, n):
  params = []
  for i in range(n):
    params.append(rospy.get_param("~cam_" + str(i) + "/" + prm))
  return params


#--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
# start_stream():
#
#  Grabs the correct device name for the camera, creates  
#--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..-- 
def start_stream(streams, hosts, port, id_list, i):
  
  dev_list = get_vid_devs(id_list) # Recompute device list
  if dev_list[i] is not None:
    streams[i] = create_stream(hosts[i], port + i, dev_list[i])
    streams[i].set_state(gst.STATE_PLAYING)


#--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
# main():
#
#    Main function.
#--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..-- 
def main():
  rospy.init_node('camera_server')
  rate = rospy.Rate(1)
  
  # TODO Implement service server for receiving camera requests
  
  # Set up custom sigint handler
  def signal_handler(sig, frame):
    try:
      gtk.main_quit()               # Shut down gstreamer
    except RuntimeError:
      rospy.loginfo("Forcing gstreamer shutdown.")
      rospy.signal_shutdown("SIGINT") # Shut down ROS   
      sys.exit(1)      
         
  signal.signal(signal.SIGINT, signal_handler) # Register sigint handler
  
  port   = rospy.get_param("~port")   # Base port number for UDP streaming
  n_devs = rospy.get_param("~n_cams") # Number of cameras
  
  id_list = get_cam_param("id", n_devs)   # List of video device ids 
  names   = get_cam_param("name", n_devs) # Camera names
  des_con = get_cam_param("default", n_devs) # Camera names
  
  cur_con = [False]*n_devs     # Current streams   
  hosts = ["127.0.0.1"]*n_devs # TODO Get host from service call
   
  dev_list = get_vid_devs(id_list) # Get available devices  
  streams = [None]*n_devs          # Initialise stream array
    
  # Initialise default streams
  for i in range(n_devs): 
    if des_con[i] is True:
      start_stream(streams, hosts, port, id_list, i)
  
  while not rospy.is_shutdown():  
      
    for i in range(n_devs):
           
      if des_con[i] is True: # If we have been requested to stream this camera
      
        if streams[i] is None: # If no stream, create one
          start_stream(streams, hosts, port, id_list, i)          
        
        else: # If stream already exists, check status         
          current, _, _ = streams[i].get_state() 
          
          # If stream has failed
          if StreamStatus(current) is not StreamStatus.GST_STATE_SUCCESS:          
            if cur_con[i] is True: 
              rospy.loginfo("Camera " + str(i) + " (" + names[i] 
                            + ") disconnected. Attempting to reconnect.") 
                            
            streams[i].set_state (gst.STATE_NULL); # Stop stream
                         
            start_stream(streams, hosts, port, id_list, i) # Restart stream
            
            cur_con[i] = False    # Record that the stream is inactive 
            
          # If stream fine, give status message
          elif cur_con[i] is False:         
           
            rospy.loginfo("Camera " + str(i) + " (" + names[i] 
                          + ") successfully connected.") 
            cur_con[i] = True     # Record that the stream is now active
    
    rate.sleep()
  
 
  
  # TODO if you try to stream the camera too early you can get an error
  # TODO wait slightly longer and it looks fine but client won't connect
  # TODO after a few seconds it's gucci
    


#--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
# Initialiser.
#--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..-- 
if __name__ == '__main__':
  try:
    start = main()   
    gtk.main()
       
  except rospy.ROSInterruptException:
    pass
        
    
'''sudo apt-get install 
python-gst0.10 
gstreamer0.10-plugins-good 
gstreamer1.0-plugins-ugly'''
