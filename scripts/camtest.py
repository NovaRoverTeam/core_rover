#!/usr/bin/env python

import rospy

import gi
gi.require_version('Gst', '1.0')
gi.require_version('GstBase', '1.0')
gi.require_version('Gtk', '3.0')
from gi.repository import GObject, Gst, GstBase, Gtk, GObject
  
def camtest():

    '''rospy.init_node('auto2', anonymous=True)
    rate = rospy.Rate(2) # Loop rate in Hz
    
    description = ("rtspsrc location=rtsp://nova:rovanova@192.168.1.52:88/videoMain ! decodebin ! xvimagesink")
    rospy.loginfo("%s", description)
    cat = gst.parse_launch(description)
    cat.set_state(gst.STATE_PLAYING)
    rospy.loginfo("cat")
    
    while not rospy.is_shutdown():  
      
   
        rate.sleep()'''
        
    class cheese:
     def __init__(self):
         Gst.init(None)
         
         command = "v4l2src device=/dev/video1 ! xvimagesink"
         self.pipeline= Gst.parse_launch(command)

         self.pipeline.set_state(Gst.State.PLAYING)
         rospy.loginfo("%s", self.pipeline)

    start=cheese()
    Gtk.main()

    
if __name__ == '__main__':
    camtest()

