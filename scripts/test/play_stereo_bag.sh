#!/bin/bash
rosbag play --clock stereo_outdoor.bag /stereo_camera/left/camera_info_throttle:=/elp/left/camera_info /stereo_camera/right/camera_info_throttle:=/elp/right/camera_info /stereo_camera/left/image_raw_throttle/compressed:=/elp/left/image_raw/compressed /stereo_camera/right/image_raw_throttle/compressed:=/elp/right/image_raw/compressed 
