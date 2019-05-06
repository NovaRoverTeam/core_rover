#!/bin/bash
rosbag play stereo_outdoor.bag /stereo_camera/left/camera_info_throttle:=/stereo/left/info /stereo_camera/right/camera_info_throttle:=/stereo/right/info /stereo_camera/left/image_raw_throttle/compressed:=/stereo/left/raw /stereo_camera/right/image_raw_throttle/compressed:=/stereo/right/raw
