#!/bin/bash
rosbag play imu-gps-test-small-fix.bag /px4/raw/gps:=/nova_common/MagnetometerRawSmoothed /px4/raw/imu:=/nova_common/ImuRaw
