#!/bin/bash
rosbag play imu-gps-test-small-fix.bag /px4/raw/gps:=/nova_common/gps_data /px4/raw/imu:=/nova_common/Imu
