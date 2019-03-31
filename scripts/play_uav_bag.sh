#!/bin/bash
rosbag play imu-gps-test-small-fix.bag /px4/raw/gps:=/gps_data /px4/raw/imu:=/imu_data
