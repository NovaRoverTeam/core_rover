#!/bin/bash
rosbag play imu-gps-test-small-fix.bag /px4/raw/gps:=/pioneer3at/gps/values /px4/raw/imu:=/pioneer3at/imu/values
