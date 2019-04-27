#!/bin/bash
rosbag play field_test_19-04_imu_gps.bag /px4/raw/gps:=/nova_common/gps_data /px4/raw/imu:=/nova_common/IMU
