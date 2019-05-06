#!/bin/bash
rosbag play field_test_ekf.bag /px4/raw/gps:=/nova_common/gps_data /px4/raw/imu:=/nova_common/IMU
