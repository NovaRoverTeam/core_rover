/*
 * To run gmapping you should start gmapping:
 * rosrun gmapping slam_gmapping scan:=/pioneer3at/Sick_LMS_291/laser_scan/layer0 _xmax:=30 _xmin:=-30 _ymax:=30 _ymin:=-30 _delta:=0.2
 */

#include "ros/ros.h"

#include <webots_ros/set_float.h>
#include <webots_ros/set_int.h>

#define TIME_STEP         32
#define NMOTORS            4

ros::NodeHandle *n;

ros::ServiceClient timeStepClient;
webots_ros::set_int timeStepSrv;

static const char *motorNames[NMOTORS] = {
  "front_left_wheel", "front_right_wheel",
  "back_left_wheel", "back_right_wheel"
};

int main(int argc, char **argv) {

  if (argc != 1) {
    ROS_INFO("Usage: $ init_simulator.");
    return 1;
  }

  // create a node named 'init_simulator' on ROS network
  ros::init(argc, argv, "init_simulator", ros::init_options::AnonymousName);
  n = new ros::NodeHandle;
  
  ros::spinOnce();

  timeStepClient = n->serviceClient<webots_ros::set_int>("pioneer3at/robot/time_step");
  timeStepSrv.request.value = TIME_STEP;

  // init motors
  for (int i=0; i<NMOTORS; ++i) {
    // position
    ros::ServiceClient set_position_client;
    webots_ros::set_float set_position_srv;
    set_position_client = n->serviceClient<webots_ros::set_float>(std::string("pioneer3at/") + std::string(motorNames[i]) + std::string("/set_position"));

    set_position_srv.request.value = INFINITY;
    if (set_position_client.call(set_position_srv) && set_position_srv.response.success)
      ROS_INFO("Position set to INFINITY for motor %s.", motorNames[i]);
    else
      ROS_ERROR("Failed to call service set_position on motor %s.", motorNames[i]);

    // speed
    ros::ServiceClient set_velocity_client;
    webots_ros::set_float set_velocity_srv;
    set_velocity_client = n->serviceClient<webots_ros::set_float>(std::string("pioneer3at/") + std::string(motorNames[i]) + std::string("/set_velocity"));

    set_velocity_srv.request.value = 0.0;
    if (set_velocity_client.call(set_velocity_srv) && set_velocity_srv.response.success == 1)
      ROS_INFO("Velocity set to 0.0 for motor %s.", motorNames[i]);
    else
      ROS_ERROR("Failed to call service set_velocity on motor %s.", motorNames[i]);
  }

  // enable lidar
  ros::ServiceClient set_lidar_client;
  webots_ros::set_int lidar_srv;
  set_lidar_client = n->serviceClient<webots_ros::set_int>("pioneer3at/Sick_LMS_291/enable");
  lidar_srv.request.value = TIME_STEP;
  if (set_lidar_client.call(lidar_srv) && lidar_srv.response.success) {
    ROS_INFO("Lidar enabled.");
    ROS_INFO("Topic for lidar initialized.");
    ROS_INFO("Topic for lidar scan connected.");
  } else {
    if ( !lidar_srv.response.success)
      ROS_ERROR("Sampling period is not valid.");
    ROS_ERROR("Failed to enable lidar.");
    return 1;
  }

  // enable gps
  
  ros::ServiceClient set_GPS_client;
  webots_ros::set_int GPS_srv;
  set_GPS_client = n->serviceClient<webots_ros::set_int>("pioneer3at/gps/enable");
  GPS_srv.request.value = 32;
  if (set_GPS_client.call(GPS_srv) && GPS_srv.response.success) 
    ROS_INFO("GPS enabled.");
    else {
    if ( !GPS_srv.response.success)
      ROS_ERROR("Sampling period is not valid.");
    ROS_ERROR("Failed to enable GPS.");
    return 1;
  }
  
  // enable compass
  ros::ServiceClient set_compass_client;
  webots_ros::set_int compass_srv;
  set_compass_client = n->serviceClient<webots_ros::set_int>("pioneer3at/compass/enable");
  compass_srv.request.value = 32;
  if (set_compass_client.call(compass_srv) && compass_srv.response.success) {
    ROS_INFO("compass enabled.");
  } else {
    if ( !compass_srv.response.success)
      ROS_ERROR("Sampling period is not valid.");
    ROS_ERROR("Failed to enable compass.");
    return 1;
  }

  // enable inertial unit
  ros::ServiceClient set_inertial_unit_client;
  webots_ros::set_int inertial_unit_srv;
  set_inertial_unit_client = n->serviceClient<webots_ros::set_int>("pioneer3at/inertial_unit/enable");
  inertial_unit_srv.request.value = 32;
  if (set_inertial_unit_client.call(inertial_unit_srv) && inertial_unit_srv.response.success) {
    ROS_INFO("Inertial unit enabled.");
  } else {
    if ( !inertial_unit_srv.response.success)
      ROS_ERROR("Sampling period is not valid.");
    ROS_ERROR("Failed to enable inertial unit.");
    return 1;
  }

  // enable accelerometer
  ros::ServiceClient set_accelerometer_client;
  webots_ros::set_int accelerometer_srv;
  set_accelerometer_client = n->serviceClient<webots_ros::set_int>("pioneer3at/accelerometer/enable");
  accelerometer_srv.request.value = 32;
  set_accelerometer_client.call(accelerometer_srv);
  
  // enable camera
  ros::ServiceClient set_camera_client;
  webots_ros::set_int camera_srv;
  set_camera_client = n->serviceClient<webots_ros::set_int>("pioneer3at/camera/enable");
  camera_srv.request.value = 64;
  set_camera_client.call(camera_srv);
  
  // enable gyro
  ros::ServiceClient set_gyro_client;
  webots_ros::set_int gyro_srv;
  set_gyro_client = n->serviceClient<webots_ros::set_int>("pioneer3at/gyro/enable");
  gyro_srv.request.value = 32;
  set_gyro_client.call(gyro_srv);

  // main loop
  while (ros::ok()) {
    if (!timeStepClient.call(timeStepSrv) || !timeStepSrv.response.success) {
      ROS_ERROR("Failed to call service time_step for next step.");
      break;
    }
    ros::spinOnce();
  }
  timeStepSrv.request.value = 0;
  timeStepClient.call(timeStepSrv);

  ros::shutdown();
  return 0;
}
