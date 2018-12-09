//--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
// File: driver.cpp
// Author: Ben Steer
// Last modified by: Ben Steer
//
// Description:
//  The driver node receives driving commands from the rover_manager
//  node and interprets these to communicate with the rover's motor
//  controllers to achieve the desired motion. If using the simulator,
//  commands will be given to the virtual rover's controllers instead.
//
// Message to Johnny and Cheston:
//  Please take note of and try to adhere to the programming practices
//  shown in this file. E.g. indentation, commenting, function
//  descriptions, meaningful naming conventions, line spacing, etc.
//  You will thank yourself later! :P
//
//--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--


//--**-- ROS includes
#include "ros/ros.h"
#include <ros/console.h>
#include <nova_common/DriveCmd.h>

//..**.. Simulator includes
#include <signal.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/String.h>
#include <tf/transform_broadcaster.h>

#include <core_rover/set_float.h>
#include <core_rover/set_int.h>

//--**-- Standard includes
#include <stdlib.h>
#include <signal.h>
#include <iostream>
#include <math.h>
#include <string>
using namespace std;

#define TIME_STEP         32
#define NMOTORS            4
#define MAX_SPEED          10
#define OBSTACLE_THRESHOLD 0.1
#define DECREASE_FACTOR    0.9
#define BACK_SLOWDOWN      0.9

bool simulator = true;

ros::NodeHandle *n; // Create node handle to talk to ROS
// ros::ServiceClient timeStepClient;
// core_rover::set_int timeStepSrv;

static const char *motorNames[NMOTORS] = {
  "front_left_wheel", "front_right_wheel",
  "back_left_wheel", "back_right_wheel"
};

//--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
// DriveCmdCb():
//    Callback function for receiving driving commands from the
//    rover_manager.
//--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--
void DriveCmdCb(const nova_common::DriveCmd::ConstPtr& msg)
{
  //Instantiiating vars
  double speedFactor  = 1.0;    //currently not in use, but can be used to increase/decrease speed by a factor
  double speeds[NMOTORS];       //array to update motor values

  int speed = msg->rpm * 2;     //speed = forward or reverse
  int steer = msg->steer_pct; //steer = steering percentage left or right

    //updating individual motor values to change speed and or direction
    speeds[0] = speed + steer;
    speeds[1] = speed - steer;
    speeds[2] = speed + steer;
    speeds[3] = speed - steer;

  // set speeds
  for (int i=0; i<NMOTORS; ++i) {
    ros::ServiceClient set_velocity_client;
    core_rover::set_float set_velocity_srv;
    set_velocity_client = n->serviceClient<core_rover::set_float>(std::string("pioneer3at/") + std::string(motorNames[i]) + std::string("/set_velocity"));
    set_velocity_srv.request.value = speeds[i];
    set_velocity_client.call(set_velocity_srv);
  }
}


//--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
// quit():
//    Overrides the default SIGINT exit function when pressing CTRL+C.
//--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--
void quit(int sig) {
  if (simulator)
  {
    ROS_INFO("User stopped the 'pioneer3at' node.");
    // timeStepSrv.request.value = 0;
    // timeStepClient.call(timeStepSrv);
  } else {}
  ros::shutdown();
  exit(0);
}

//--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
// main():
//    Main function. This will run first.
//--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--
int main(int argc, char **argv)
{

  ros::init(argc, argv, "driver", ros::init_options::AnonymousName); // Initialise node
  n = new ros::NodeHandle;

  // Declare subscriber to drive cmds
  signal(SIGINT, quit);

  //subscribing to drive_cmd topic and callback: DriveCmdCb
  ros::Subscriber drive_cmd_sub = n->subscribe("/core_rover/driver/drive_cmd", 1, DriveCmdCb);

  // TODO Declare service clients for simulator rover stuff

  // Boolean variable describing whether we are using the simulator
  // or a real rover. Later we will retrieve this as a launch
  // file parameter.

  if (simulator)
  {
    // TODO Initialise the service clients you declared above. Note the
    // difference between declaring and initialising. Above, you want
    // to simply declare that the clients exist but give them no value.
    // Here, you want to actually create the client. The point of this
    // is to avoid scope issues (if you both declare and initialise the
    // client within this "if" statement, it will be destroyed once the
    // program exits the statement).


    // timeStepClient = n->serviceClient<core_rover::set_int>("pioneer3at/robot/time_step");
    // timeStepSrv.request.value = TIME_STEP;

      // init motors
     for (int i=0; i<NMOTORS; ++i) {
		// position
    ros::ServiceClient set_position_client;
    core_rover::set_float set_position_srv;
    set_position_client = n->serviceClient<core_rover::set_float>(std::string("pioneer3at/") + std::string(motorNames[i]) + std::string("/set_position"));

    set_position_srv.request.value = INFINITY;
    if (set_position_client.call(set_position_srv) && set_position_srv.response.success)
      ROS_INFO("Position set to INFINITY for motor %s.", motorNames[i]);
    else
      ROS_ERROR("Failed to call service set_position on motor %s.", motorNames[i]);

    // speed
    ros::ServiceClient set_velocity_client;
    core_rover::set_float set_velocity_srv;
    set_velocity_client = n->serviceClient<core_rover::set_float>(std::string("pioneer3at/") + std::string(motorNames[i]) + std::string("/set_velocity"));

    set_velocity_srv.request.value = 0.0;
    if (set_velocity_client.call(set_velocity_srv) && set_velocity_srv.response.success == 1)
      ROS_INFO("Velocity set to 0.0 for motor %s.", motorNames[i]);
    else
      ROS_ERROR("Failed to call service set_velocity on motor %s.", motorNames[i]);

    ROS_INFO("%d", int(set_velocity_client.call(set_velocity_srv)));
      ROS_INFO("%d", int(set_velocity_srv.response.success));
    }
  }

  while (ros::ok()) // Main loop
  {
    if (simulator)
    {
      // TODO Use the received drive commands to direct the virtual
      // rover via service calls.
      // if (!timeStepClient.call(timeStepSrv) || !timeStepSrv.response.success) {
      //   ROS_ERROR("Failed to call service time_step for next step.");
      //   //break;
      // }
    }
    else
    {
      // Do nothing for now. Later we will communicate with the
      // real rover.
    }

    ros::spinOnce();   // Messages are received and callbacks called
     //loop_rate.sleep(); // Sleep for the specified rate
  }

  return 0;
}
