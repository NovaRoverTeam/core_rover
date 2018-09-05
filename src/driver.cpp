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

//--**-- Standard includes
#include <stdlib.h>
#include <signal.h>
#include <iostream>
#include <math.h>
#include <string>
using namespace std;


//--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
// drive_cmd_cb():
//
//    Callback function for receiving driving commands from the 
//    rover_manager.
//--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--
void drive_cmd_cb(const nova_common::DriveCmd::ConstPtr& msg)
{   
  // TODO do something with the drive commands
}


//--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
// SigintHandler():
//
//		Overrides the default SIGINT exit function when pressing CTRL+C.
//--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--
void SigintHandler(int sig)
{
  // All the default sigint handler does is call shutdown()
  ros::shutdown();
}


//--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
// main():
//
//    Main function. This will run first.
//--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--
int main(int argc, char **argv)
{
  ros::init(argc, argv, "driver"); // Initialise node
  ros::NodeHandle n; // Create node handle to talk to ROS  
  ros::Rate loop_rate(1); // Main loop rate, 1 Hz

  // Declare subscriber to drive cmds
  ros::Subscriber drive_cmd_sub = 
    n.subscribe("/rover_manager/drive_cmd", 1, drive_cmd_cb); 

  // TODO Declare service clients for simulator rover stuff

  // Boolean variable describing whether we are using the simulator
  // or a real rover. Later we will retrieve this as a launch
  // file parameter.
  bool simulator = true; 

  if (simulator)
  {
    // TODO Initialise the service clients you declared above. Note the
    // difference between declaring and initialising. Above, you want 
    // to simply declare that the clients exist but give them no value.
    // Here, you want to actually create the client. The point of this 
    // is to avoid scope issues (if you both declare and initialise the
    // client within this "if" statement, it will be destroyed once the
    // program exits the statement).
  }

  while (ros::ok()) // Main loop
  {
    if (simulator)
    {
      // TODO Use the received drive commands to direct the virtual 
      // rover via service calls.
    }
    else
    {
      // Do nothing for now. Later we will communicate with the
      // real rover.
    }

    ros::spinOnce();   // Messages are received and callbacks called
    loop_rate.sleep(); // Sleep for the specified rate
  }

  return 0;
}
