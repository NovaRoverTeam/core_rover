//--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
// File: driver.cpp
// Author: Ben Steer
// Last modified by: Emily Kuo
//
// Description:
//  The driver node receives driving commands from the rover_manager
//  node and interprets these to communicate with the rover's motor
//  controllers to achieve the desired motion. If using the simulator,
//  commands will be given to the virtual rover's controllers instead.
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

//--*-- Talon SRX includes
#define Phoenix_No_WPI // remove WPI dependencies
#include "ctre/Phoenix.h"
#include "ctre/phoenix/platform/Platform.h"
#include "ctre/phoenix/unmanaged/Unmanaged.h"
#include "ctre/phoenix/MotorControl/CAN/WPI_TalonSRX.h"
#include <string>
#include <iostream>
#include <chrono>
#include <thread>
#include <SDL2/SDL.h>
#include <unistd.h>

using namespace std;

int speed;
int steer;

ros::NodeHandle *n; // Create node handle to talk to ROS

//Declaring and creating talonSRX objects to control the 6 motors. 
TalonSRX talon1(1); 
TalonSRX talon2(2);
TalonSRX talon3(3);
TalonSRX talon4(4);
TalonSRX talon5(5);
TalonSRX talon0(0);

//--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
// DriveCmdCb():
//    Callback function for receiving driving commands from the
//    rover_manager.
//--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--
void DriveCmdCb(const nova_common::DriveCmd::ConstPtr& msg)
{
  //Instantiiating vars
  double speedFactor  = 1.0;    //currently not in use, but can be used to increase/decrease speed by a factor

  speed = msg->rpm * 2;     //speed = forward or reverse
  steer = msg->steer_pct; //steer = steering percentage left or right
}

//--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
// main():
//    Main function. This will run first.
//--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--
int main(int argc, char **argv)
{

  //Talon SRX Setup
  std::string interface;
  interface = "can0";
  ctre::phoenix::platform::can::SetCANInterface(interface.c_str());
  talon0.SetInverted(true);
  talon1.SetInverted(true);
  talon2.SetInverted(true);
  talon2.SetNeutralMode(Brake);
  ros::init(argc, argv, "driver", ros::init_options::AnonymousName); // Initialise node
  n = new ros::NodeHandle;

  // Declare subscriber to drive cmds
  ros::Subscriber drive_cmd_sub = n->subscribe("/core_rover/driver/drive_cmd", 1, DriveCmdCb);

  // Boolean variable describing whether we are using the simulator
  // or a real rover. 
  string vehicle;
  bool simulator;
  simulator = false;

	//It's supposed to detect the vehicle but this don't work yet :'(
  //n->getParam("~Vehicle", vehicle);
  //if (vehicle.compare("Simulator")){
  //  simulator = true;
  //}

  double wheel[6]; //array to update motor values

  //Declare service clients for simulator rover stuff
  ros::ServiceClient set_velocity_client;
  core_rover::set_float set_velocity_srv;
  

  while (ros::ok()) // Main loop
  {
    if (simulator) //add condition if auto mode
    {
      // Use the received drive commands to direct the virtual
      // rover via service calls.

      static const char *motorNames[4] = {
        "front_left_wheel", "front_right_wheel",
        "back_left_wheel", "back_right_wheel"
      };
      
      wheel[0] = speed + steer;
      wheel[1] = speed - steer;
      wheel[2] = speed + steer;
      wheel[3] = speed - steer;

      for (int i=0; i<4; ++i) {
      //updating individual motor values to change speed and or direction
      set_velocity_client = n->serviceClient<core_rover::set_float>(std::string("pioneer3at/") + std::string(motorNames[i]) + std::string("/set_velocity"));
      set_velocity_srv.request.value = wheel[i];
      set_velocity_client.call(set_velocity_srv);
      }
    }
    else
    {
      //-50 to 50 for RPM | -100 to 100 for steer
      float talon_speed = speed / 50.0;
      float talon_steer = steer / 100.0;
      //float talon2_speed = talon_speed - talon_steer; 
      //float talon4_speed = talon_speed + talon_steer;
      talon_speed = 0.0;
     /* if(speed>0){
         talon_speed = 0.3;
}
      else if (speed<0){
         talon_speed = -0.3;
}*/

      talon_speed = speed/50.0;
      float right = talon_speed - talon_steer;   //Positive turn decreases right motors speeds to turn right.
      float left = talon_speed + talon_steer;
      
      printf("%f",talon_speed);

      //LEFT SIDE
      talon1.Set(ControlMode::PercentOutput, left);
      talon2.Set(ControlMode::PercentOutput, left);
      talon0.Set(ControlMode::PercentOutput, left);
      //RIGHT SIDE
      talon4.Set(ControlMode::PercentOutput, right);
      talon5.Set(ControlMode::PercentOutput, right);
      talon3.Set(ControlMode::PercentOutput, right);

      //Enable rover with a timeout of 100ms
      ctre::phoenix::unmanaged::FeedEnable(100);
    }

    ros::spinOnce();   // Messages are received and callbacks called
  }

  return 0;
}
