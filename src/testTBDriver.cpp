//--**-- ROS includes
#include "ros/ros.h"
#include <ros/console.h>
#include <nova_common/DriveCmd.h>
#include <sstream>
//..**.. Simulator includes
#include <signal.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32MultiArray.h>
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

int speed;
int steer;





//--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
// TennisBallCb():
//    Callback function to tell the rover which direction to turn and whether to move forward 
//    for the tennis ball 
//--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--
void TennisBallCb(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
  //Instantiiating vars
  //double = 1.0;    //currently not in use, but can be used to increase/decrease speed by a factor
  //ROS_INFO("X: %s", msg->data.c_str());

  //ROS_INFO("Hi");
  //speed = msg->rpm * 2;     //speed = forward or reverse
  //steer = msg->steer_pct; //steer = steering percentage left or right
}


//--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
// main():
//    Main function. This will run first.
//--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--
int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "testTBDriver", ros::init_options::AnonymousName); // Initialise node
  // Declare subscriber to tennis_loc (tennis ball location and coordiantes)
  ros::NodeHandle n; // Create node handle to talk to ROS

  ros::Subscriber tennis_loc_sub = n.subscribe("/core_rover/navigation/tennis_loc", 1, TennisBallCb);
  ros::spin();   // Messages are received and callbacks called
  return 0;
}
