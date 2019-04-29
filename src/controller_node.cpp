
//--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
// File: controller.cpp
//
// Description:
//  The controller node receives a stream of velocity commands  meant 
//  for execution by a mobile base from the move_base node and then 
//  transform it into a stream of drive commands to the driver node
//
//--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <nova_common/DriveCmd.h>

#include <sstream>

/**
 *  This handles the convertion between geometry_msgs/Twist and DriveCmd
 */
void cmdCallback(const move_base::Twist::ConstPtr& msg) {

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "controller");

    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("cmd_vel", 1000, cmdCallback)

    ros::Publisher chatter_pub = 
}