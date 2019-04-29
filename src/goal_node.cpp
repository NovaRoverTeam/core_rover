//--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
// File: goal.cpp
//
// Description:
// The goal node receives message from "/nova_common/gps_data"
// target gps coordinate and convert it
// to geometry_msgs/PoseStamped Message and pubish to 
// topic 'planner_goal' (remapped from 'move_base_simple/goal') 
// for move_base to consume
//
//--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--

#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include "geometry_mesgs/Quaternion"
#include <sensor_msgs/NavSatFix.h>

#include <sstream>

ros::Publisher pub = nh.advertise<sgeometry_msgs/Pose>("geometry_msgs/Twist", 1000);

/**
 *  This handles the convertion between geometry_msgs/Twist and DriveCmd
 */
void goalCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
    
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "controller");

    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("/nova_common/gps_data", 1000, goalCallback);

    ros::spin();

    return 0;
}
