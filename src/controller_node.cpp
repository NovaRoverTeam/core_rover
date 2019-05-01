
//--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
// File: controller.cpp
//
// Description:
//  The controller node receives a stream of velocity commands  meant 
//  for execution by a mobile base from the move_base node and then 
//  transform it into a stream of drive commands to the driver node
//
//--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--

#include <stdint.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <nova_common/DriveCmd.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>


ros::Subscriber sub;
ros::Publisher pub;


/**
 *  This handles the convertion between geometry_msgs/Twist and DriveCmd
 */
float linearAccelerationToRpm(geometry_msgs::Vector3 linear) {
    // scale between -50 to 50 int16
    return (float) linear.y;
}

int16_t angularAccelerationToSteer(geometry_msgs::Vector3 angular) {
    // scale between -100 to 100 float32
    // multiply by -1 because angular.z > 0 means turn left
    // but drive_msg.steer < 0 also means turn left 
    return -1 * (int16_t) angular.z;
}

void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    nova_common::DriveCmd cmd;
    cmd.rpm = linearAccelerationToRpm(msg->linear);
    cmd.steer_pct = angularAccelerationToSteer(msg->angular);
    pub.publish(cmd);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "controller");

    ros::NodeHandle n;
    sub = n.subscribe("cmd_vel", 1000, cmdVelCallback);
    pub = n.advertise<nova_common::DriveCmd>("/core_rover/driver/drive_cmd", 1000);

    while(ros::ok())
    {
        ros::spinOnce();
    }

    return 0;
}