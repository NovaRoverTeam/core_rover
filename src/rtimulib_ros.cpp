// RTIMULib ROS Node
// Copyright (c) 2017, Romain Reignier
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright
//     notice, this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of the <organization> nor the
//     names of its contributors may be used to endorse or promote products
//     derived from this software without specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include <RTIMULib.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <nova_common/RPY.h>
#include <math.h> 
#include <tuple>

static const double G_TO_MPSS = 9.80665;

std::tuple<float, float, float> quaternion_to_euler(float x, float y, float z, float w){
        
    float t0, t1, t2, t3, t4, X, Y, Z;

    t0 = 2.0 * (w * x + y * z);
    t1 = 1.0 - 2.0 * (x * x + y * y);
    X = 180* (atan2(t0, t1)) / M_PI;

    t2 = 2.0 * (w * y - z * x);
    t2 = (t2 > +1.0) ? 1.0 : t2;
    t2 = (t2 < -1.0) ? -1.0 : t2;
    Y = 180 * (asin(t2))/ M_PI;

    t3 = 2.0 * (w * z + x * y);
    t4 = 1.0 - 2.0 * (y * y + z * z);
    Z = 180 * (atan2(t3, t4))/M_PI;

    return std::make_tuple(X, Y, Z);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rtimulib_node",ros::init_options::AnonymousName);
    ROS_INFO("Imu driver is now running");
    ros::NodeHandle nh("~");

    std::string calibration_file_path;
    if(!nh.getParam("calibration_file_path", calibration_file_path))
    {
        ROS_ERROR("The calibration_file_path parameter must be set to use a "
                  "calibration file.");
        ROS_BREAK();
    }

    std::string calibration_file_name = "RTIMULib";
    if(!nh.getParam("calibration_file_name", calibration_file_name))
    {
        ROS_WARN_STREAM("No calibration_file_name provided - default: "
                        << calibration_file_name);
    }

    ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("/nova_common/IMU", 1);
    ros::Publisher RPY_pub = nh.advertise<nova_common::RPY>("/nova_common/RPY", 1);

    // Load the RTIMULib.ini config file
    RTIMUSettings *settings = new RTIMUSettings(calibration_file_path.c_str(),
                                                calibration_file_name.c_str());

    RTIMU *imu = RTIMU::createIMU(settings);

    if ((imu == NULL) || (imu->IMUType() == RTIMU_TYPE_NULL))
    {
        ROS_ERROR("No Imu found");
        ROS_BREAK();
    }

    // Initialise the imu object
    imu->IMUInit();

    // Set the Fusion coefficient
    imu->setSlerpPower(0.02);
    // Enable the sensors
    imu->setGyroEnable(true);
    imu->setAccelEnable(true);
    imu->setCompassEnable(true);

    sensor_msgs::Imu imu_msg;
    nova_common::RPY RPY_msg;


    while (ros::ok())
    {
        if (imu->IMURead())
        {
            RTIMU_DATA imu_data = imu->getIMUData();

            imu_msg.header.stamp = ros::Time::now();
            imu_msg.header.frame_id = "base_link";

            imu_msg.orientation.x = imu_data.fusionQPose.x(); 
            imu_msg.orientation.y = imu_data.fusionQPose.y(); 
            imu_msg.orientation.z = imu_data.fusionQPose.z(); 
            imu_msg.orientation.w = imu_data.fusionQPose.scalar(); 

            imu_msg.angular_velocity.x = imu_data.gyro.x();
            imu_msg.angular_velocity.y = imu_data.gyro.y();
            imu_msg.angular_velocity.z = imu_data.gyro.z();

            imu_msg.linear_acceleration.x = imu_data.accel.x() * G_TO_MPSS;
            imu_msg.linear_acceleration.y = imu_data.accel.y() * G_TO_MPSS;
            imu_msg.linear_acceleration.z = imu_data.accel.z() * G_TO_MPSS;

            imu_pub.publish(imu_msg);
            
            float roll, pitch, yaw;            
            std::tie(roll, pitch, yaw) =  quaternion_to_euler(imu_msg.orientation.x,imu_msg.orientation.y,imu_msg.orientation.z,imu_msg.orientation.w);
            RPY_msg.roll = roll;
            RPY_msg.pitch = pitch;
            RPY_msg.yaw = yaw;

            RPY_pub.publish(RPY_msg);

        }
        ros::spinOnce();
        ros::Duration(imu->IMUGetPollInterval() / 1000.0).sleep();
    }
    return 0;
}
