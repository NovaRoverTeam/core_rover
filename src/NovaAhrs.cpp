
#include "ros/ros.h"
#include <ros/console.h>

#include "../include/Fusion/Fusion/Fusion.h"
#include "../include/Fusion/Fusion/FusionAhrs.c"
#include "../include/Fusion/Fusion/FusionBias.c"

#include "../include/Fusion/Fusion.h"
#include "../include/Fusion/FusionAhrs.c"
#include "../include/Fusion/FusionBias.c"
#include "../include/Fusion/FusionCompass.c"
#include "../include/utils/lowpassfilter.cpp"
#include "../include/utils/wmm/worldMagneticModel.h"
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <stdio.h>
#include <cmath>
#include <sys/time.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Wrench.h>
#include <sensor_msgs/Imu.h>

FusionBias fusionBias;
FusionAhrs fusionAhrs;

//Kalman filter configs

constexpr double kKFProcessNoise_data[3][3] = {
    {0.1, 0, 0}, {0, 0.1, 0}, {0, 0, 0.1}};

constexpr double kKFSensorNoise_data[6][6] = {
    {0.1, 0, 0, 0, 0, 0}, {0, 0.1, 0, 0, 0, 0}, {0, 0, 0.1, 0, 0, 0},
    {0, 0, 0, 0.1, 0, 0}, {0, 0, 0, 0, 0.1, 0}, {0, 0, 0, 0, 0, 0.1}};

constexpr double kKFInitialCovariance_data[3][3] = {
    {0, 0, 0}, {0, 0, 0}, {0, 0, 0}};

constexpr double kKFInitialEstimate_data[4][1] = {{1}, {0}, {0}, {0}};

float samplePeriod = 0.05f;

ros::NodeHandle *n; // Create node handle to talk to ROS

FusionVector3 gyro_sensitivity = { // +- 250 deg/s
	0.00763f,
	0.00763f,
	0.00763f,
};

FusionVector3 acc_sensitivity = { // +- 2g
	16.384f,
	16.384f,
	16.384f,
};

FusionVector3 hardIronBias = {
	0.0f,
	0.0f,
	0.0f,
};

int main(int argc, char **argv) {
	
	ros::init(argc, argv, "NovaAhrs", ros::init_options::AnonymousName); // Initialise node
        ros::NodeHandle n;
	ros::Publisher imu_pub = n.advertise<sensor_msgs::Imu>("/nova_common/IMU", 1);
	ros::Publisher mag2_pub = n.advertise<geometry_msgs::Vector3>("/nova_common/magnetometer", 1);
	ros::Publisher mag_pub = n.advertise<geometry_msgs::Wrench>("/nova_common/magnetometer_compare", 1);
	ros::Publisher heading_pub = n.advertise<std_msgs::Float32>("/nova_common/heading", 1);

	// initialise gyroscope bias correction with stationary threshold of 0.5 degrees/s
	FusionBiasInitialise(&fusionBias, 0.5f, samplePeriod);
	
	//initialise AHRS algorithm with gain of 0.5
	FusionAhrsInitialise(&fusionAhrs, 0.95f);
	
	// (optional) magnetic field limit setting (20-70uT valid range [may be a little low])
	//FusionAhrsSetMagneticField(&fusionAhrs, 20.0f, 70.0f);
	
	// declare Sensor Fusion variables
	// raw data
	FusionVector3 acc_raw; // accelerometer
	FusionVector3 gyro_raw; // gyroscope
	FusionVector3 mag_raw; // magnetometer
	// calibrated data
	FusionVector3 acc_cal; // accelerometer
	FusionVector3 gyro_cal; // gyroscope
	FusionVector3 mag_cal; // magnetometer
	// filtered data
	FusionVector3 acc_fil; // accelerometer
	FusionVector3 gyro_fil; // gyroscope
	FusionVector3 mag_fil; // magnetometer
	
	FusionEulerAngles eulerAngles;
	
	// setup i2c buses
	int mpu_i2c;
	int mag_i2c;
	int length = 4;
	unsigned char buffer[6] = {0};
	
	char *filename = (char*)"/dev/i2c-0";
	
	if ((mpu_i2c = open(filename, O_RDWR)) < 0) {
		printf("Failed to open the i2c bus for mpu\n");
		return 1;
	}
	
	int addr_mpu = 0x68; // i2c address of mpu (gyroscope and accelerometer)
	if (ioctl(mpu_i2c, I2C_SLAVE, addr_mpu) < 0) {
		printf("Failed to access bus and/or talk to slave\n");
	}
	
	int addr_mag = 0x0c; // i2c address of magnetometer
	buffer[0] = 0x37; // bypass mpu for direct i2c access to magnetometer
	buffer[1] = 0x02;
	length = 2;
	if (write(mpu_i2c, buffer, length) != length) {
		printf("Error sending on i2c bus - while bypassing mpu\n");
	}
	
	if ((mag_i2c = open(filename, O_RDWR)) < 0) {
		printf("Failed to open the i2c bus for magnetometer\n");
	}
	
	if (ioctl(mag_i2c, I2C_SLAVE, addr_mag) < 0) {
		printf("Failed to access bus and/or talk to slave\n");
	}
	
	// setup initial delay for periodic reading
	struct timeval t_val;
	gettimeofday(&t_val, NULL);
	unsigned long time_now_us = (unsigned long)(t_val.tv_sec * 1000000) + (unsigned long)t_val.tv_usec;
	unsigned long next_read_us = (unsigned long)time_now_us + (unsigned long)(samplePeriod * 1000000.0f);
	
	// Initialise low pass filter for the magnetometer

	int timeConstant = 750; // *1/1000 sec
	int milliSamplePeriod =floor(1000*samplePeriod);
	FirstOrderLowPass smoothed_gyro(timeConstant, milliSamplePeriod); 
	FirstOrderLowPass smoothed_acc(timeConstant, milliSamplePeriod);
	FirstOrderLowPass smoothed_mag(timeConstant, milliSamplePeriod);

	double magSmoothed[3][1];
	FirstOrderLowPass smoothed_acc_x(timeConstant, milliSamplePeriod); 
	FirstOrderLowPass smoothed_acc_y(timeConstant, milliSamplePeriod);
	FirstOrderLowPass smoothed_acc_z(timeConstant, milliSamplePeriod);

	double accSmoothed[3][1];
	FirstOrderLowPass smoothed_gyro_x(timeConstant, milliSamplePeriod); 
	FirstOrderLowPass smoothed_gyro_y(timeConstant, milliSamplePeriod);
	FirstOrderLowPass smoothed_gyro_z(timeConstant, milliSamplePeriod);

	float acc_x,acc_y,acc_z,gyro_x,gyro_y,gyro_z;
	double gyroSmoothed[3][1];
	
	//main loop
	do {	
		// request single magnetometer read
		buffer[0] = 0x0A;
		buffer[1] = 0x11;
		
		write(mag_i2c, buffer, 2);
		
		// sleep until next read time
		gettimeofday(&t_val, NULL);
		time_now_us = (unsigned long)(t_val.tv_sec * 1000000) + (unsigned long)t_val.tv_usec;
		//printf("Time now: %li\nSleeping for: %li us\n", time_now_us, next_read_us - time_now_us);
		usleep(next_read_us - time_now_us);
		
		// set next read time
		gettimeofday(&t_val, NULL);
		next_read_us = (unsigned long)(t_val.tv_sec * 1000000) + (unsigned long)t_val.tv_usec + (unsigned long)(samplePeriod * 1000000.0f);
		
		buffer[0] = 59; // read accelerometer data
		length = 1;
		if (write(mpu_i2c, buffer, length) != length) {
			printf("Error sending on i2c bus - while reading accelerometer\n");
		}
		
		length = 6;
		int bytes_read = (int)read(mpu_i2c, buffer, length);
		//printf("%i - ", bytes_read);
		
		signed short acc_x_raw = (buffer[0] << 8) + buffer[1];
		signed short acc_y_raw = (buffer[2] << 8) + buffer[3];
		signed short acc_z_raw = (buffer[4] << 8) + buffer[5];
		FusionVector3 acc_raw = {(float)acc_x_raw, (float)acc_y_raw, (float)acc_z_raw,};
		
		//printf("X: %1.6f, Y: %1.6f, Z: %1.6f\n", (float)acc_x_raw/32768, (float)acc_y_raw/32768, (float)acc_z_raw/32768);
		
		buffer[0] = 67; // read gyroscope data
		length = 1;
		if (write(mpu_i2c, buffer, length) != length) {
			printf("Error sending on i2c bus - while reading gyroscope\n");
		}
		
		length = 6;
		bytes_read = (int)read(mpu_i2c, buffer, length);
		//printf("%i - ", bytes_read);
		
		signed short gyro_x_raw = (buffer[0] << 8) + buffer[1];
		signed short gyro_y_raw = (buffer[2] << 8) + buffer[3];
		signed short gyro_z_raw = (buffer[4] << 8) + buffer[5];
		FusionVector3 gyro_raw = {(float)gyro_x_raw, (float)gyro_y_raw, (float)gyro_z_raw,};
		
		//printf("X: %1.6f, Y: %1.6f, Z: %1.6f\n", (float)gyro_x_raw/32768, (float)gyro_y_raw/32768, (float)gyro_z_raw/32768);
		
		buffer[0] = 0x03; // read magnetometer data
		length = 1;
		if (write(mag_i2c, buffer, length) != length) {
			printf("Error sending on i2c bus - while reading magnetometer\n");
		}
		
		length = 6;
		bytes_read = (int)read(mag_i2c, buffer, length);
		//printf("%i - ", bytes_read);
		//Different pattern with the gyro and acc!!
		signed short mag_x_raw = (buffer[1] << 8) + buffer[0];
		signed short mag_y_raw = (buffer[3] << 8) + buffer[2];
		signed short mag_z_raw = (buffer[5] << 8) + buffer[4];
		FusionVector3 mag_raw = {
			(float)mag_x_raw,
			(float)mag_y_raw,
			(float)mag_z_raw,
		};
		
		
		
		// Sensor Fusion
		// Calibrate sensors
		gyro_cal = FusionCalibrationInertial(gyro_raw, FUSION_ROTATION_MATRIX_IDENTITY, gyro_sensitivity, FUSION_VECTOR3_ZERO);
		acc_cal = FusionCalibrationInertial(acc_raw, FUSION_ROTATION_MATRIX_IDENTITY, acc_sensitivity, FUSION_VECTOR3_ZERO);
		mag_cal = FusionCalibrationMagnetic(mag_raw, FUSION_ROTATION_MATRIX_IDENTITY, hardIronBias);

		// Filter sensor value
		gyro_fil = smoothed_gyro.FusionVector3LPFilter(gyro_cal);
		acc_fil = smoothed_acc.FusionVector3LPFilter(acc_cal);
		mag_fil = smoothed_mag.FusionVector3LPFilter(mag_cal);

		std_msgs::Float32 heading_msg;
		heading_msg.data = 180 - FusionCompassCalculateHeading(acc_fil, mag_fil);
		heading_pub.publish(heading_msg);
		
		// Update AHRS
		gyro_cal = FusionBiasUpdate(&fusionBias, gyro_cal); // gyroscope bias correction
		FusionAhrsUpdate(&fusionAhrs, gyro_fil, acc_fil, mag_fil, samplePeriod);	
		FusionQuaternion quaternion = FusionAhrsGetQuaternion(&fusionAhrs);
		FusionVector3 linear_acceleration = FusionAhrsGetLinearAcceleration(&fusionAhrs);


		//then match xyz values to 8 compass directions (flat AND 4 tilt directions)
		//look into wireshielding / differential i2c bus 
		//imu_pub.publish(imu_msg);
		
		sensor_msgs::Imu imu_msg;
		imu_msg.orientation.w = quaternion.element.w;
		imu_msg.orientation.x = quaternion.element.x;
		imu_msg.orientation.y = quaternion.element.y;
		imu_msg.orientation.z = quaternion.element.z;

		imu_msg.linear_acceleration.x = linear_acceleration.axis.x/32768;
		imu_msg.linear_acceleration.y = linear_acceleration.axis.y/32768;
		imu_msg.linear_acceleration.z = linear_acceleration.axis.z/32768;
		imu_pub.publish(imu_msg);

		geometry_msgs::Vector3 mag2_msg;
		mag2_msg.x = (float)mag_x_raw;
		mag2_msg.y = (float)mag_y_raw;
		mag2_msg.z = (float)mag_z_raw;
		mag2_pub.publish(mag2_msg);

		//Test filter on the magnetometer values
		geometry_msgs::Wrench mag_msg;
		mag_msg.force.x = mag_cal.axis.x;
		mag_msg.force.y = mag_cal.axis.y;
		mag_msg.force.z = mag_cal.axis.z;
		mag_msg.torque.x = mag_fil.axis.x;
		mag_msg.torque.y = mag_fil.axis.y;
		mag_msg.torque.z = mag_fil.axis.z;
		mag_pub.publish(mag_msg);

	} while(ros::ok());
	
	return 0;
};
