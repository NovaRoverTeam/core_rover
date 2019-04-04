// compile & linking command:
// gcc NovaAhrs.c ../../Fusion/Fusion/FusionBias.c ../../Fusion/Fusion/FusionAhrs.c -lm

//--**-- ROS includes
#include "ros/ros.h"
#include <ros/console.h>
#include "../include/utils/ekf_utils/kalman_filter.h"
#include "../include/utils/ekf_utils/matrix.h"
#include "../include/Fusion/Fusion/Fusion.h"
#include "../include/Fusion/Fusion/FusionAhrs.c"
#include "../include/Fusion/Fusion/FusionBias.c"
#include "../include/utils/lowpassfilter.cpp"
#include "../include/utils/wmm/worldMagneticModel.cpp"
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <stdio.h>
#include <cmath>
#include <sys/time.h>
#include <sensor_msgs/MagneticField.h>

FusionBias fusionBias;
FusionAhrs fusionAhrs;


//Kalman filter configs
// missing r1 - generated using WMM
// missing r2 - generated using 0.98

double kKFProcessNoise_data[3][3] = {
    {0.1, 0, 0}, {0, 0.1, 0}, {0, 0, 0.1} };
double kKFSensorNoise_data[6][6] = {
    {0.1, 0, 0, 0, 0, 0}, {0, 0.1, 0, 0, 0, 0}, {0, 0, 0.1, 0, 0, 0},
    {0, 0, 0, 0.1, 0, 0}, {0, 0, 0, 0, 0.1, 0}, {0, 0, 0, 0, 0, 0.1} };
double kKFInitialCovariance_data[3][3] = {
    {0, 0, 0}, {0, 0, 0}, {0, 0, 0} };
double kKFInitialEstimate_data[4][1] = {
    {1}, {0}, {0}, {0} };


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
	ros::Publisher magRaw_pub = n.advertise<sensor_msgs::MagneticField>("/nova_common/MagnetometerRaw", 1);
 	ros::Publisher magFiltered_pub = n.advertise<sensor_msgs::MagneticField>("/nova_common/MagnetometerFiltered", 1);

    r_vector mag_field_reference;
    mag_field_reference = MagModel(2019.4, 0.0, 0.0, 0.0);

    Matrix kKFProcessNoise(kKFProcessNoise_data);
    Matrix kKFSensorNoise(kKFSensorNoise_data);
    Matrix kKFInitialCovariance(kKFInitialCovariance_data);
    Matrix kKFInitialEstimate(kKFInitialEstimate_data);

    //KalmanFilter kalman_filter(uint16_t(50), kKFProcessNoise, kKFSensorNoise, kKFInitialCovariance, kKFInitialEstimate);

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

	int timeConstant = 1000; //1 second time constant for now
	int milliSamplePeriod =floor(1000*samplePeriod);
	FirstOrderLowPass smoothed_mag_x(timeConstant, milliSamplePeriod); 
	FirstOrderLowPass smoothed_mag_y(timeConstant, milliSamplePeriod);
	FirstOrderLowPass smoothed_mag_z(timeConstant, milliSamplePeriod);

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
		
		signed short mag_x_raw = (buffer[0] << 8) + buffer[1];
		signed short mag_y_raw = (buffer[2] << 8) + buffer[3];
		signed short mag_z_raw = (buffer[4] << 8) + buffer[5];
		FusionVector3 mag_raw = {
			(float)mag_x_raw,
			(float)mag_y_raw,
			(float)mag_z_raw,
		};
		
		//printf("X: %1.6f, Y: %1.6f, Z: %1.6f\n", (float)mag_x_raw/32768, (float)mag_y_raw/32768, (float)mag_z_raw/32768);
		
		// Sensor Fusion
		// Calibrate sensors
		//gyro_cal = FusionCalibrationInertial(gyro_raw, FUSION_ROTATION_MATRIX_IDENTITY, gyro_sensitivity, FUSION_VECTOR3_ZERO);
		//acc_cal = FusionCalibrationInertial(acc_raw, FUSION_ROTATION_MATRIX_IDENTITY, gyro_sensitivity, FUSION_VECTOR3_ZERO);
		//mag_cal = FusionCalibrationMagnetic(mag_raw, FUSION_ROTATION_MATRIX_IDENTITY, hardIronBias);
		
		// Update AHRS
		//gyro_cal = FusionBiasUpdate(&fusionBias, gyro_cal); // gyroscope bias correction
		//FusionAhrsUpdate(&fusionAhrs, gyro_cal, acc_cal, mag_cal, samplePeriod);
		
		// Get Euler Angles
		//eulerAngles = FusionQuaternionToEulerAngles(FusionAhrsGetQuaternion(&fusionAhrs));
		//printf("Roll = %0.1f, Pitch = %0.1f, Yaw = %0.1f\n", eulerAngles.angle.roll, eulerAngles.angle.pitch, eulerAngles.angle.yaw);

		//nova_common::IMU imu_msg;
		//imu_msg.pitch = eulerAngles.angle.pitch;
		//imu_msg.roll = eulerAngles.angle.roll;
		//imu_msg.yaw = eulerAngles.angle.yaw;

		//then match xyz values to 8 compass directions (flat AND 4 tilt directions)
		//look into wireshielding / differential i2c bus 
		//imu_pub.publish(imu_msg);
		sensor_msgs::MagneticField mag_msg;
		mag_msg.magnetic_field.x = (float)mag_x_raw/32768;
		mag_msg.magnetic_field.y = (float)mag_y_raw/32768;
		mag_msg.magnetic_field.z = (float)mag_z_raw/32768;
		magRaw_pub.publish(mag_msg);
		
		sensor_msgs::MagneticField magSmooth;
		magSmooth.magnetic_field.x = smoothed_mag_x.ProcessSample(mag_msg.magnetic_field.x);
		magSmooth.magnetic_field.y = smoothed_mag_y.ProcessSample(mag_msg.magnetic_field.y);
		magSmooth.magnetic_field.z = smoothed_mag_z.ProcessSample(mag_msg.magnetic_field.z);
		
		magFiltered_pub.publish(mag_msg);

	} while(ros::ok());
	
	return 0;
};
