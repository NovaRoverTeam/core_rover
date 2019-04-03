/***************************************************************************************************
* NOVA ROVER TEAM - URC2018
* This code is a ROS package which periodically retrieves data from a Waveshare NEO-7M-C GPS module.
* The data is retrieved via UART, and is parsed through to obtain the latitude and longitude
* GPS coordinates.
*
* The GPS module being used:
* www.robotshop.com/ca/en/uart-neo-7m-c-gps-module.html
*
* Author: Andrew Stuart
* Last Modified by: Matthew Harker (29/03/2019)
***************************************************************************************************/
/***************************************************************************************************
* INCLUDES, DECLARATIONS AND GLOBAL VARIABLES
***************************************************************************************************/
#include "ros/ros.h"
#include <string.h>
#include <stdio.h>
#include <fcntl.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <errno.h>
#include <boost/algorithm/string.hpp>
//#include <cmath>
#include <sensor_msgs/NavSatFix.h>
#include <sstream>
#define LOOP_HERTZ 1	// GPS sends data every ~1 second
std::string s_data_array;	// Holds data from the GLL line to be parsed
bool use_fake = false; // Use fake coords?
bool test_data = false;
double fake_lat, fake_long; // Fake GPS coords
bool print_coords = true;
bool print_ground_speed = true;
int gps_fix_aquired = -1;
ros::Publisher lat_long_pub;

/**************************************************************************************************
* DEGREES-MINUTES-SECONDS TO DECIMAL DEGREES CONVERSION FUNCTION
*
* Converts a number in decimal-minutes-seconds to decimal degrees and returns it.
*
* Inputs: int deg - the degree value of the coordinate
* int min - the minutes value of the coordinate
* float sec - the seconds value of the coordinate, should be a decimal
* int dir - the direction of the coordinate as a positive or negative multiplier (1/-1).
*
* Output: float DecDeg - the decimal degrees representation of the DMS number
**************************************************************************************************/
float ConvertDMSToDD(int deg, int min, float sec, int dir) {
	float DecDeg = (float)dir * (((float)deg) + ((float)min/60) + (sec/3600));
	return DecDeg;
}

int StrToInt(std::string value) {
	int output = 0;
	
	for (int i = 0; i < value.length(); i++) {
		output *= 10;
		output += (int)(value[i] - '0');
	}
	
	return output;
}

float StrToFloat(std::string value) {
	float output = 0.0f;
	int decimalPlaces = -1;
	
	for (int i = 0; i < value.length(); i++) {
		if (value[i] == '.') {
			decimalPlaces++;
		} else {
			if (decimalPlaces >= 0) {
				decimalPlaces++;
			}
			
			output *= 10.0f;
			output += (float)(value[i] - '0');
		}
	}
	
	if (decimalPlaces >= 0) {
		output /= (pow(10, decimalPlaces));
	}
	return output;
}

/***************************************************************************************************
* DATA PARSING FUNCTION
* This function parses the GPRMC line sent by module for the GPS coordinates.
* The coordinates are then published to ROS.
* 
* Ground speed and Heading are also extracted from the GPS module, but are currently not published.
* 
* The GPRMC line is stored in the global variable s_data_array as a string, with the line identifier removed
* Data is stored in comma-delimited fields of varying lengths.
***************************************************************************************************/
void process_GPRMC() {
	// example data for testing without GPS module
	if (test_data) {
		s_data_array = ",123456.78,A,1234.56,N,12345.67,E,1.2,1.2,1234,1.2,E,A*3D";
	}
				
	std::vector<std::string> segments;
	boost::split(segments, s_data_array, [](char c){return (c ==',' || c == '*');}); // split into comma delimited segments
	
	// initial data field validation
	if (segments.size() != 14) {
		ROS_INFO_STREAM("Incomplete GPS data line");
		return;
	}
	
	char checksum = 'G'^'P'^'R'^'M'^'C'; // XOR checksum
	
	int target_checksum = 0;
	sscanf(segments[13].c_str(), "%x", &target_checksum);
	
	// iterate bitwise XOR for each character (excluding checksum itself)
	for (int i = 0; i < s_data_array.length() - 1 - segments[13].length(); i++) {
		checksum ^= s_data_array[i];
	}
	
	// verify checksum
	if (checksum != target_checksum) {
		ROS_INFO_STREAM("GPS checksum mismatch");
		
	} else if (segments[2] == "A") { // valid GPS reading
		// inform that GPS has a fix
		if (gps_fix_aquired != 1) {
			gps_fix_aquired = 1;
			ROS_INFO_STREAM("GPS Fix Aquired");
		}
		
		sensor_msgs::NavSatFix msg;
		
		std::string s_timestamp = segments[1]; // timestamp (UTC)    - not used yet
		std::string s_datestamp = segments[9]; // datestamp (ddmmyy) - not used yet
		
		// extract latitude
		std::string s_latitude = segments[3];
		msg.latitude = ConvertDMSToDD(
			StrToInt(s_latitude.substr(0,2)),         // degrees
			StrToInt(s_latitude.substr(2,2)),         // minutes
			StrToFloat(s_latitude.substr(4,-1)) * 60, // seconds
			(segments[4] == "N") ? 1 : -1             // direction
		);
		
		// extract longitude
		std::string s_longitude = segments[5];
		msg.longitude = ConvertDMSToDD(
			StrToInt(s_longitude.substr(0,3)),         // degrees
			StrToInt(s_longitude.substr(3,2)),         // minutes
			StrToFloat(s_longitude.substr(5,-1)) * 60, // seconds
			(segments[6] == "E") ? 1 : -1              // direction
		);
		
		if (print_coords) { // publish coordinates to ROS
			//ROS_INFO("Latitude: %f, Longitude: %f", msg.latitude, msg.longitude);
			lat_long_pub.publish(msg);
		}
		
		// extract GPS ground speed
		std::string s_ground_speed = segments[7];
		float ground_speed = StrToFloat(s_ground_speed) / 1.944f; // convert knots to m/s
		
		// extract GPS heading (degrees true north)
		std::string s_heading = segments[8];
		float heading = StrToFloat(s_heading);
		
		if (print_ground_speed) { // publish ground speed and heading to ROS (info stream only)
			ROS_INFO_STREAM("Ground Speed: " << ground_speed << " m/s, Heading: " << heading << " degrees");
		}
	} else { // unable to get GPS data
		// inform that GPS fix is lost
		if (gps_fix_aquired == 1) {
			ROS_INFO_STREAM("GPS Fix Lost");
			gps_fix_aquired = 0;
		} else if (gps_fix_aquired == -1) {
			ROS_INFO_STREAM("No GPS Fix");
			gps_fix_aquired = 0;
		}
	}
}

/***************************************************************************************************
* MAIN FUNCTION
* Sets up UART connection and periodically reads data on connection for new GPS data.
* This parsed data is then published via ROS.
* Published messages:
* Valid Reading - Sends ROS message containing latitude and longitude in  decimal degrees format.
*                 Sends Ground speed (m/s) and heading (degrees True North) on ROS info stream
* 
* Invalid Reading - Nothing. A warning will be printed to the ROS info stream to inform the user.
***************************************************************************************************/
int main(int argc, char **argv) {
	ros::init(argc, argv, "gps"); // Initialise ROS package
	ros::NodeHandle n("/");
	
	lat_long_pub = n.advertise<sensor_msgs::NavSatFix>("/nova_common/gps_data", 1000);
	ros::Rate loop_rate(LOOP_HERTZ); // Define loop rate
	
	int fd; // UART file descriptor
	char uartChar;	// The character retrieved from the UART RX buffer
	int lineType = -1; // GPS data line type: {-1 = unknown, 0 = unused, 1 = GPRMC}
	std::string s_lineID = "";
	unsigned int lineIndex = 0;
	
	// Grab ROS params
	//n.getParam("/gps/use_fake", use_fake);
	//n.getParam("/gps/fake_lat", fake_lat);
	//n.getParam("/gps/fake_long", fake_long);
	//use_fake=false;
	fake_lat = -37.9089064;
	fake_long = 145.1338591;
	
	// Attempt to open UART
	if((fd = open("/dev/ttyTHS2",9600))<0) {
		ROS_INFO_STREAM("GPS Serial Connection Failed");
		return 1;
	} else {
		ROS_INFO_STREAM("GPS Connected");
	}
	
	// main ROS loop
	while (ros::ok()) {
		// check UART buffer
		size_t nb;
		ioctl(fd, FIONREAD, &nb);
		
		if (!use_fake) {
			while(nb > 0) {
				// If there is new UART data...
				// ... retrieve the next character
				read(fd, &uartChar, 1);
				//printf("%c", uartChar); // debug: print UART input
				lineIndex++;
				
				if(uartChar == '$') { // start of new line
					lineType = -1;
					lineIndex = 0;
					s_lineID = "";
					s_data_array = "";
				} else if (lineType == -1) {
					// within the line ID
					s_lineID += uartChar;
					
					if (lineIndex == 5) { // end of lineID segment - set lineType based on lineID
						if (s_lineID == "GPRMC") {
							lineType = 1;
						} else {
							lineType = 0;
						}
					}
				} else if (lineType == 1) { // GPMRC line
					if (uartChar == '\n') {
						// Process Data
						process_GPRMC();
					} else if (s_data_array.length() < 60) { // limit uart line length (only occurs if uart is corrupted)
						s_data_array += uartChar;
					}
				}
			
				fflush(stdout);	// Flush buffer
				ioctl(fd, FIONREAD, &nb); // read number of bytes in buffer
			}
		} else { // faking data
			sensor_msgs::NavSatFix msg;
			
			msg.latitude = fake_lat;
			msg.longitude = fake_long;
			
			ROS_INFO("(Fake) Latitude: %f, Longitude: %f", fake_lat, fake_long);
			lat_long_pub.publish(msg);
		}
				
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
