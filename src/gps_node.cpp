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
//#include <cmath>
#include <sensor_msgs/NavSatFix.h>
#include <sstream>
#define LOOP_HERTZ 1	// GPS sends data every ~1 second
std::string s_data_array;	// Holds data from the GLL line to be parsed

std::vector<std::string> splitString(const std::string &_source, const std::string &_delimiter = "", bool _ignoreEmpty = false) {
    std::vector<std::string> result;
    if (_delimiter.empty()) {
        result.push_back(_source);
        return result;
    }
    std::string::const_iterator substart = _source.begin(), subend;
    while (true) {
        subend = search(substart, _source.end(), _delimiter.begin(), _delimiter.end());
        std::string sub(substart, subend);
        
        if (!_ignoreEmpty || !sub.empty()) {
            result.push_back(sub);
        }
        if (subend == _source.end()) {
            break;
        }
        substart = subend + _delimiter.size();
    }
    return result;
}

int toInt(const std::string &_intString) {
    int x = 0;
    std::istringstream cur(_intString);
    cur >> x;
    return x;
}

float toFloat(const std::string &_floatString) {
    float x = 0;
    std::istringstream cur(_floatString);
    cur >> x;
    return x;
}

void getLocation(std::string line, float *lat, float *lon){
	std::vector<std::string> data = splitString(line,",");

	// If got a FIX
	if (data[2] == "A"){
	    // parse the data
	    *lat = 0.0f;
	    *lon = 0.0f;

	    // LATITUD
	    *lat = toInt(data[3].substr(0,2));
	    *lat += toFloat(data[3].substr(2))/60.0;
	    if(data[4]=="S")
		*lat *= -1.0;

	    // LONGITUD
	    *lon = toInt(data[5].substr(0,3));
	    *lon += toFloat(data[5].substr(3))/60.0;
	    if(data[6]=="W")
		*lon *= -1.0;
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
	
	ros::Publisher lat_long_pub = n.advertise<sensor_msgs::NavSatFix>("/nova_common/gps_data", 1000);
	ros::Rate loop_rate(LOOP_HERTZ); // Define loop rate
	
	int fd; // UART file descriptor
	char uartChar;	// The character retrieved from the UART RX buffer
	int lineType = -1; // GPS data line type: {-1 = unknown, 0 = unused, 1 = GPRMC}
	std::string s_lineID = "";
	unsigned int lineIndex = 0;
		
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
					float lat, lon;
					sensor_msgs::NavSatFix msg;
					getLocation(s_data_array,&lat,&lon);
					msg.latitude = lat;
					msg.longitude = lon;
					lat_long_pub.publish(msg);
				} else if (s_data_array.length() < 60) { // limit uart line length (only occurs if uart is corrupted)
					s_data_array += uartChar;
				}
			}
		
			fflush(stdout);	// Flush buffer
			ioctl(fd, FIONREAD, &nb); // read number of bytes in buffer
		}
				
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
