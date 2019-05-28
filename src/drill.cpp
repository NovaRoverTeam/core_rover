

#include "ros/ros.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdint.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <sys/ioctl.h>
#include "std_msgs/Int8.h"
#include <std_msgs/Empty.h>
#include <iostream>
#define LOOP_HERTZ 50

ros::NodeHandle *nh;
std::string inputString;
int fd;
// Forward declare functions
bool isNumber(std::string str);
bool hbeat = false;
int hbeat_cnt = 0;
const int hbeat_timeout = 0.5*LOOP_HERTZ; //Duration without heartbeat before turning off
int stop = 4;

//--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
// DrillCmdCb():
//    Receives int8 ros messages from /base_station/drill_cmd and sends it on the serial
//--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--
void DrillCmdCb(const std_msgs::Int8::ConstPtr& msg)
{
  int data = msg->data;
  ROS_INFO("%d",msg->data);
  ROS_INFO("test");
  write(fd, &data, 1);
}

//--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
// HbeatCb():
//    Receives empty messages at 1 Hz from base_sync
//--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--
void HbeatCb(const std_msgs::Empty::ConstPtr& msg)
{
  hbeat = true;
  hbeat_cnt = 0;
}

int main(int argc, char *argv[])
{

  int n, i;

  ros::init(argc,argv, "drill", ros::init_options::AnonymousName);
  nh = new ros::NodeHandle;
  ros::Subscriber drill_cmd_sub = nh->subscribe("/base_station/drill_cmd", 1, DrillCmdCb);
  ros::Subscriber hbeat_sub = nh->subscribe("/heartbeat", 1, HbeatCb);

  ros::Rate loop_rate(LOOP_HERTZ);

  char buf[64] = "temp text";
  struct termios toptions;

  /* open serial port */
  fd = open("/dev/megaduino", O_RDWR | O_NOCTTY);
  printf("fd opened as %i\n", fd);
  
  /* wait for the Arduino to reboot */
  usleep(3500000);
  
  /* get current serial port settings */
  tcgetattr(fd, &toptions);
  /* set 9600 baud both ways */
  cfsetispeed(&toptions, B9600);
  cfsetospeed(&toptions, B9600);
  /* 8 bits, no parity, no stop bits */
  toptions.c_cflag &= ~PARENB;
  toptions.c_cflag &= ~CSTOPB;
  toptions.c_cflag &= ~CSIZE;
  toptions.c_cflag |= CS8;
  /* Canonical mode */
  toptions.c_lflag |= ICANON;
  /* commit the serial port settings */
  tcsetattr(fd, TCSANOW, &toptions);
  ROS_INFO("Ready to send");

  while (ros::ok()){
    hbeat_cnt++;

    if(hbeat_cnt > hbeat_timeout){
        write(fd, &stop, 1); // Write the stop drill command
        hbeat = true;
    }
    ros::spinOnce();
    loop_rate.sleep();
}
  while(false){

  std::cout << "Give input: ";
  std::getline(std::cin, inputString);

  int inputInt;
  /*Check if input is a number and convert it*/
  if (isNumber(inputString)) {
    inputInt = std::stoi(inputString);
  }
  else {
    inputInt = 4; // Stops drill if not a number
  }

  std::cout << inputString << " Sent";
  write(fd, &inputInt, 1);

  
  if(inputString == "exit"){
     break; //Break and close the file
  }
  }
  std::cout << "end";
  close(fd);
  return 0;
}


bool isNumber(std::string str) {
  // Make sure string isnt empty
  if (str.empty()) {
    return false;
  }

  // Make sure string doesnt have non digits
  for (char character : str) {
    if (!std::isdigit(character)) {
      return false;
    }
  }

  return true;
}
