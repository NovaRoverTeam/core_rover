

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

#define DEBUG 0
ros::NodeHandle *nh;
std::string inputString;
int fd;
// Forward declare functions
bool isNumber(std::string str);

void DrillCmdCb(const std_msgs::Int8::ConstPtr& msg)
{
  int data = msg->data;
  ROS_INFO("%d",msg->data);
  ROS_INFO("test");
  write(fd, &data, 1);
}


int main(int argc, char *argv[])
{

  int n, i;

  ros::init(argc,argv, "drill", ros::init_options::AnonymousName);
  nh = new ros::NodeHandle;
  ros::Subscriber drill_cmd_sub = nh->subscribe("/base_station/drill_cmd", 1, DrillCmdCb);


  

  char buf[64] = "temp text";
  struct termios toptions;

  /* open serial port */
  fd = open("/dev/ttyACM0", O_RDWR | O_NOCTTY);
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

  /* Send byte to trigger Arduino to send string back */
  ROS_INFO("cat");
//  write(fd, "0", 1);
//  write(fd, "1", 1);
  /* Receive string from Arduino */
  //n = read(fd, buf, 64);
  /* insert terminating zero in the string */
  //buf[n] = 0;

  //printf("%i bytes read, buffer contains: %s\n", n, buf);
 
  if(DEBUG)
    {
      printf("Printing individual characters in buf as integers...\n\n");
      for(i=0; i<n; i++)
	{
	  printf("Byte %i:%i, ",i+1, (int)buf[i]);
	}
      printf("\n");
    }

  while (ros::ok()){
    ros::spinOnce();
}
  while(false){

  std::cout << "Give input: ";
  std::getline(std::cin, inputString);

  int inputInt;
  if (isNumber(inputString)) {
    inputInt = std::stoi(inputString);
  }
  else {
    inputInt = 0;
  }

  //std::cin >> inputString;
  std::cout << inputString;
  ROS_INFO("Test");

	ROS_INFO("dog");
	write(fd, &inputInt, 1);

  
  if(inputString == "exit"){
     break;
  }
  }
  printf("end");
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
