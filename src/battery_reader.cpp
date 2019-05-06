
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdint.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <signal.h>
#include <iostream>
#include <fstream>
#include <cstdlib>
#include <time.h>

FILE *fOut;
char serial_buffer[1];
std::string message = "";
int counter = 0;
int line_counter = 0;
int is_v = 0;
int is_eq = 0;
int minute_count = 0;
std::string minute_buffer = "";
std::string string_buffer;
std::ofstream out;

int main(int argc, char *argv[])
{
  time_t now;
  int fd = open("/dev/ttyUSB0", O_RDONLY | O_NOCTTY);
  //int fd = 0;
  printf("fd opened as %i\n", fd);
  now = time(NULL);

  std::string time_now = "";// asctime(localtime(&now));
  std::cout << time_now;
  for(int i=0; i<time_now.length(); i++){
     if(time_now[i] == ' ' or time_now[i] == '\n' or time_now[i] =='\r' or time_now[i] == ':')
     {
        time_now.erase(i,1);
     }
}
  std::string location = time_now;
  printf("1");
  std::cout << location;
  printf("testing");
  std::string s = "/home/nvidia/catkin_ws/src/core_rover/battery_data.txt";
  std::cout << s;
  char cstr[s.size()+1];
  strcpy(cstr, s.c_str());
  printf("fd opened as %i\n", fd);
//  fOut = fopen(cstr,"w"); //resetting
//  fclose(fOut);
  /* wait for the Arduino to reboot */
  //usleep(3500000);
  struct termios toptions;
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
  while(true){
  //std::cout<<fOut;

  //if(fOut!=0){
  if(true){
    int n = read(fd, &serial_buffer, sizeof(serial_buffer));
    line_counter++;
    string_buffer = serial_buffer[0];
    if(serial_buffer[0] == '\n' and line_counter > 5){
    //char equals[] = "= ";
//    fputs(equals, fOut);
    minute_count++;
    //minute_buffer += " ";
    time(&now);
    time_now = asctime(localtime(&now));
    minute_buffer += time_now;
//    fputs(" ",fOut);

    //minute_buffer += time_now;
    if (minute_count > 5){
       std::cout << minute_buffer;
       out.open(s, std::ios::app);
       minute_count = 0;
       out << minute_buffer;
       minute_buffer = "";
       out.close(); 
       puts("TeST!");
       
}
//    fputs(asctime(localtime(&now)), fOut);
    line_counter = 0;

}
else if(serial_buffer[0] == '\n'){
continue;

}

//    fputs(serial_buffer,fOut);
    minute_buffer += string_buffer;
    char test = 'r';
    double volts;
    counter++;
    if(serial_buffer[0]=='V' && is_v!=1){
    is_v = 1;
}

if(is_eq==1){
message+=serial_buffer[0];
}
if(serial_buffer[0] == '=' && is_eq!=1){
is_eq = 1;
}

if(serial_buffer[0] == ' ' && is_eq){
volts=std::stod(message);
printf("%lf",volts);
message = "";
is_eq = 0;
is_v = 0;
}





    puts(serial_buffer);
    //fclose(fOut);
}
}
}
