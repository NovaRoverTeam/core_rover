
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
#include <cstdlib>
#include <time.h>

FILE *fOut;
char serial_buffer[1];
std::string message = "";
int counter = 0;
int line_counter = 0;
int is_v = 0;
int is_eq = 0;
int main(int argc, char *argv[])
{
  time_t now;
  int fd = open("/dev/ttyUSB0", O_RDONLY | O_NOCTTY);
  printf("fd opened as %i\n", fd);
  //fOut = fopen("../battery_data.txt","w"); //resetting
  //fclose(fOut);
  /* wait for the Arduino to reboot */
  usleep(3500000);
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
  fOut = fopen("../battery_data.txt","a");

  if(fOut!=0){
    int n = read(fd, &serial_buffer, sizeof(serial_buffer));
    line_counter++;
    if(serial_buffer[0] == '\n' and line_counter > 5){
    //char equals[] = "= ";
//    fputs(equals, fOut);
    fputs(" ",fOut);
    time(&now);
    fputs(asctime(localtime(&now)), fOut);
    line_counter = 0;

}
    else if(serial_buffer[0] == '\n'){
    continue;

}

    fputs(serial_buffer,fOut);
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





    //puts(serial_buffer);
    fclose(fOut);
}
}
}
