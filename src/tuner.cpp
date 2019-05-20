//--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
// File: tuner.cpp
// Author: David Ting
// 
//
// Description:
//  Simple command based PID tuner to send PID commands over can to talon SRX's.
//
//--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--

#include "ros/ros.h"
#include <ros/console.h>
#include <nova_common/DriveCmd.h>
#include <nova_common/TuneCmd.h>
//--**-- Standard includes
#include <stdlib.h>
#include <signal.h>
#include <iostream>
#include <math.h>
#include <string>

//--*-- Talon SRX includes
#define Phoenix_No_WPI // remove WPI dependencies
#include "ctre/Phoenix.h"
#include "ctre/phoenix/platform/Platform.h"
#include "ctre/phoenix/unmanaged/Unmanaged.h"
#include "ctre/phoenix/MotorControl/CAN/WPI_TalonSRX.h"
#include <string>
#include <iostream>
#include <chrono>
#include <thread>
#include <SDL2/SDL.h>
#include <unistd.h>

using namespace std;
/*TalonSRX talon1(1); 
TalonSRX talon2(2);
TalonSRX talon3(3);
TalonSRX talon4(4);
TalonSRX talon5(5);
TalonSRX talon0(0);

void ConfigTalon(TalonSRX* talon, double num); */

std::string inputString;
std::string inputMode;

float p;
float i;
float d;
float val;
std::string val_str = "";
int main(int argc, char *argv[])
{
 
  //int i,j;
/*  for(j=0; j<10; j++) {
    for(i=0; i<20; i++) {
      printf("%f\t",sinf(i * 1 / 10.0 + j*1/10));
    }
    printf("\n");
  } */
 
  //std::string interface;
  //interface = "can0";
  //ctre::phoenix::platform::can::SetCANInterface(interface.c_str());
  ros::init(argc, argv, "tuner");
  ros::NodeHandle n;
  ros::Publisher tune_pub = n.advertise<nova_common::TuneCmd>("/core_rover/PID_tune", 1);
  while(true){
  val_str = std::to_string(val);
  std::cout << "Give input "+ inputMode + "=" + val_str + ": ";
  //std::cout << "Give input" + inputMode + "=" + val;
  std::getline(std::cin, inputString);
  if(inputString == "p" or inputString == "i" or inputString == "d"){
  inputMode = inputString;
  if (inputString == "p") val = p;
  if (inputString == "i") val = i;
  if (inputString == "d") val = d;
}
  else{
  double value = atof(inputString.c_str());
  nova_common::TuneCmd msg;
  msg.coefficient = value;
  msg.constant = inputMode;
  tune_pub.publish(msg);
  val = value;
  if (inputMode == "p") p = val;
  if (inputMode == "i") i = val;
  if (inputMode == "d") d = val;

}

  }
}

/*
void ConfigTalon(TalonSRX* talon,double value) {

	const int kTimeoutMs = 0;
	const int kPIDLoopIdx = 0;
	
        /* first choose the sensor 
	talon->ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder, 0, kTimeoutMs);
	talon->SetSensorPhase(false);
	if(inputMode=="p"){
                p = value;
                val = p;
		talon->Config_kP(kPIDLoopIdx, value, kTimeoutMs); //0.22
}
	else if(inputMode=="i"){
                i = value;
                val = i;
		talon->Config_kI(kPIDLoopIdx, value, kTimeoutMs); //0.22
}
	else if(inputMode=="d"){
                d = value;
                val = d;
		talon->Config_kD(kPIDLoopIdx, value, kTimeoutMs); //0.22
}
	 set the peak and nominal outputs 
}*/
