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
TalonSRX talon1(1); 
TalonSRX talon2(2);
TalonSRX talon3(3);
TalonSRX talon4(4);
TalonSRX talon5(5);
TalonSRX talon0(0);

void ConfigTalon(TalonSRX* talon, double num);

std::string inputString;
std::string inputMode;

float p;
float i;
float d;
std::string val = "";

int main(int argc, char *argv[])
{
 
  int i,j;
  for(j=0; j<10; j++) {
    for(i=0; i<20; i++) {
      printf("%f\t",sinf(i * 1 / 10.0 + j*1/10));
    }
    printf("\n");
  }
 
  std::string interface;
  interface = "can0";
  ctre::phoenix::platform::can::SetCANInterface(interface.c_str());
  while(true){

  std::cout << "Give input "+inputMode + "=" + val + ": ";
  //std::cout << "Give input" + inputMode + "=" + val;
  std::getline(std::cin, inputString);
  if(inputString == "p" or inputString == "i" or inputString == "d"){
	inputMode = inputString;
	if (inputString == "p") val = std::to_string(p);
	if (inputString == "i") val = std::to_string(i);
	if (inputString == "d") val = std::to_string(d);
     
}
  else{
	double value = atof(inputString.c_str());
	ConfigTalon(&talon5,value);
	ConfigTalon(&talon4,value);
	ConfigTalon(&talon3,value);
	ConfigTalon(&talon2,value);
	ConfigTalon(&talon1,value);
	ConfigTalon(&talon0,value);

}

  }
}


void ConfigTalon(TalonSRX* talon,double value) {

	const int kTimeoutMs = 0;
	const int kPIDLoopIdx = 0;
	
        /* first choose the sensor */
	talon->ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder, 0, kTimeoutMs);
	talon->SetSensorPhase(false);
	if(inputMode=="p"){
                p = value;
		talon->Config_kP(kPIDLoopIdx, value, kTimeoutMs); //0.22
}
	else if(inputMode=="i"){
                i = value;
		talon->Config_kI(kPIDLoopIdx, value, kTimeoutMs); //0.22
}
	else if(inputMode=="d"){
                d = value;
		talon->Config_kD(kPIDLoopIdx, value, kTimeoutMs); //0.22
}
	/* set the peak and nominal outputs */
}
