
//--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
// File: driver.cpp
// Author: Ben Steer
// Last modified by: Emily Kuo
//
// Description:
//  The driver node receives driving commands from the rover_manager
//  node and interprets these to communicate with the rover's motor
//  controllers to achieve the desired motion. If using the simulator,
//  commands will be given to the virtual rover's controllers instead.
//
//--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--

//--**-- ROS includes
#include "ros/ros.h"
#include <ros/console.h>
#include <nova_common/DriveCmd.h>
#include <nova_common/TuneCmd.h>
#include <nova_common/EncoderData.h>

//..**.. Simulator includes
#include <signal.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <tf/transform_broadcaster.h>

#include <core_rover/set_float.h>
#include <core_rover/set_int.h>

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

#define LOOP_HERTZ 50

using namespace std;

int speed;
int steer;
int num;
bool hbeat = false;
int hbeat_cnt = 0;
double prev_left = 0.0;
double prev_right = 0.0;
double max_delta = 0.02;
std::string tune_cmd = "";
ros::Publisher encoder_data_pub;
float tune_num;
ros::NodeHandle *n; // Create node handle to talk to ROS
const int kTimeoutMs = 0;
const int kPIDLoopIdx = 0;
//Declaring and creating talonSRX objects to control the 6 motors. 
TalonSRX talon1(1); 
TalonSRX talon2(2);
TalonSRX talon3(3);
TalonSRX talon4(4);
TalonSRX talon5(5);
TalonSRX talon0(0);

//Forward Declare functions
void ConfigTalon(TalonSRX* talon);

//--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
// DriveCmdCb():
//    Callback function for receiving driving commands from the
//    rover_manager.
//--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--
void DriveCmdCb(const nova_common::DriveCmd::ConstPtr& msg)
{
  //Instantiiating vars
  double speedFactor  = 2.0;    //currently not in use, but can be used to increase/decrease speed by a factor
  speed = msg->rpm * 1;     //speed = forward or reverse
  steer = msg->steer_pct; //steer = steering percentage left or right
}

//--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
// HbeatCb():
//    Callback function for receiving a heartbeat from the
//    base station if still connected.
//--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..-
void HbeatCb(const std_msgs::Empty::ConstPtr& msg)
{
  hbeat = true;
  hbeat_cnt = 0;
}

//--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
// TuneCb():
//    Callback function for receiving a heartbeat from the
//    base station if still connected.
//--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..-
void TuneCb(const nova_common::TuneCmd::ConstPtr& msg)
{
    
    tune_cmd = msg->constant;
    cout << "TEST"<< msg->coefficient <<"end";
    if (tune_cmd == "p"){
    talon0.Config_kP(kPIDLoopIdx, msg->coefficient, kTimeoutMs);
    talon1.Config_kP(kPIDLoopIdx, msg->coefficient, kTimeoutMs);
    talon2.Config_kP(kPIDLoopIdx, msg->coefficient, kTimeoutMs);
    talon3.Config_kP(kPIDLoopIdx, msg->coefficient, kTimeoutMs);
    talon4.Config_kP(kPIDLoopIdx, msg->coefficient, kTimeoutMs);
    talon5.Config_kP(kPIDLoopIdx, msg->coefficient, kTimeoutMs);
    }
    if (tune_cmd == "i"){
    talon0.Config_kI(kPIDLoopIdx, msg->coefficient, kTimeoutMs);
    talon1.Config_kI(kPIDLoopIdx, msg->coefficient, kTimeoutMs);
    talon2.Config_kI(kPIDLoopIdx, msg->coefficient, kTimeoutMs);
    talon3.Config_kI(kPIDLoopIdx, msg->coefficient, kTimeoutMs);
    talon4.Config_kI(kPIDLoopIdx, msg->coefficient, kTimeoutMs);
    talon5.Config_kI(kPIDLoopIdx, msg->coefficient, kTimeoutMs);
    }
    if (tune_cmd == "d"){
    talon0.Config_kD(kPIDLoopIdx, msg->coefficient, kTimeoutMs);
    talon1.Config_kD(kPIDLoopIdx, msg->coefficient, kTimeoutMs);
    talon2.Config_kD(kPIDLoopIdx, msg->coefficient, kTimeoutMs);
    talon3.Config_kD(kPIDLoopIdx, msg->coefficient, kTimeoutMs);
    talon4.Config_kD(kPIDLoopIdx, msg->coefficient, kTimeoutMs);
    talon5.Config_kD(kPIDLoopIdx, msg->coefficient, kTimeoutMs);
    }
}

//--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
// PIDCb():
//    Change between PID control and voltage control of the motors
//    
//--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..-
void PIDCb(const std_msgs::Bool::ConstPtr& msg)
{
  if(msg->data==true){
    n->setParam("/core_rover/driver/control_mode","PID");
  }
  else{
    n->setParam("/core_rover/driver/control_mode","Voltage");
  }
}

//--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
// EncoderFeedback():
//    Publishes encoder data to rostopic
//    
//--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..-
void EncoderFeedback(){
  nova_common::EncoderData msg;
  msg.talon0Velocity=talon0.GetSelectedSensorVelocity()*0.0818;
  msg.talon1Velocity=talon1.GetSelectedSensorVelocity()*0.0818;
  msg.talon2Velocity=talon2.GetSelectedSensorVelocity()*0.0818;
  msg.talon3Velocity=talon3.GetSelectedSensorVelocity()*0.0818;
  msg.talon4Velocity=talon4.GetSelectedSensorVelocity()*0.0818;
  msg.talon5Velocity=talon5.GetSelectedSensorVelocity()*0.0818;

  msg.talon0Position=talon0.GetSelectedSensorPosition()*0.00818;
  msg.talon1Position=talon1.GetSelectedSensorPosition()*0.00818;
  msg.talon2Position=talon2.GetSelectedSensorPosition()*0.00818;
  msg.talon3Position=talon3.GetSelectedSensorPosition()*0.00818;
  msg.talon4Position=talon4.GetSelectedSensorPosition()*0.00818;
  msg.talon5Position=talon5.GetSelectedSensorPosition()*0.00818;
  
  encoder_data_pub.publish(msg);
  
}

void ConfigTalon(TalonSRX* talon) {
        /* first choose the sensor */
	talon->ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder, 0, kTimeoutMs);
	talon->SetSensorPhase(false);

	/* set the peak and nominal outputs */
	talon->ConfigNominalOutputForward(0, kTimeoutMs);
	talon->ConfigNominalOutputReverse(0, kTimeoutMs);
	talon->ConfigPeakOutputForward(0.7, kTimeoutMs);
	talon->ConfigPeakOutputReverse(-0.7, kTimeoutMs);

	/* set closed loop gains in slot0 */
	talon->Config_kF(kPIDLoopIdx, 0.1097, kTimeoutMs); //0.1097
	talon->Config_kP(kPIDLoopIdx, 2.9, kTimeoutMs); //0.22
	talon->Config_kI(kPIDLoopIdx, 0.03, kTimeoutMs); //0.02
	talon->Config_kD(kPIDLoopIdx, 0.03, kTimeoutMs);
        talon->Config_IntegralZone(kPIDLoopIdx, 10, kTimeoutMs);

	talon->SetSelectedSensorPosition(0);
}

//--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
// main():
//    Main function. This will run first.
//--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--
int main(int argc, char **argv)
{

  //Talon SRX Setup
  std::string interface;
  interface = "can0";
  ctre::phoenix::platform::can::SetCANInterface(interface.c_str());
  talon3.ConfigFactoryDefault();

  talon2.SetSensorPhase(false); //Reverse encoder feedback
  //Set left side inverted 
  talon0.SetInverted(true);
  talon1.SetInverted(true);
  talon2.SetInverted(true);

  //Set all talons to brake when receiving a 0 command
  talon0.SetNeutralMode(Brake);
//  talon1.SetNeutralMode(Brake);
//  talon2.SetNeutralMode(Brake);
//  talon3.SetNeutralMode(Brake);
  talon4.SetNeutralMode(Brake);
//  talon5.SetNeutralMode(Brake);
  
  talon0.Follow(talon1);
  talon3.Follow(talon4);
  
  //Voltage ramp for running on voltage mode
  double delay = 0.0;
  talon0.ConfigOpenloopRamp(delay,0);
  talon1.ConfigOpenloopRamp(delay,0);
  talon2.ConfigOpenloopRamp(delay,0);
  talon3.ConfigOpenloopRamp(delay,0);
  talon4.ConfigOpenloopRamp(delay,0);
  talon5.ConfigOpenloopRamp(delay,0);
  //talon5.ConfigFactoryDefault();

  //Configure talons for pid control
  ConfigTalon(&talon5);
  ConfigTalon(&talon4);
  ConfigTalon(&talon3);
  ConfigTalon(&talon2);
  ConfigTalon(&talon1);
  ConfigTalon(&talon0);

  ros::init(argc, argv, "driver"); // Initialise node
  n = new ros::NodeHandle;
  ros::Rate loop_rate(LOOP_HERTZ); //Set loop rate of ROS, utilised to time heartbeat

  n->setParam("/core_rover/driver/control_mode","PID"); //Sets control mode of talons

  // Declare subscribers to drive cmds
  ros::Subscriber drive_cmd_sub = n->subscribe("/core_rover/driver/drive_cmd", 1, DriveCmdCb);
  ros::Subscriber hbeat_sub = n->subscribe("/heartbeat", 1, HbeatCb);
  ros::Subscriber PID_sub = n->subscribe("/base_station/PID_cmd", 1, PIDCb); //Toggle PID or velocity
  ros::Subscriber PID_tune = n->subscribe("/core_rover/PID_tune", 1, TuneCb); // Tune PID
  encoder_data_pub = n->advertise<nova_common::EncoderData>("/core_rover/encoder_data", 1); //Publishes position and velocity of all encoders
 
  string vehicle;
  bool simulator = false;

  const int hbeat_timeout = 0.5*LOOP_HERTZ; //Set heartbeat timeout amount
  string paramKey = "Vehicle";
  n->getParam(paramKey, vehicle);
  if (vehicle == "Simulator"){
    simulator = true; //If base_station is set up on same device
  }

  double wheel[6]; //array to update motor values

  //Declare service clients for simulator rover stuff
  ros::ServiceClient set_velocity_client;
  core_rover::set_float set_velocity_srv;
  
  int loopCount = 0;
  while (ros::ok()) // Main loop
  {

    if (simulator) //add condition if auto mode
    {
      ROS_INFO("Running Simulator");

      // Use the received drive commands to direct the virtual
      // rover via service calls.

      static const char *motorNames[4] = {
        "front_left_wheel", "front_right_wheel",
        "back_left_wheel", "back_right_wheel"
      };
      
      wheel[0] = speed + steer;
      wheel[1] = speed - steer;
      wheel[2] = speed + steer;
      wheel[3] = speed - steer;

      for (int i=0; i<4; ++i) {
      //updating individual motor values to change speed and or direction
      set_velocity_client = n->serviceClient<core_rover::set_float>(std::string("pioneer3at/") + std::string(motorNames[i]) + std::string("/set_velocity"));
      set_velocity_srv.request.value = wheel[i];
      set_velocity_client.call(set_velocity_srv);
      }
    }
    else
    {

      float talon_speed;
      float talon_steer;
      string mode;
      string paramKey2 = "/core_rover/Mode";
      n->getParam(paramKey2, mode);


	  //If heartbeat detected speed and steer equal published amount, else set speed and steer to 0 until new heartbeat.
      if(hbeat || mode.compare("Auto") == 0){ 
           talon_speed = speed / 50.0;
           talon_steer = steer / 100.0;
      }
      else{
           talon_speed = 0;
           talon_steer = 0;
           std::cout << "No beep boop detected" << std::endl;
	   
      }

      
      float right;
      float left; 
      string param;
      n->getParam("/core_rover/driver/control_mode", param);

      //PID MODE
      if(param == "PID"){
		  talon_speed = talon_speed * 250;
		  talon_steer = talon_steer * 250;
		  right = talon_speed - talon_steer;
		  left = talon_speed + talon_steer;
		  //std::cout << "speed: " << talon1.GetSelectedSensorVelocity() << std::endl;
		  //talon0.Set(ControlMode::Velocity, left);
		  talon1.Set(ControlMode::Velocity, left);
		  talon2.Set(ControlMode::Velocity, left);
		  //RIGHT SIDE
		  //talon3.Set(ControlMode::Velocity, right);
		  talon4.Set(ControlMode::Velocity, right);
		  talon5.Set(ControlMode::Velocity, right);
      }    
      else{
		  right = talon_speed - talon_steer;   //Positive turn decreases right motors speeds to turn right.
		  left = talon_speed + talon_steer;
		  float delta_right = right - prev_right;
		  float delta_left = left - prev_left;
 
		//TALON Protection
		  if (abs(right)<0.02) right=0;  // If value is basically 0, set it off.
		  else{
			  if (abs(right) < abs(prev_right) && abs(delta_right)>max_delta){
				  if (delta_right > 0){
				       right = prev_right + max_delta;
				       delta_right = max_delta;
				  }
				  else{
				       right = prev_right - max_delta;
				       delta_right = max_delta;
				  }
			  }
	  	  }
		  if(abs(left)<0.02){ // If value is basically 0, set it off
		     left = 0;
		  }
		  else{
		  if (abs(left) < abs(prev_left) && abs(delta_left)>max_delta){
		      if (delta_left > 0){
		           left = prev_left + max_delta;
		           delta_left = max_delta;
		      }
		      else{
		           left = prev_left - max_delta;
		           delta_left = max_delta;
		      }
		  }
		  //Setting the previous amount for each side
		  prev_right = right;
		  prev_left = left;
		  }
		  //talon0.Set(ControlMode::PercentOutput, left);
		  talon1.Set(ControlMode::PercentOutput, left);
		  talon2.Set(ControlMode::PercentOutput, left);
		  //talon3.Set(ControlMode::PercentOutput, right);
		  talon4.Set(ControlMode::PercentOutput, right);
		  talon5.Set(ControlMode::PercentOutput, right);
		  }


      
      //Output debug information
      if (loopCount >= 0) {
        loopCount = 0;
      }

      EncoderFeedback(); //Publish Encoder data
      ctre::phoenix::unmanaged::FeedEnable(100); //Enable rover with a timeout of 100ms
    }                     
    loopCount++;

    if(hbeat_cnt > hbeat_timeout){
        hbeat = true;
    }

    hbeat_cnt++;
    ros::spinOnce();   // Messages are received and callbacks called
    loop_rate.sleep();
  }

  return 0;
}

