/*
DESCRIPTION: 

FUNCTIONS:
void 	messageCallbackVicon(geometry_msgs::TransformStamped)
	Subscribes to the orbot object on vicon at /vicon/orbot/orbot and assigns the pose to loc and att
void	messageCallbackTarget(geometry_msgs::TransformStamped)
	Subscribes to the target on /orbot_server/target and assigns the target pose to tarLoc and tarAtt
int 	main(int, char**)
	Starts the subscribers and publisher, publishes delta between target and current pose to the /orbot_server/orbot_delta

NOTES:
	1. 	Could try and ensure rotation is mostly taken care of by the time that the bot reaches within a certain radius of its goal, 
		then fix rotation and translation separately until it converges. This involves giving rotation a superficially large priority based
		on distance to the goal (pretty much a multiplier, just need to ensure it's less than 42/14 times y at all times).
	2.	Could also attempt to have it run regularly until it's less than .5 meters out then go to translation and rotation separately
*/
#include <ros/ros.h>
#include <time.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include "quaternion.h"
#include "kinova_msgs/AddPoseToCartesianTrajectory.h"
#include <iostream>
#include <unistd.h>
#include <cstdlib>
#include <math.h>
#include <stdio.h>
#include <SerialPort.h>//to install, use sudo apt-get install libserial-dev
#include "orbot_utils.h"

ros::Subscriber velcmd;
bool update;

const float pi=3.14159265359;
const float wMax=122*2*pi/60;
const float vMax=.0762*wMax/1.424;// m/s
float rates[4];





void velocityCallback(geometry_msgs::Twist twist)
{
	getRotationRates(rates,twist.linear.x,twist.linear.y,twist.angular.z);
	std::cout<<twist.linear.x<<" "<<twist.linear.y<<" "<<twist.angular.z<<std::endl;
}


void writeToPort(SerialPort *ser, char* write){
	bool written=false;
	while(!written){
		try{
			ser->Write(write);
			written=true;
		}
		catch(std::exception& e){
			continue;
		}
	}
}
void writeDeltas(SerialPort *ser1, SerialPort *ser2, float *rates){
		
		//TODO: make this more straightforward
		char output_buffer[64];
		int len=sprintf(output_buffer,"!g 1 %d\r",(int)rates[0]);
		char* write =(char*) malloc(len+1);
		strncpy(write,output_buffer,len);
		write[len]='\0';
		writeToPort(ser1,write);
		//std::cout<<"wrote "<<write<<"\n";

		len=sprintf(output_buffer,"!g 2 %d\r",(int)rates[1]);
		write =(char*) malloc(len+1);
		strncpy(write,output_buffer,len);
		write[len]='\0';
		writeToPort(ser1,write);
		//std::cout<<"wrote "<<write<<"\n";

		len=sprintf(output_buffer,"!g 1 %d\r",(int)rates[2]);
		write =(char*) malloc(len+1);
		strncpy(write,output_buffer,len);
		write[len]='\0';
		writeToPort(ser2,write);
		//std::cout<<"wrote "<<write<<"\n";

		len=sprintf(output_buffer,"!g 2 %d\r",(int)rates[3]);
		write =(char*) malloc(len+1);
		strncpy(write,output_buffer,len);
		write[len]='\0';
		writeToPort(ser2,write);
		//std::cout<<"wrote "<<write<<"\n";
}

int main(int argc, char** argv){

	ros::init(argc,argv,"orbot_client");
	ros::NodeHandle nh;
	//viconSub=nh.subscribe("/vicon/Orbot/Orbot",1000,messageCallbackVicon);
	velcmd = nh.subscribe("/orbot/cmd_vel",1000,velocityCallback);
	ros::Rate loop_rate(30);//TODO: spiit up the updating and writing rates with time class
	
	
	ros::Time prevWrite=ros::Time::now();
	ros::Time start_time=ros::Time::now();
	ros::Duration writeDelay(.01);
	ros::Duration d;
	
	double t;
	
	SerialPort ser1("/dev/ttyACM0");
	ser1.Open();
	ser1.SetBaudRate(SerialPort::BAUD_115200);
	
	SerialPort ser2("/dev/ttyACM1");
	ser2.Open();
	ser2.SetBaudRate(SerialPort::BAUD_115200);
		

	while(ros::ok()){
		
		bool write=false;
		
		if(write || ros::Time::now()-prevWrite>writeDelay){
			
			write=false;
			writeDeltas(&ser1,&ser2,rates); //Rover Uncomment
			prevWrite=ros::Time::now();
		}
		
//		loop_rate.sleep();
		ros::spinOnce();
		loop_rate.sleep();		
	}
	//ser1.Close(); Rover Uncomment
	//ser2.Close(); Rover Uncomment
	return 0;
}
