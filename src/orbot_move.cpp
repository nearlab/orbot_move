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
#include <dlfcn.h>
#include <time.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Header.h>
#include <tf/transform_datatypes.h>
#include "quaternion.h"
#include "kinova_driver/kinova_comm.h"
#include <iostream>
#include <unistd.h>
#include <cstdlib>
#include <math.h>
#include <stdio.h>
#include <SerialPort.h>//to install, use sudo apt-get install libserial-dev
//#include "orbot_utils.h"
#include <actionlib/client/simple_action_client.h>
#include <kinova_msgs/ArmPoseAction.h>
#include <kinova_msgs/HomeArm.h>
#include <kinova_msgs/HomeArmRequest.h>
using namespace kinova;
ros::Subscriber velcmd;
ros::Subscriber armcmd;
bool update;

const float pi=3.14159265359;
const float wMax=122*2*pi/60;
const float vMax=.0762*wMax/1.424;// m/s
float rates[4];
KinovaPose position;
//Orbot orbot;
ros::Time armTime;
ros::Duration armDelay(0.1);

kinova_msgs::ArmPoseGoal goal;

void getRotationRates(float* rates,float* goal_vel, float* real_vel, float x_cm=0, float y_cm=0, float length=.04, float width=.037, float radius=.0076);

void getRotationRates(float* rates, float* goal_vel,float* real_vel, float x_cm, float y_cm, float length, float width, float radius)
{
	const float pi=3.14159265359;
	float wheel_locs[2][4];
	for(uint8_t i=0;i<4;i++){
		wheel_locs[0][i]=((i&2)>0?-length/2:length/2)-x_cm;//hardcoding is for dummies
		wheel_locs[1][i]=(((i&2)>>1)^(i&1)>0?-width/2:width/2)-y_cm;
	}
	float b[] = {0,0,pi,pi};
	for(uint8_t i=0;i<4;i++){
		float a=atan2(wheel_locs[1][i],wheel_locs[0][i]);//about pi/4 or -pi/4
		//float b=(((i&2)>>1)^(i&1))>0?-pi/2:pi/2;
		float c=i&1>0?-pi/4:pi/4;
		float l=sqrt(wheel_locs[0][i]*wheel_locs[0][i]+wheel_locs[1][i]*wheel_locs[1][i]);
		rates[i]=(goal_vel[0] + 0*(goal_vel[0] - real_vel[0]))/radius*cos(b[i]-c)/sin(c);
		rates[i]-=(goal_vel[1] + 0*(goal_vel[1] - real_vel[1]))/radius*sin(b[i]-c)/sin(c);
		rates[i]-=(goal_vel[2] + 0*(goal_vel[2] - real_vel[2]))/radius*l*sin(b[i]-c+a)/sin(c);
	}

}

void velocityCallback(geometry_msgs::Twist twist)
{
	float goal_vel[3];
	goal_vel[0] = twist.linear.x;
	goal_vel[1] = twist.linear.y;
	goal_vel[2] = twist.angular.z;
	getRotationRates(rates,goal_vel,goal_vel);
	//getRotationRates(rates,twist.linear.x,twist.linear.y,twist.angular.z);
	std::cout<<twist.linear.x<<" "<<twist.linear.y<<" "<<twist.angular.z<<std::endl;
}


void armCallback(geometry_msgs::Pose pose)
{
	//ROS_INFO("Received a joy command");
	double r,p,y;
	tf::Quaternion quat;
	std_msgs::Header header;
	header.frame_id = "m1n6s300_link_base";
	
	goal.pose.header = header;
	goal.pose.pose.position.x = pose.position.x;
	goal.pose.pose.position.y = pose.position.y;
	goal.pose.pose.position.z = pose.position.z;
	goal.pose.pose.orientation.x = pose.orientation.x;
	goal.pose.pose.orientation.y = pose.orientation.y;
	goal.pose.pose.orientation.z = pose.orientation.z;
	goal.pose.pose.orientation.w = pose.orientation.w;
	/*
	quaternionMsgToTF(pose.orientation,quat);
	tf::Matrix3x3 m(quat);
	m.getRPY(r,p,y);
	position.ThetaX = r;
	position.ThetaY = p;
	position.ThetaZ = y;
	position.X = pose.position.x;
	position.Y = pose.position.y;
	position.Z = pose.position.z;
	std::cout<<position.ThetaX<<" "<<position.ThetaY<<" "<<position.ThetaZ<<" "<<position.X<<" "<<position.Y<<" "<<position.Z<<std::endl;
	*/
	
	
	// if(ros::Time::now()-armTime>armDelay)
	// {
	// 	actionlib::SimpleActionClient<kinova_msgs::ArmPoseAction> ac("/m1n6s300_driver/pose_action/tool_pose", true);
	// ROS_INFO("Waiting for action server to start.");
	// ac.waitForServer(); //will wait for infinite time
	// 	ROS_INFO("Action server started, sending goal.");
	
	// ac.cancelGoal();
	// ac.sendGoal(goal);
	// armTime = ros::Time::now();
	// bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

 //  if (finished_before_timeout)
 //  {
 //    actionlib::SimpleClientGoalState state = ac.getState();
 //    ROS_INFO("Action finished: %s",state.toString().c_str());
 //  }
 //  else
 //    ROS_INFO("Action did not finish before the time out.");
	// }
	

	
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
/*
void writeToPort(SerialPort *ser, char* write)
{
	bool written=false;
	int count = 0;
	while(!written && count<10){
		count++;
		try{
			ser->Write(write);
			written=true;
			std::cout<<"wrote\n";
		}
		catch(std::exception& e){
			std::cout<<"exception\n"<<count;
			std::cerr<<e.what();
			continue;
		}
		
	}
	if (written==false)
	{
		ser->Close();
		ser->Open();
		std::cout<<"Could not write\n";
	}
}
*/
void writeDeltas(SerialPort *ser1, SerialPort *ser2, float *rates){
		ser1->flush();
		ser2->flush();
		//TODO: make this more straightforward
		char output_buffer[64];
		int len=sprintf(output_buffer,"!g 1 %d\r",(int)rates[0]);
		char* write =(char*) malloc(len+1);
		strncpy(write,output_buffer,len);
		write[len]='\0';
		writeToPort(ser1,write);
		//std::cout<<"wrote "<<write<<"\n";

		ser1->flush();
		len=sprintf(output_buffer,"!g 2 %d\r",(int)rates[1]);
		write =(char*) malloc(len+1);
		strncpy(write,output_buffer,len);
		write[len]='\0';
		writeToPort(ser1,write);
		//std::cout<<"wrote "<<write<<"\n";

		ser2->flush();
		len=sprintf(output_buffer,"!g 1 %d\r",(int)rates[2]);
		write =(char*) malloc(len+1);
		strncpy(write,output_buffer,len);
		write[len]='\0';
		writeToPort(ser2,write);
		//std::cout<<"wrote "<<write<<"\n";

		ser2->flush();
		len=sprintf(output_buffer,"!g 2 %d\r",(int)rates[3]);
		write =(char*) malloc(len+1);
		strncpy(write,output_buffer,len);
		write[len]='\0';
		writeToPort(ser2,write);
		//std::cout<<"wrote "<<write<<"\n";
		delete write;
}

int main(int argc, char** argv){

	ros::init(argc,argv,"orbot_client");
	ros::NodeHandle nh;
	std::string robotName;
        nh.getParam("RobotName",robotName);
	
	velcmd = nh.subscribe("/orbot/cmd_vel",1000,velocityCallback);
	armcmd = nh.subscribe("/orbot/arm_pose",1000,armCallback);
	//quadpose = nh.subscribe("/vicon/Bilbo",1000,viconCallback);
	ros::Rate loop_rate(30);//TODO: spiit up the updating and writing rates with time class

	
	ros::Time prevWrite=ros::Time::now();
	armTime=ros::Time::now();
	ros::Time start_time=ros::Time::now();
	ros::Duration writeDelay(.01);
	ros::Duration d;
	
	double t;
	actionlib::SimpleActionClient<kinova_msgs::ArmPoseAction> ac("/m1n6s300_driver/pose_action/tool_pose", true);
	ROS_INFO("Waiting for action server to start.");
	//ac.waitForServer(); //will wait for infinite time
		ROS_INFO("Action server started, sending goal.");
	// if(homeclient.call())
	// {
	// 	std::cout<<"Arm homed\n";
	// }
	
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
		
		/*if(ros::Time::now()-armTime>armDelay)
	{
		
	
	ac.cancelGoal();
	ac.sendGoal(goal);
	armTime = ros::Time::now();
	bool finished_before_timeout = ac.waitForResult(ros::Duration(3.0));

  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());
  }
  else
    ROS_INFO("Action did not finish before the time out.");
	}*/

//		loop_rate.sleep();
		ros::spinOnce();
		loop_rate.sleep();		
	}
	//ser1.Close(); Rover Uncomment
	//ser2.Close(); Rover Uncomment
	return 0;
}
