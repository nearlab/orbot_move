/*
Last edited by James Bell on 1/8/18

Returns the rotation rate in rad/s for each wheel in a column vector on a mechanum robot
Count wheels from front left, moving clockwise around the robot
Assume rollers are pointing away from the bot.
Reference: http://research.ijcaonline.org/volume113/number3/pxc3901586.pdf
*/
#include "orbot_utils.h"

Orbot::Orbot(){
}
Orbot::Orbot(float x_cm, float y_cm, float length, float width, float radius){
	this->x_cm = x_cm;
	this->y_cm = y_cm;
	this->length = length;
	this->width = width;
	this->radius = radius;
	wheel_locs[0][0] = length/2;
	wheel_locs[1][0] = -width/2;
	wheel_locs[0][1] = length/2;
	wheel_locs[1][1] = width/2;
	wheel_locs[0][2] = -length/2;
	wheel_locs[1][2] = width/2;
	wheel_locs[0][3] = -length/2;
	wheel_locs[1][3] = -width/2;
}
void Orbot::getRotationRates(float* rates, float vx,float vy, float vTheta){
	for(int i=0;i<4;i++){
		float l=sqrt(wheel_locs[0][i]*wheel_locs[0][i]+wheel_locs[1][i]*wheel_locs[1][i]);
		rates[i] = vx/radius;
		if(i == 0 || i == 2){
			rates[i] -= vy/radius;
		}else{
			rates[i] += vy/radius;
		}
		if(i == 0 || i == 3){
			rates[i] -= vTheta/radius/l;
		}else{
			rates[i] += vTheta/radius/l;
		}
	}
}

