#ifndef JOYSTICK_CONTROLLER_H_
#define JOYSTICK_CONTROLLER_H_

#include <iostream>
#include "self_driving_rc_car/RcCarTeleop.h"

class JoystickTeleop{
private:
	ros::NodeHandle nh;

	ros::Publisher teleop_cmd_pub;
	ros::Subscriber joy_sub;

public:
	JoystickTeleop();
	void joyCallback(const sensor_msgs::Joy::ConstPtr &joy);
	void joyCommandPublish();

	double forward, inverse, servo;

	std_msgs::Float64 fwr_msg;
	self_driving_rc_car::RcCarTeleop teleop_msg;

};

JoystickTeleop::JoystickTeleop(){
	teleop_cmd_pub = nh.advertise<self_driving_rc_car::RcCarTeleop>("rc_car/teleop_cmd",1);
	joy_sub = nh.subscribe<sensor_msgs::Joy>("joy",10, &JoystickTeleop::joyCallback,this);
}


#endif