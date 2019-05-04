#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float64.h>


#include "joystick_controller.h"

void JoystickTeleop::joyCallback(const sensor_msgs::Joy::ConstPtr &joy){

  //LT:joy->axes[2]
  //RT:joy->axes[5]
  //Left Stick Horizontal: joy->axes[0]
  //Left Stick Vertical: joy->axes[1]
  //Right Stick Horizontal: joy->axes[3]
  //Right Stick Vertical: joy->axes[4]

  teleop_msg.forward = joy->axes[5];
  teleop_msg.reverse = joy->axes[2];
  teleop_msg.servo = joy->axes[0];
}

void JoystickTeleop::joyCommandPublish(){

  teleop_msg.header.frame_id = "frame_id";
  teleop_msg.header.stamp = ros::Time::now();

  teleop_cmd_pub.publish(teleop_msg);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_rc_car");

  JoystickTeleop joy_teleop;

  ros::Rate rate(30);
  while(ros::ok())
  {
    joy_teleop.joyCommandPublish();

    ros::spinOnce();
    rate.sleep();
  }  

}