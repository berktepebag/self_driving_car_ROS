#include <iostream>
#include "ros/ros.h"
#include <math.h>

#include "geometry_msgs/Twist.h"
#include "std_msgs/Float32.h"
#include "self_driving_rc_car/RcCarTeleop.h"
#include "self_driving_rc_car/WheelEncoder.h"

#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>


float rad2deg(float rad)
{
    return rad * 180 / M_PI;
}

float deg2rad(float deg)
{
    return deg * M_PI / 180;
}

class RC_Odometry
{
    public:
        float velocity_received, steering_angle, pos_x, pos_y, psi, vel_x, vel_y;
        ros::Time current_time, prev_time;
        ros::Duration delta_time; double dt;
        float lf = 0.11;
        int counter;
        float distance;
    private:

    public:
        RC_Odometry(){
            ROS_INFO_STREAM("Constructor called");
            pos_x = 0;
            pos_y = 0;
            psi = 0;
            ROS_INFO("pos_x: %.2f pos_y: %.2f psi: %.2f  ",pos_x,pos_y,psi);

            counter = 0;
            distance = 0;

        }
        void wheel_encoder_callback(const self_driving_rc_car::WheelEncoder::ConstPtr &wheel_encoder_msg)
        {
            current_time = ros::Time::now();
            delta_time = current_time - prev_time;
            dt = delta_time.toSec();

            // counter = wheel_encoder_msg->counter;
            if (wheel_encoder_msg->velocity > 0.005)
            {
                // distance += wheel_encoder_msg->velocity;
                velocity_received = wheel_encoder_msg->velocity;
                distance += velocity_received * dt;
                calculate_position();
                // ROS_INFO("vel: %.2f steer: %.2f dt: %.4f  ",velocity_received, steering_angle, sec);
            }
            // ROS_INFO("distance: %.2f",distance);
            prev_time = current_time;
        }     
        void steering_angle_callback(const self_driving_rc_car::RcCarTeleop::ConstPtr& teleop_msg)
        {
            steering_angle = teleop_msg->servo;
            // std::cout << "str: " << teleop_msg->servo << std::endl;
        }     

        void calculate_position()
        {
            // double dt = delta_time.toSec();

            psi += velocity_received / lf * (steering_angle*14*M_PI/180.0) * dt;
            double pi2 = 2*M_PI;
            if(fabs(psi)>pi2)
            {
                psi = 0;
            }
            // ROS_INFO("%.2f", psi*180/M_PI);
            // ROS_INFO("vel: %.2f psi: %.2f dt: %.2f",velocity_received, psi, dt);
            pos_x += velocity_received * cos(psi) * dt;
            pos_y += velocity_received * sin(psi) * dt;

            ROS_INFO("vel: %.2f pos_x: %.2f pos_y: %.2f psi: %.2f  ",velocity_received, pos_x,pos_y,rad2deg(psi));
            // ROS_INFO("counter: %d",counter);
        }
};

int main(int argc, char **argv)
{
    ros::init(argc,argv,"odom_listener");

    ros::NodeHandle nh;

    RC_Odometry rc_odometry;

    ros::Subscriber wheel_encoder_sub = nh.subscribe("rc_car/wheel_encoder", 1, &RC_Odometry::wheel_encoder_callback, &rc_odometry);
    ros::Subscriber steering_angle_sub = nh.subscribe("rc_car/teleop_cmd", 1, &RC_Odometry::steering_angle_callback, &rc_odometry);

    ros::spin();  
   
    return 0;
}