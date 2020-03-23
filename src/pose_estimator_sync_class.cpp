#include <iostream>
#include "ros/ros.h"
#include <math.h>

#include "geometry_msgs/Twist.h"
#include "std_msgs/Float32.h"
#include "self_driving_rc_car/RcCarTeleop.h"
#include "self_driving_rc_car/WheelEncoder.h"

#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

using namespace message_filters;

float rad2deg(float rad)
{
    return rad * 180 / M_PI;
}

float deg2rad(float deg)
{
    return deg * M_PI / 180;
}

class OdometryClass
{
    public: //variables
        float wheelEncoderReading, steeringAngle, pos_x, pos_y, psi, vel_x, vel_y;
        ros::Time current_time, prev_time;
        ros::Duration delta_time; double dt;
        float lf = 0.11;
        int counter;
        float distance;

    private:
        ros::NodeHandle nh;
        ros::Publisher odom_pub;

        message_filters::Subscriber<self_driving_rc_car::RcCarTeleop> teleop_sub;
        message_filters::Subscriber<self_driving_rc_car::WheelEncoder> encoder_sub;

        typedef sync_policies::ApproximateTime<self_driving_rc_car::RcCarTeleop, self_driving_rc_car::WheelEncoder> OdomSyncPolicy;
        typedef Synchronizer<OdomSyncPolicy> Sync;
        boost::shared_ptr<Sync> sync;        

    public: //functions
        OdometryClass()
        {         
            // Initialize timers
            current_time = ros::Time::now();
            prev_time = current_time;

            teleop_sub.subscribe(nh, "/rc_car/teleop_cmd", 1);
            encoder_sub.subscribe(nh, "/rc_car/wheel_encoder", 1);

            sync.reset(new Sync(OdomSyncPolicy(30), teleop_sub,encoder_sub));
            sync->registerCallback(boost::bind(&OdometryClass::callback, this, _1, _2));          
        }

        void callback(const self_driving_rc_car::RcCarTeleop::ConstPtr& teleop_msg, const self_driving_rc_car::WheelEncoder::ConstPtr& encoder_msg)
        {          
            current_time = ros::Time::now();
            delta_time = prev_time - current_time;
            dt = delta_time.toSec();

            wheelEncoderReading = encoder_msg->velocity;
            if (wheelEncoderReading < 0.005)
            {
                wheelEncoderReading = 0.0;
            }
            
            steeringAngle = -1 * teleop_msg->servo; //Joystick returns -1 for CW, +1 for CCW, mult. by -1 to normalize axis.

            // ROS_INFO("%.2f", steeringAngle);

            psi += wheelEncoderReading / lf * (steeringAngle*deg2rad(14)) * dt;

            double pi2 = 2*M_PI;
            if(fabs(psi)>pi2)
            {
                psi = 0;
            }
            
            pos_x += wheelEncoderReading * cos(psi) * dt;
            pos_y += wheelEncoderReading * sin(psi) * dt;

            ROS_INFO("vel: %.2f pos_x: %.2f pos_y: %.2f psi: %.2f  ",wheelEncoderReading, pos_x,pos_y,rad2deg(psi));

            prev_time = current_time;
        }
};

int main(int argc, char** argv) 
{
   ros::init(argc,argv,"OdomNode");

   OdometryClass odomClass;

   ros::spin();
}