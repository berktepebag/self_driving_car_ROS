#include "ros/ros.h"
#include <iostream>
#include "sensor_msgs/Imu.h"
#include <math.h>

#include "Eigen/Dense"
#include "extended_kalman_filter/measurement_package.h"
#include "extended_kalman_filter/tracking.h"

#include "self_driving_rc_car/WheelEncoder.h"
#include "self_driving_rc_car/RcCarTeleop.h"

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

class SensorClass
{
    private:

        ros::NodeHandle nh;

        ros::Time current_time, prev_time;
        ros::Duration delta_time; float dt;

        ros::Subscriber imu_sub;

        float ax,ay,rot_z;

        float wheelEncoderReading, steeringAngle, psi_dot, vel_x, vel_y;
        float str_ang_multiplier = deg2rad(35.0);

        int direction = 1;
        float lf = 0.35;

        Tracking tracking;
        Eigen::VectorXd imu_meas = Eigen::VectorXd(3);
        Eigen::VectorXd encoder_meas = Eigen::VectorXd(3);   

        std::string sensorType;     

        message_filters::Subscriber<self_driving_rc_car::RcCarTeleop> teleop_sub;
        message_filters::Subscriber<self_driving_rc_car::WheelEncoder> encoder_sub;

        typedef message_filters::sync_policies::ApproximateTime<self_driving_rc_car::RcCarTeleop, self_driving_rc_car::WheelEncoder> OdomSyncPolicy;
        typedef message_filters::Synchronizer<OdomSyncPolicy> Sync;
        boost::shared_ptr<Sync> sync;     

        bool DEBUG = false;

   
    public:
        SensorClass()
        {
            imu_sub = nh.subscribe("imu/data_filtered",10, &SensorClass::imuCallback, this);
           
            current_time = ros::Time::now();
            prev_time = current_time;
            
            // Sync steering and wheel encoder topics to calculate position with bicycle model
            teleop_sub.subscribe(nh, "/rc_car/teleop_cmd", 1);
            encoder_sub.subscribe(nh, "/rc_car/wheel_encoder", 1);

            sync.reset(new Sync(OdomSyncPolicy(30), teleop_sub,encoder_sub));
            sync->registerCallback(boost::bind(&SensorClass::odomCallback, this, _1, _2));   
        }

        ~SensorClass(){}

        void odomCallback(const self_driving_rc_car::RcCarTeleop::ConstPtr& teleop_msg, const self_driving_rc_car::WheelEncoder::ConstPtr& encoder_msg)
        { 
            sensorType = "WHEEL_ENCODER";
            if(DEBUG) ROS_INFO("wheel encoder call back");

            //Moving diretion according to the joystick input:            
            if(teleop_msg->forward < 0.95){direction = 1;}// if(teleop_msg->forward < 1.0 || (teleop_msg->forward < teleop_msg->reverse))
            else if(teleop_msg->reverse < 0.95){direction = -1;} // else if(teleop_msg->reverse < 1.0 || (teleop_msg->reverse < teleop_msg->forward)) 

            wheelEncoderReading = encoder_msg->velocity * direction;
            if (fabs(wheelEncoderReading) < 0.01){wheelEncoderReading = 0.0;} // Due to sensivity of the hall effect reader, sometimes velocity stucks at > 0.0 m/s even if the car is at complete stop. Prevent this by applying a low filter.

            float current_psi = tracking.ekf_.x_[2]; // Last calculated psi angle from KF
            float vel_x = wheelEncoderReading * cos(current_psi);
            float vel_y = wheelEncoderReading * sin(current_psi);

            steeringAngle = teleop_msg->servo; //Joystick returns -1 for CW, +1 for CCW. 
            psi_dot = wheelEncoderReading / lf * (steeringAngle*str_ang_multiplier); // rad/s

            // Send wheel encoder readings to kalman filter.
            encoder_meas << vel_x, vel_y, psi_dot; 
            tracking.ProcessMeasurement(sensorType, encoder_meas);   

            if(DEBUG) ROS_INFO("WHEEL ENCODER-> v_x: %.2f v_y: %.2f v_psi: %.4f", vel_x,vel_y,psi_dot);

        }

        void imuCallback(const sensor_msgs::ImuConstPtr &imu_msg)
        {
            sensorType = "IMU";
            if(DEBUG) ROS_INFO("wheel encoder call back");

            ax = imu_msg->linear_acceleration.x;
            ay = imu_msg->linear_acceleration.y;
            rot_z = imu_msg->angular_velocity.z;
            
            // Send IMU readings to kalman filter.
            imu_meas << ax ,ay, rot_z;
            tracking.ProcessMeasurement(sensorType, imu_meas);

            if(DEBUG) ROS_INFO("IMU ax: %.4f ay: %.4f rot_z: %.4f", ax, ay, rot_z);

        }

        float deg2rad(float deg)
        {
            return deg * M_PI / 180;
        }
};



int main(int argc, char **argv)
{

    ros::init(argc,argv,"ext_kf");    

    SensorClass SensorClass;
    
    ros::spin();

    return 0;
}