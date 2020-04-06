#include "imu_bmi160/bmi160_interface.h"

#include <iostream>
#include <vector>
#include <fstream>
#include <stdlib.h>
#include <iomanip> // for using std::fixed and std::setprecision
#include <ctime>
#include <assert.h>
#include <math.h>

#include "ros/ros.h"
#include "sensor_msgs/Imu.h"

class IMU_BMI160{
    public:
     
    private:
        int8_t rslt;
        bmi160_dev sensor;
        BMI160 bmi;

        std::vector<double> gravity_calibration_values;
        

    public:

    IMU_BMI160()
    {
        ROS_INFO("Initializing IMU_BMI160 class");

        bmi = BMI160();
        rslt = BMI160_OK;
        sensor = bmi.initialize(rslt);

        // Set default sensor values
        set_default_sensor_values(&sensor);
        /* Set the sensor configuration */
        rslt = bmi160_set_sens_conf(&sensor);
           
    }

    double normalizeRawGyro(double raw_gyro)
    {
        return raw_gyro / 16.4; //datasheet page 9, Output Signal Gyroscope, Sensivity RFS2000, 16.4
    }

    double rawTOmilliG(double raw_data)
    {
        double multiplier = 4000 / std::pow(2.0,16); 
        return raw_data*multiplier;
    }

    double milligTOms2(double milliG_data)
    {
        return  milliG_data / 1000 * 9.81;
    }

    double deg2Rad(double degree)
    {
        return degree * M_PI / 180;
    }

    void set_default_sensor_values(bmi160_dev *sensor_)
    {
        ROS_INFO("Setting default sensor values");
        /*https://ae-bst.resource.bosch.com/media/_tech/media/datasheets/BST-BMI160-DS000.pdf
        / page 8
        / Select the Output data rate, range of accelerometer sensor_ */
        sensor_->accel_cfg.odr = BMI160_ACCEL_ODR_1600HZ; // Min/Max Range -2000->2000==4000 mili-G in 16 bits =>  2^16 = 65,535.  
        sensor_->accel_cfg.range = BMI160_ACCEL_RANGE_2G; // Acceleration values between -2G/2G
        sensor_->accel_cfg.bw = BMI160_ACCEL_BW_NORMAL_AVG4;

        /* Select the power mode of accelerometer sensor_ */
        sensor_->accel_cfg.power = BMI160_ACCEL_NORMAL_MODE;

        /* Select the Output data rate, range of Gyroscope sensor_ 
        / Page 9 */
        sensor_->gyro_cfg.odr = BMI160_GYRO_ODR_3200HZ;
        //RFs2000 => 16.4 LSB/degree/s
        sensor_->gyro_cfg.range = BMI160_GYRO_RANGE_2000_DPS;
        sensor_->gyro_cfg.bw = BMI160_GYRO_BW_NORMAL_MODE;

        /* Select the power mode of Gyroscope sensor_ */
        sensor_->gyro_cfg.power = BMI160_GYRO_NORMAL_MODE; 
        ROS_INFO("DEFAULT SENSOR VALUES HAS BEEN SET...");
    }

    void calibrate_sensor_for_gravity(struct bmi160_sensor_data raw_acc)
    {
        ROS_INFO("Calibrating sensor for gravitational forces");
        uint16_t n_samples = 1024;      
        double acc_x, acc_y, acc_z;

        for(int i=0; i<n_samples; i++)
        {
            rslt = bmi160_get_sensor_data(BMI160_ACCEL_SEL, &raw_acc, NULL, &sensor);
            acc_x += raw_acc.x;
            acc_y += raw_acc.y;
            acc_z += raw_acc.z;
            ROS_INFO("acc_x %.2f acc_y %.2f acc_z %.2f",raw_acc.x,raw_acc.y,raw_acc.z);
        }

        gravity_calibration_values.push_back(acc_x/n_samples);
        gravity_calibration_values.push_back(acc_y/n_samples);
        gravity_calibration_values.push_back(acc_z/n_samples);    

        ROS_INFO("GRAVITY CALIBRATION COMPLETED \n x: %.2f y: %.2f z: %.2f!",gravity_calibration_values[0],gravity_calibration_values[1],gravity_calibration_values[2]);
    }

    void publish_raw_imu_data(struct bmi160_sensor_data raw_acc, struct bmi160_sensor_data raw_gyro,ros::Publisher *imu_data_raw_pub)
    {
        double raw_acc_x, raw_acc_y, raw_acc_z, raw_gyro_x, raw_gyro_y, raw_gyro_z;

        sensor_msgs::Imu raw_imu_msg;

        raw_acc_x = raw_acc.x;
        raw_acc_y = raw_acc.y;
        raw_acc_z = raw_acc.z;
        // std::cout << " acc x: " << raw_acc.x << " acc y: " << raw_acc.y << " acc z: " << raw_acc.z << std::endl;

        raw_gyro_x = raw_gyro.x;
        raw_gyro_y = raw_gyro.y;
        raw_gyro_z = raw_gyro.z;
                
        // Convert raw to milli-G and milli-G to m/s^2
        double acc_x = milligTOms2(rawTOmilliG(raw_acc_x));
        double acc_y = milligTOms2(rawTOmilliG(raw_acc_y));
        double acc_z = milligTOms2(rawTOmilliG(raw_acc_z));
        
        std::cout << std::fixed << std::setprecision(2) << "avg x: " <<  acc_x << " m/s^2, raw_acc_y: " << acc_y << " m/s^2 raw_acc_z: " << acc_z << " m/s^2" << std::endl;
        // avg x: 0.20 m/s^2, avg_ y: -1.01 m/s^2 raw_acc_z: 10.05 m/s^2

        // Convert raw gyro to 
        double gyro_x = deg2Rad(normalizeRawGyro(raw_gyro_x));
        double gyro_y = deg2Rad(normalizeRawGyro(raw_gyro_y));
        double gyro_z = deg2Rad(normalizeRawGyro(raw_gyro_z));
        
        // std::cout << std::fixed << std::setprecision(2) << "gyro x: " <<  gyro_x << " rad/s, gyro y: " << gyro_y << " rad/s gyro z: " << gyro_z << " rad/s" << std::endl;
        // avg x: 0.20 m/s^2, avg_ y: -1.01 m/s^2 raw_acc_z: 10.05 m/s^2

        /* IMU Message Begins
        / Header */
        raw_imu_msg.header.stamp = ros::Time::now();
        // Acceleration
        raw_imu_msg.linear_acceleration.x = acc_x;
        raw_imu_msg.linear_acceleration.y = acc_y;
        raw_imu_msg.linear_acceleration.z = acc_z;
        // Gyro
        raw_imu_msg.angular_velocity.x = gyro_x;
        raw_imu_msg.angular_velocity.y = gyro_y;
        raw_imu_msg.angular_velocity.z = gyro_z;
        /* Finish Message */

        imu_data_raw_pub->publish(raw_imu_msg);

    }

    void publish_filtered_imu_data(struct bmi160_sensor_data raw_acc, struct bmi160_sensor_data raw_gyro, ros::Publisher *imu_data_filtered_pub, std::vector<double>  gravity_calibration_values, int &counter, std::vector<double> &average_imu_readings, const std::vector<double> mechanical_filters)
    {
        int lowPassFilterLimit =  30;

        double raw_acc_x, raw_acc_y, raw_acc_z, raw_gyro_x, raw_gyro_y, raw_gyro_z, calibrated_acc_x, calibrated_acc_y, calibrated_acc_z;

        sensor_msgs::Imu filtered_imu_msg;

        raw_acc_x = raw_acc.x;
        raw_acc_y = raw_acc.y;
        raw_acc_z = raw_acc.z;
        // std::cout << " acc x: " << raw_acc.x << " acc y: " << raw_acc.y << " acc z: " << raw_acc.z << std::endl;

        raw_gyro_x = raw_gyro.x;
        raw_gyro_y = raw_gyro.y;
        raw_gyro_z = raw_gyro.z;

        // Low Pass Filter
        if(counter < lowPassFilterLimit)
        {
            average_imu_readings[0] += raw_acc_x - gravity_calibration_values[0];
            average_imu_readings[1] += raw_acc_y - gravity_calibration_values[1];
            average_imu_readings[2] += raw_acc_z - gravity_calibration_values[2];
            counter++;
        }
        else{
            average_imu_readings[0] /= lowPassFilterLimit;
            average_imu_readings[1] /= lowPassFilterLimit;
            average_imu_readings[2] /= lowPassFilterLimit;
            counter=0;

            // Mechanical Filter   
            for(int i=0; i<3; i++)
            {
                if(fabs(average_imu_readings[i]) < mechanical_filters[i])
                {
                    average_imu_readings[i] = 0;
                }
            }               

                    
            // Convert raw to milli-G and milli-G to m/s^2
            double acc_x = milligTOms2(rawTOmilliG(average_imu_readings[0]));
            double acc_y = milligTOms2(rawTOmilliG(average_imu_readings[1]));
            double acc_z = milligTOms2(rawTOmilliG(average_imu_readings[2]));
            
            // std::cout << std::fixed << std::setprecision(2) << "avg x: " <<  acc_x << " m/s^2, raw_acc_y: " << acc_y << " m/s^2 raw_acc_z: " << acc_z << " m/s^2" << std::endl;
            // avg x: 0.20 m/s^2, avg_ y: -1.01 m/s^2 raw_acc_z: 10.05 m/s^2

            // Convert raw gyro to 
            double gyro_x = deg2Rad(normalizeRawGyro(raw_gyro_x));
            double gyro_y = deg2Rad(normalizeRawGyro(raw_gyro_y));
            double gyro_z = deg2Rad(normalizeRawGyro(raw_gyro_z));
            
            // std::cout << std::fixed << std::setprecision(2) << "gyro x: " <<  gyro_x << " rad/s, gyro y: " << gyro_y << " rad/s gyro z: " << gyro_z << " rad/s" << std::endl;
            // avg x: 0.20 m/s^2, avg_ y: -1.01 m/s^2 raw_acc_z: 10.05 m/s^2

            /* IMU Message Begins
            / Header */
            filtered_imu_msg.header.stamp = ros::Time::now();
            // Acceleration
            filtered_imu_msg.linear_acceleration.x = acc_x;
            filtered_imu_msg.linear_acceleration.y = acc_y;
            filtered_imu_msg.linear_acceleration.z = acc_z;
            // Gyro
            filtered_imu_msg.angular_velocity.x = gyro_x;
            filtered_imu_msg.angular_velocity.y = gyro_y;
            filtered_imu_msg.angular_velocity.z = gyro_z;
            /* Finish Message */

            imu_data_filtered_pub->publish(filtered_imu_msg);

            //Set averages to zero
            average_imu_readings[0] = 0;
            average_imu_readings[1] = 0;
            average_imu_readings[2] = 0;
        }
    }    
    
};
