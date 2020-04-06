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

bool SELF_TEST = false;
bool CALIBRATE_SENSOR_FOR_GRAVITY = true;
bool READ__FROM_XML = false;

BMI160 bmi = BMI160();

void set_default_sensor_values(bmi160_dev *sensor)
{
    /*https://ae-bst.resource.bosch.com/media/_tech/media/datasheets/BST-BMI160-DS000.pdf
    / page 8
    / Select the Output data rate, range of accelerometer sensor */
    sensor->accel_cfg.odr = BMI160_ACCEL_ODR_1600HZ; // Min/Max Range -2000->2000==4000 mili-G in 16 bits =>  2^16 = 65,535.  
    sensor->accel_cfg.range = BMI160_ACCEL_RANGE_2G; // Acceleration values between -2G/2G
    sensor->accel_cfg.bw = BMI160_ACCEL_BW_NORMAL_AVG4;

    /* Select the power mode of accelerometer sensor */
    sensor->accel_cfg.power = BMI160_ACCEL_NORMAL_MODE;

    /* Select the Output data rate, range of Gyroscope sensor 
    / Page 9 */
    sensor->gyro_cfg.odr = BMI160_GYRO_ODR_3200HZ;
    //RFs2000 => 16.4 LSB/degree/s
    sensor->gyro_cfg.range = BMI160_GYRO_RANGE_2000_DPS;
    sensor->gyro_cfg.bw = BMI160_GYRO_BW_NORMAL_MODE;

    /* Select the power mode of Gyroscope sensor */
    sensor->gyro_cfg.power = BMI160_GYRO_NORMAL_MODE; 
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

double normalizeRawGyro(double raw_gyro)
{
    return raw_gyro / 16.4; //datasheet page 9, Output Signal Gyroscope, Sensivity RFS2000, 16.4
}

double deg2Rad(double degree)
{
    return degree * M_PI / 180;
}

void calibrate_sensor_for_gravity(bmi160_dev &sensor, int8_t &rslt,std::vector<double> &gravity_calibration_values)
{
    ROS_INFO("Calibrating sensor for gravitational forces");
    uint16_t n_samples = 1024;

    struct bmi160_sensor_data acc_data;

    double acc_x, acc_y, acc_z;

    for(int i=0; i<n_samples; i++)
    {
        rslt = bmi160_get_sensor_data(BMI160_ACCEL_SEL, &acc_data, NULL, &sensor);
        acc_x += acc_data.x;
        acc_y += acc_data.y;
        acc_z += acc_data.z;
    }

    gravity_calibration_values.push_back(acc_x/n_samples);
    gravity_calibration_values.push_back(acc_y/n_samples);
    gravity_calibration_values.push_back(acc_z/n_samples);    

    ROS_INFO("CALIBRATION COMPLETED!");
}

void publish_raw_imu_data(struct bmi160_sensor_data raw_acc, struct bmi160_sensor_data raw_gyro, ros::Publisher *imu_data_raw_pub)
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
    uint8_t lowPassFilterLimitCount =  10;

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
    if(counter < lowPassFilterLimitCount)
    {
        average_imu_readings[0] += raw_acc_x - gravity_calibration_values[0];
        average_imu_readings[1] += raw_acc_y - gravity_calibration_values[1];
        average_imu_readings[2] += raw_acc_z - gravity_calibration_values[2];
        counter++;
    }
    else{
        average_imu_readings[0] /= lowPassFilterLimitCount;
        average_imu_readings[1] /= lowPassFilterLimitCount;
        average_imu_readings[2] /= lowPassFilterLimitCount;
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

int main(int argc, char **argv){

    ros::init(argc, argv, "imu_publisher");
    ros::NodeHandle nh;

    ros::Publisher imu_data_raw_pub = nh.advertise<sensor_msgs::Imu>("imu/data_raw",10);
    ros::Publisher imu_data_filtered_pub = nh.advertise<sensor_msgs::Imu>("imu/data_filtered",10);

    ros::Rate loop_rate(500);

    int8_t rslt = BMI160_OK;
    bmi160_dev sensor;
    sensor = bmi.initialize(rslt);
    // Set default sensor values
    set_default_sensor_values(&sensor);
    /* Set the sensor configuration */
    rslt = bmi160_set_sens_conf(&sensor);

    struct bmi160_sensor_data raw_acc;   
    struct bmi160_sensor_data raw_gyro;


    //Calibrate for gravitational forces, read from CSV file or DO NOT CALIBRATE.
    std::vector<double>  gravity_calibration_values = {};
    if(CALIBRATE_SENSOR_FOR_GRAVITY){
        ROS_INFO("Put the vehicle on steady state for gravity calibration...");       
        calibrate_sensor_for_gravity(sensor, rslt, gravity_calibration_values);
    }
    else{
        std::cout << "Proceeding without calibration!" << std::endl;   
        gravity_calibration_values = {0,0,0};  
    }    

    std::vector<double> mechanical_filters = {250,250,800};
    
    time_t current_time, prev_time;
    bool first_run = true;
    double prev_acc_x, prev_acc_y, prev_acc_z, prev_vel_x, prev_vel_y, prev_vel_z = 0;
    
    int counter = 0;
    std::vector<double> average_imu_readings(3);

    while(ros::ok)
    {
        // Set current time and if it is first run set prev_time equal to the current time.
        time(&current_time);
        if(first_run)
        {
            std::cout << "Running for the first time..." << std::endl;
            prev_time = current_time;
            first_run = false;
        }

        rslt = bmi160_get_sensor_data(BMI160_BOTH_ACCEL_AND_GYRO, &raw_acc, &raw_gyro, &sensor);

        publish_raw_imu_data(raw_acc, raw_gyro, &imu_data_raw_pub);

        publish_filtered_imu_data(raw_acc,raw_gyro, &imu_data_filtered_pub, gravity_calibration_values, counter, average_imu_readings, mechanical_filters);

        ros::spinOnce();

        loop_rate.sleep();
    }

    return 0;
}