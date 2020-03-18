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
bool CALIBRATE_SENSOR_FOR_GRAVITY = false;
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

void calibrate_sensor_for_gravity(bmi160_dev &sensor, int8_t &rslt,std::vector<double> &calibration_values)
{
    std::cout << "Calibrating sensor for gravitational forces" << std::endl;
    uint16_t n_samples = 1024;

    // double [3];

    struct bmi160_sensor_data acc_data;

    double acc_x, acc_y, acc_z;

    for(int i=0; i<n_samples; i++)
    {
        rslt = bmi160_get_sensor_data(BMI160_ACCEL_SEL, &acc_data, NULL, &sensor);
        acc_x += acc_data.x;
        acc_y += acc_data.y;
        acc_z += acc_data.z;
    }

    calibration_values.push_back(acc_x/n_samples);
    calibration_values.push_back(acc_y/n_samples);
    calibration_values.push_back(acc_z/n_samples);


    std::cout << "Calibration completed! Saving to calibration.csv" << std::endl;

    // Saving to csv
    std::ofstream csv_file;
    csv_file.open("calibration.csv");

    for(std::vector<double>::iterator it = calibration_values.begin(); it!=calibration_values.end();it++)
    {
        std::cout << *it << std::endl;        
        csv_file << *it << "\n";
    }

    std::cout << "Saving to calibration.csv completed.." << std::endl;
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

int main(int argc, char **argv){

    ros::init(argc, argv, "imu_publisher");
    ros::NodeHandle nh;

    ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("imu/data_raw",10);

    ros::Rate loop_rate(10);

    int8_t rslt = BMI160_OK;
    bmi160_dev sensor;
    sensor = bmi.initialize(rslt);
    // Set default sensor values
    set_default_sensor_values(&sensor);
    /* Set the sensor configuration */
    rslt = bmi160_set_sens_conf(&sensor);

    if(SELF_TEST){
        rslt = bmi160_perform_self_test(BMI160_ACCEL_ONLY, &sensor);
        /* Utilize the enum BMI160_GYRO_ONLY instead of BMI160_ACCEL_ONLY
        to perform self test for gyro */
        if (rslt == BMI160_OK) {
            std::cout << "\n raw_acc SELF TEST RESULT SUCCESS" << std::endl;
        } 
        else {
            std::cout << "\n raw_acc SELF TEST RESULT FAIL" << std::endl;
        }
    }

    //Calibrate for gravitational forces, read from CSV file or DO NOT CALIBRATE.
    if(READ__FROM_XML == true && CALIBRATE_SENSOR_FOR_GRAVITY == true)
    {
        READ__FROM_XML = false;
    }    
    std::vector<double>  calibration_values = {};
    if(CALIBRATE_SENSOR_FOR_GRAVITY){
        calibrate_sensor_for_gravity(sensor, rslt, calibration_values);
    }
    else if(READ__FROM_XML){
        calibration_values.clear();

        std::fstream csv_file;
        csv_file.open("calibration.csv");        

        std::string line;
        double cal_value;

        while(getline(csv_file, line)){
            double value = atof(line.c_str());
            calibration_values.push_back(value);
        }   
    }
    else
    {
        std::cout << "Proceeding without calibration!" << std::endl;   
        calibration_values = {0,0,0};
  
    }
    
    struct bmi160_sensor_data raw_acc;   
    struct bmi160_sensor_data raw_gyro;

    double avg_x_acc, avg_y_acc, avg_z_acc, avg_x_gyro, avg_y_gyro, avg_z_gyro;
    uint8_t n_noise_samples = 30;

    std::vector<double> mechanical_filters = {150,150,350};
    
    time_t current_time, prev_time;
    bool first_run = true;
    double prev_acc_x, prev_acc_y, prev_acc_z, prev_vel_x, prev_vel_y, prev_vel_z = 0;
    
    int8_t zero_counter;

    while(ros::ok)
    {
        sensor_msgs::Imu imu_msg;

        // Set current time and if it is first run set prev_time equal to the current time.
        time(&current_time);
        if(first_run)
        {
            std::cout << "Running for the first time..." << std::endl;
            prev_time = current_time;
            first_run = false;
        }

        rslt = bmi160_get_sensor_data(BMI160_BOTH_ACCEL_AND_GYRO, &raw_acc, &raw_gyro, &sensor);

        avg_x_acc = raw_acc.x;
        avg_y_acc = raw_acc.y;
        avg_z_acc = raw_acc.z;
        // std::cout << " acc x: " << raw_acc.x << " acc y: " << raw_acc.y << " acc z: " << raw_acc.z << std::endl;

        avg_x_gyro = raw_gyro.x;
        avg_y_gyro = raw_gyro.y;
        avg_z_gyro = raw_gyro.z;
             
        // Convert raw to milli-G and milli-G to m/s^2
        double acc_x = milligTOms2(rawTOmilliG(avg_x_acc));
        double acc_y = milligTOms2(rawTOmilliG(avg_y_acc));
        double acc_z = milligTOms2(rawTOmilliG(avg_z_acc));
        
        // std::cout << std::fixed << std::setprecision(2) << "avg x: " <<  acc_x << " m/s^2, avg_y_acc: " << acc_y << " m/s^2 avg_z_acc: " << acc_z << " m/s^2" << std::endl;
        // avg x: 0.20 m/s^2, avg_ y: -1.01 m/s^2 avg_z_acc: 10.05 m/s^2

        // Convert raw gyro to 
        double gyro_x = deg2Rad(normalizeRawGyro(avg_x_gyro));
        double gyro_y = deg2Rad(normalizeRawGyro(avg_y_gyro));
        double gyro_z = deg2Rad(normalizeRawGyro(avg_z_gyro));
        
        // std::cout << std::fixed << std::setprecision(2) << "gyro x: " <<  gyro_x << " rad/s, gyro y: " << gyro_y << " rad/s gyro z: " << gyro_z << " rad/s" << std::endl;
        // avg x: 0.20 m/s^2, avg_ y: -1.01 m/s^2 avg_z_acc: 10.05 m/s^2

        /* IMU Message Begins
        / Header */
        imu_msg.header.stamp = ros::Time::now();
        // Acceleration
        imu_msg.linear_acceleration.x = acc_x;
        imu_msg.linear_acceleration.y = acc_y;
        imu_msg.linear_acceleration.z = acc_z;
        // Gyro
        imu_msg.angular_velocity.x = gyro_x;
        imu_msg.angular_velocity.y = gyro_y;
        imu_msg.angular_velocity.z = gyro_z;
        /* Finish Message */

        imu_pub.publish(imu_msg);

        ros::spinOnce();

        loop_rate.sleep();
    }

    return 0;
}