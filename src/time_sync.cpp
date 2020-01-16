#include "ros/ros.h"

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sensor_msgs/CompressedImage.h>
#include <self_driving_rc_car/RcCarTeleop.h>

#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

using namespace message_filters;

ros::Publisher img_pub;
ros::Publisher cmd_pub;

cv::Mat resized_img;
sensor_msgs::ImagePtr resized_img_msg;
image_transport::Publisher resized_img_pub;


void callback(const self_driving_rc_car::RcCarTeleop::ConstPtr& teleop_msg, 
              const sensor_msgs::Image::ConstPtr& img_msg) {

    cv::Mat img_received = cv_bridge::toCvShare(img_msg, "bgr8")->image;
    cv::Size size = img_received.size();

    cv::resize(img_received,resized_img,cv::Size(), 0.5,0.5, cv::INTER_AREA);
    cv::Size resized = resized_img.size();

    std_msgs::Header header; // empty header
    header.stamp = teleop_msg->header.stamp; // time
    resized_img_msg = cv_bridge::CvImage(header, "bgr8", resized_img).toImageMsg();    
    resized_img_pub.publish(resized_img_msg);

    cmd_pub.publish(teleop_msg);
}

int main(int argc, char** argv) {

    ros::init(argc, argv, "combinedNode");
    ros::NodeHandle nh;
    
    message_filters::Subscriber<self_driving_rc_car::RcCarTeleop> f_sub(nh, "rc_car/teleop_cmd", 1);
    message_filters::Subscriber<sensor_msgs::Image> s_sub(nh, "image_color", 1);

    typedef sync_policies::ApproximateTime<self_driving_rc_car::RcCarTeleop, sensor_msgs::Image> MySyncPolicy;

    Synchronizer<MySyncPolicy> sync(MySyncPolicy(30), f_sub, s_sub);
    sync.registerCallback(boost::bind(&callback, _1, _2));

    image_transport::ImageTransport it(nh);
    resized_img_pub = it.advertise("sync/resized_image", 1); 

    //img_pub = nh.advertise<sensor_msgs::CompressedImage>("/sync/image_color",1);
    cmd_pub = nh.advertise<self_driving_rc_car::RcCarTeleop>("/sync/teleop_cmd",1);

    ros::spin();    
    
    return 0;
}