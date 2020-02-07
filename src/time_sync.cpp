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
ros::Publisher sync_cmd_pub;

cv::Mat resized_img;
sensor_msgs::ImagePtr synced_img_msg;
image_transport::Publisher sync_img_pub;


void callback(const self_driving_rc_car::RcCarTeleop::ConstPtr& teleop_msg, 
              const sensor_msgs::CompressedImage::ConstPtr& img_msg) {

    cv::Mat comp_img = cv::imdecode(cv::Mat(img_msg->data),1); // converts compressed image to cv::Mat
    synced_img_msg = cv_bridge::CvImage(img_msg->header,"bgr8",comp_img).toImageMsg();

    sync_img_pub.publish(synced_img_msg);
    sync_cmd_pub.publish(teleop_msg);

    
}

int main(int argc, char** argv) {

    ros::init(argc, argv, "sync_node");
    ros::NodeHandle nh;
    
    message_filters::Subscriber<self_driving_rc_car::RcCarTeleop> f_sub(nh, "rc_car/teleop_cmd", 1);
    message_filters::Subscriber<sensor_msgs::CompressedImage> s_sub(nh, "rc_car/image_color/compressed", 1);
    // message_filters::Subscriber<sensor_msgs::Image> s_sub(nh, "/jetbot_camera/raw", 1);    

    image_transport::ImageTransport it(nh);
    sync_img_pub = it.advertise("sync/resized_image", 1); 

    sync_cmd_pub = nh.advertise<self_driving_rc_car::RcCarTeleop>("/sync/teleop_cmd",1);

    typedef sync_policies::ApproximateTime<self_driving_rc_car::RcCarTeleop, sensor_msgs::CompressedImage> MySyncPolicy;

    Synchronizer<MySyncPolicy> sync(MySyncPolicy(30), f_sub, s_sub);
    sync.registerCallback(boost::bind(&callback, _1, _2));

    ros::spin();    
    
    return 0;
}