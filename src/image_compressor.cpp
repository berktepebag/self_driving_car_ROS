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

class ImageConverter
{
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;

    sensor_msgs::ImagePtr resized_img_msg;
    cv::Mat resized_img;

public:
    ImageConverter() : it_(nh_)
    {

        //Subscribe to jetson raw image topic
        image_sub_ = it_.subscribe("/jetbot_camera/raw", 1, &ImageConverter::imageCb, this);
        image_pub_ = it_.advertise("/rc_car/image_color", 1);
    }

    ~ImageConverter() { ROS_INFO("Image converter destroyed."); }

    void imageCb(const sensor_msgs::ImageConstPtr &img_msg)
    {
        cv_bridge::CvImagePtr cv_ptr;

        try
        {
            cv::Mat img_received = cv_bridge::toCvShare(img_msg, "bgr8")->image;
            cv::Size size = img_received.size();

            cv::resize(img_received,resized_img,cv::Size(), 0.5,0.5, cv::INTER_AREA);
            cv::Size resized = resized_img.size();

            std_msgs::Header header; // empty header
            header.stamp = ros::Time::now(); // time
            resized_img_msg = cv_bridge::CvImage(header, "bgr8", resized_img).toImageMsg();         
        }
        catch (cv_bridge::Exception &e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        image_pub_.publish(resized_img_msg);
    }
};

int main(int argc, char **argv)
{

    ros::init(argc, argv, "raw_to_compressed_image_converter");
    ImageConverter ic;
    ros::spin();
    return 0;
}