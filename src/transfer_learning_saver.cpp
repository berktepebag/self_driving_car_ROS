#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    //cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
    //cv::waitKey(30);
    cv::Mat img_received = cv_bridge::toCvShare(msg, "bgr16")->image;
    cv::Size size = img_received.size();
    ROS_INFO("received img size: %dx%d", size.width,size.height);

    // cv::Mat gray_img;
    // cv::cvtColor(img_received, gray_img, CV_BGR2GRAY);
    // cv::imwrite("../imgs/",gray_img);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;
  //cv::namedWindow("view");
  //cv::startWindowThread();
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("/jetbot_camera/raw", 1, imageCallback);
  ros::spin();
  //cv::destroyWindow("view");
}
