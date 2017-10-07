#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Int32MultiArray.h"

#include <opencv2/imgproc.hpp>  // for cvtColor
#include <opencv2/opencv.hpp>   // for inRange

#include <stdio.h>

class OrangeColSender
{
  ros::NodeHandle nh_{};
  ros::NodeHandle pnh_{"~"};
  image_transport::ImageTransport it_{nh_};
  image_transport::Subscriber sub_{it_.subscribe("mask_image", 1, &OrangeColSender::send_col, this)};
  ros::Publisher pub_{nh_.advertise<std_msgs::Int32MultiArray>("orange_col", 1000)};
public:
  OrangeColSender() = default;
  void send_col(const sensor_msgs::ImageConstPtr& msg)
  {
    try {
      cv_bridge::CvImageConstPtr img_ptr{cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8)};
      std_msgs::Int32MultiArray dest{};
      const int begin{0}, square_size{15}, end{img_ptr->image.rows-square_size};
      for (int y{begin}; y < end; ++y)
        for (int x{0}; x < img_ptr->image.cols-square_size; ++x)
          if (cv::mean(cv::Mat{img_ptr->image, cv::Rect{x, y, square_size, square_size}})[0] > 200) dest.data.push_back(x);
      pub_.publish(dest);
    }
    catch (cv_bridge::Exception& e) {
      ROS_ERROR_STREAM("cv_bridge exception: " << e.what());
    } 
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "orange_col_sender");
  OrangeColSender sender {};
  ros::spin();
  return 0;
}
