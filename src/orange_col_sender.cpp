#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/UInt32MultiArray.h"

#include <opencv2/imgproc.hpp>  // for cvtColor
#include <opencv2/opencv.hpp>   // for inRange

class OrangeColSender
{
  ros::NodeHandle nh_{};
  ros::NodeHandle pnh_{"~"};
  image_transport::ImageTransport it_{nh_};
  image_transport::Subscriber sub_{it_.subscribe("mask_image", 1, &OrangeColSender::send_col, this)};
  ros::Publisher pub_{nh_.advertise<std_msgs::UInt32MultiArray>("orange_col", 1000)};
public:
  OrangeColSender() = default;
  void send_col(const sensor_msgs::ImageConstPtr& msg)
  {
    try {
      cv_bridge::CvImageConstPtr img_ptr{cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8)};
      std_msgs::UInt32MultiArray dest{};
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
