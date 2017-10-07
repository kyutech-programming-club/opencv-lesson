#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Int32MultiArray.h"

#include <opencv2/imgproc.hpp>  // for cvtColor
#include <opencv2/opencv.hpp>   // for inRange

static constexpr int default_rows{480};

class OrangeColSender
{
public:
  OrangeColSender();
  void send_col(const sensor_msgs::ImageConstPtr& msg);
private:
  ros::NodeHandle nh_{};
  ros::NodeHandle pnh_{"~"};
  image_transport::ImageTransport it_{nh_};
  image_transport::Subscriber sub_{it_.subscribe("mask_image", 1, &OrangeColSender::send_col, this)};
  ros::Publisher pub_{nh_.advertise<std_msgs::Int32MultiArray>("orange_col", 1000)};
  int begin_, square_size_, end_;
  int threshold_;
  int row_stride_, col_stride_;
};

OrangeColSender::OrangeColSender()
  : begin_{0}, square_size_{15}, end_{default_rows-square_size_},
    threshold_{200},
    row_stride_{1}, col_stride_{1}
{
}

void OrangeColSender::send_col(const sensor_msgs::ImageConstPtr& msg)
{
  try {
    cv_bridge::CvImageConstPtr img_ptr{cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8)};
    if (img_ptr->image.rows != default_rows) throw img_ptr->image.rows;
    std_msgs::Int32MultiArray dest{};
    for (int y{begin_}; y < end_; y+=row_stride_)
      for (int x{0}; x < img_ptr->image.cols-square_size_; x+=col_stride_)
        if (cv::mean(cv::Mat{img_ptr->image, cv::Rect{x, y, square_size_, square_size_}})[0] > threshold_)
          dest.data.push_back(x-img_ptr->image.rows*0.5);
    pub_.publish(dest);
  }
  catch (int rows) { ROS_ERROR_STREAM("ATTENTION: Image's \"rows\" is " << rows << ", not " << default_rows); }
  catch (cv_bridge::Exception& e) { ROS_ERROR_STREAM("cv_bridge exception: " << e.what()); } 
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "orange_col_sender");
  OrangeColSender sender {};
  ros::spin();
  return 0;
}
