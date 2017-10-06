#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <dynamic_reconfigure/server.h>
#include <opencv3mixing/ColorThresholdConfig.h>

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp> //inRange用

class ImageConverter
{
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  image_transport::ImageTransport it_;
  image_transport::ImageTransport pit_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  dynamic_reconfigure::Server<opencv3mixing::ColorThresholdConfig> server_;
  dynamic_reconfigure::Server<opencv3mixing::ColorThresholdConfig>::CallbackType f_;
  float  low_h_,  low_s_,  low_v_;
  float high_h_, high_s_, high_v_;
public:
  ImageConverter()
    : nh_ {},
      pnh_ {"~"},
      it_ {nh_},
      pit_ {pnh_},
      image_sub_ {it_.subscribe("input_image", 1, &ImageConverter::imageCb, this)},
      image_pub_ {pit_.advertise("image_raw", 1)},
      server_{},
      f_{boost::bind(&ImageConverter::update_threshold, this, _1, _2)}
  {
    server_.setCallback(f_);
  }
  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    try {
      cv_bridge::CvImagePtr img_ptr{cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)};
      cv::cvtColor(img_ptr->image, img_ptr->image, CV_BGR2HSV); // BGR -> HSV.
      cv::inRange(img_ptr->image, cv::Scalar{low_h_, low_s_, low_v_}, cv::Scalar{high_h_, high_s_, high_v_}, img_ptr->image); // Detect orange.
      cv::cvtColor(img_ptr->image, img_ptr->image, CV_GRAY2BGR, 3); // 1ch -> 3ch.
      image_pub_.publish(img_ptr->toImageMsg());
    }
    catch (cv_bridge::Exception& e) {
      ROS_ERROR_STREAM("cv_bridge exception: " << e.what());
    }
  }
  void update_threshold(opencv3mixing::ColorThresholdConfig& config, uint32_t level)
  {
    low_h_  = (config.center_h - config.range_h*0.5)*0.5;
    low_s_  = (config.center_s - config.range_s*0.5)*2.55;
    low_v_  = (config.center_v - config.range_v*0.5)*2.55;

    high_h_ = (config.center_h + config.range_h*0.5)*0.5;
    high_s_ = (config.center_s + config.range_s*0.5)*2.55;
    high_v_ = (config.center_v + config.range_v*0.5)*2.55;
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic {};
  ros::spin();
  return 0;
}
