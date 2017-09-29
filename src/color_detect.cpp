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
  ros::NodeHandle nh_ {};
  ros::NodeHandle pnh_ {"~"};
  image_transport::ImageTransport it_ {nh_};
  image_transport::ImageTransport pit_ {pnh_};
  image_transport::Subscriber image_sub_ {it_.subscribe("input_image", 1, &ImageConverter::imageCb, this)};
  image_transport::Publisher image_pub_ {pit_.advertise("image_raw", 1)};
  dynamic_reconfigure::Server<opencv3mixing::ColorThresholdConfig> server{};
  dynamic_reconfigure::Server<opencv3mixing::ColorThresholdConfig>::CallbackType f{boost::bind(&ImageConverter::receive_threshold, this, _1, _2)};
  int  low_h,  low_s,  low_v;
  int high_h, high_s, high_v;
public:
  ImageConverter()
  {
    server.setCallback(f);
  }
  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    try {
      cv::Mat mask_img;
      cv::Mat hsv_img;
      cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
      cv_bridge::CvImageConstPtr mask_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);

      cv::cvtColor(cv_ptr->image, hsv_img, CV_BGR2HSV);
      cv::inRange(hsv_img, cv::Scalar(low_h, low_s, low_v), cv::Scalar(high_h, high_s, high_v), mask_img); //オレンジ色検出
      cv::cvtColor(mask_img, mask_ptr->image, CV_GRAY2BGR, 3); //mask_img[1ch]を3chのMat画像(mask_ptr->image)に変換
      image_pub_.publish(mask_ptr->toImageMsg());
    }
    catch (cv_bridge::Exception& e) {
      ROS_ERROR_STREAM("cv_bridge exception: " << e.what());
    }
  }
  void receive_threshold(opencv3mixing::ColorThresholdConfig& config, uint32_t level)
  {
    low_h = config.low_h;
    low_s = config.low_s;
    low_v = config.low_v;
    high_h = config.high_h;
    high_s = config.high_s;
    high_v = config.high_s;
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic {};
  ros::spin();
  return 0;
}
