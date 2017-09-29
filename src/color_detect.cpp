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
  int  low_h_,  low_s_,  low_v_;
  int high_h_, high_s_, high_v_;
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
      cv::Mat mask_img;
      cv::Mat hsv_img;
      cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
      cv_bridge::CvImageConstPtr mask_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);

      cv::cvtColor(cv_ptr->image, hsv_img, CV_BGR2HSV);
      cv::inRange(hsv_img, cv::Scalar(low_h_, low_s_, low_v_), cv::Scalar(high_h_, high_s_, high_v_), mask_img); //オレンジ色検出
      cv::cvtColor(mask_img, mask_ptr->image, CV_GRAY2BGR, 3); //mask_img[1ch]を3chのMat画像(mask_ptr->image)に変換
      image_pub_.publish(mask_ptr->toImageMsg());
    }
    catch (cv_bridge::Exception& e) {
      ROS_ERROR_STREAM("cv_bridge exception: " << e.what());
    }
  }
  void update_threshold(opencv3mixing::ColorThresholdConfig& config, uint32_t level)
  {
    low_h_  = config.low_h;  low_s_  = config.low_s;  low_v_ = config.low_v;
    high_h_ = config.high_h; high_s_ = config.high_s; high_v_ = config.high_s;
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic {};
  ros::spin();
  return 0;
}
