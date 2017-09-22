#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp> //inRange用

#define LOW_B 0
#define LOW_G 0
#define LOW_R 150
#define UP_B 110
#define UP_G 165
#define UP_R 255

class ImageConverter
{
  ros::NodeHandle nh_ {};
  ros::NodeHandle pnh_ {"~"};
  image_transport::ImageTransport it_ {nh_};
  image_transport::ImageTransport pit_ {pnh_};
  image_transport::Subscriber image_sub_ {it_.subscribe("input_image", 1, &ImageConverter::imageCb, this)};
  image_transport::Publisher image_pub_ {pit_.advertise("image_raw", 1)};

public:
  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    try {
      cv::Mat mask_img;
      cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
      cv_bridge::CvImageConstPtr mask_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);

      cv::inRange(cv_ptr->image, cv::Scalar(LOW_B, LOW_G, LOW_R), cv::Scalar(UP_B, UP_G, UP_R), mask_img); //オレンジ色検出(閾値要調整)
      cv::cvtColor(mask_img, mask_ptr->image, CV_GRAY2BGR, 3); //mask_img(1ch)を3chのMat画像(mask_ptr->image)に変換
      image_pub_.publish(mask_ptr->toImageMsg());
    }
    catch (cv_bridge::Exception& e) {
      ROS_ERROR_STREAM("cv_bridge exception: " << e.what());
    }
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic {};
  ros::spin();
  return 0;
}
