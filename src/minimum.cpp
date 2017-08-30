#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

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
      cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      // Draw an example circle on the video stream
      if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
        cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));

      // Output modified video stream
      image_pub_.publish(cv_ptr->toImageMsg());
    } catch (cv_bridge::Exception& e) {
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
