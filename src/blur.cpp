#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

// #include <opencv2/improc.hpp>

class ImageBlurer {
public:
  ImageBlurer();
  void blur_image(const sensor_msgs::ImageConstPtr& msg);
private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  image_transport::ImageTransport it_;
  image_transport::ImageTransport pit_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
};

ImageBlurer::ImageBlurer()
  : nh_{},
    pnh_{"~"},
    it_{nh_},
    pit_{pnh_},
    image_sub_{it_.subscribe("src_image", 1, &ImageBlurer::blur_image, this)},
    image_pub_{pit_.advertise("dest_image", 1)}
{
}

void ImageBlurer::blur_image(const sensor_msgs::ImageConstPtr& msg)
{
  try {
    cv::Mat dest_image;
    cv::blur(cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8)->image, dest_image, cv::Size(2, 100));
    image_pub_.publish(cv_bridge::CvImage{std_msgs::Header{}, "bgr8", dest_image}.toImageMsg());
  }
  catch (cv_bridge::Exception& e) {
    ROS_ERROR_STREAM("cv_bridge exception: " << e.what());
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_blurer");
  ImageBlurer ib{};
  ros::spin();
  return 0;
}
