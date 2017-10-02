#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

// #include <opencv2/improc.hpp>

class EdgeDetector {
public:
  void detect_edge(const sensor_msgs::ImageConstPtr& msg);
private:
  ros::NodeHandle nh_{};
  ros::NodeHandle pnh_{"~"};
  image_transport::ImageTransport it_{nh_};
  image_transport::ImageTransport pit_{pnh_};
  image_transport::Subscriber sub_{it_.subscribe("src_image", 1, &EdgeDetector::detect_edge, this)};
  image_transport::Publisher pub_{pit_.advertise("dest_image", 1)};
};

void EdgeDetector::detect_edge(const sensor_msgs::ImageConstPtr& msg)
{
  cv_bridge::CvImageConstPtr src{cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8)};
  cv::Mat dest{};
  cv::Canny(src->image, dest, 50, 200);
  pub_.publish(cv_bridge::CvImage{std_msgs::Header{}, "mono8", dest}.toImageMsg());
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "edge_detector");
  EdgeDetector detector{};
  ros::spin();
  return 0;
}
