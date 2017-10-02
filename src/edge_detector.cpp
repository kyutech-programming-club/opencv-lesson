#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <dynamic_reconfigure/server.h>
#include <opencv3mixing/EdgeParamConfig.h>

// #include <opencv2/improc.hpp>

class EdgeDetector {
public:
  EdgeDetector();
  void detect_edge(const sensor_msgs::ImageConstPtr& msg);
  void update_param(opencv3mixing::EdgeParamConfig& config, uint32_t level);
private:
  ros::NodeHandle nh_{};
  ros::NodeHandle pnh_{"~"};
  image_transport::ImageTransport it_{nh_};
  image_transport::ImageTransport pit_{pnh_};
  image_transport::Subscriber sub_{it_.subscribe("src_image", 1, &EdgeDetector::detect_edge, this)};
  image_transport::Publisher pub_{pit_.advertise("dest_image", 1)};
  dynamic_reconfigure::Server<opencv3mixing::EdgeParamConfig> server;
  dynamic_reconfigure::Server<opencv3mixing::EdgeParamConfig>::CallbackType f{boost::bind(&EdgeDetector::update_param, this, _1, _2)};
  double high_thresh_, low_thresh_;
  int apertureSize_;
  bool L2gradient_;
};

EdgeDetector::EdgeDetector()
{
  server.setCallback(f);
}

void EdgeDetector::detect_edge(const sensor_msgs::ImageConstPtr& msg)
{
  cv_bridge::CvImageConstPtr src{cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8)};
  cv::Mat dest{};
  cv::Canny(src->image, dest, low_thresh_, high_thresh_, apertureSize_, L2gradient_);
  pub_.publish(cv_bridge::CvImage{std_msgs::Header{}, "mono8", dest}.toImageMsg());
}

void EdgeDetector::update_param(opencv3mixing::EdgeParamConfig& config, uint32_t level)
{
  high_thresh_ = config.high_thresh;
  low_thresh_ = config.low_thresh;
  apertureSize_ = config.apertureSize;
  L2gradient_ = config.L2gradient;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "edge_detector");
  EdgeDetector detector{};
  ros::spin();
  return 0;
}
