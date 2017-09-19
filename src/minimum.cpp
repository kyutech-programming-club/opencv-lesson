#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/imgproc.hpp>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "minimum_node");

  ros::NodeHandle nh {};
  ros::NodeHandle pnh {"~"};//ノードハンドル(プライベート)
  image_transport::ImageTransport it {nh};//ImageTransportデータクラスの作成:ノードハンドルを渡して初期化
  image_transport::ImageTransport pit {pnh};//ImageTransportデータクラスの作成:プライベートノードハンドルを渡して初期化
  auto image_pub {pit.advertise("image_raw", 1)};//image_rawトピックへの吐き出しを指定して, データサイズを"1"に指定

  auto image_sub {it.subscribe<sensor_msgs::Image>("input_image", 1, [&image_pub](const sensor_msgs::ImageConstPtr& msg){
    try {
      auto cv_ptr {cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)}; // openCV形式にコンバート
      // Draw an example circle on the video stream
      if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60) // 画像サイズがこれから書き込む↓のcircleに満足かを確認
        cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0)); // 唯一のOpenCV処理:円形描画

      // Output modified video stream
      image_pub.publish(cv_ptr->toImageMsg()); // cv_ptrの指すMat構造体(openCV形式)をMessage(ROS形式)にしてパブリッシュ
    } catch (cv_bridge::Exception& e) { // 例外処理
      ROS_ERROR_STREAM("cv_bridge exception: " << e.what());
    }
  })};

  ros::spin();
  return 0;
}
