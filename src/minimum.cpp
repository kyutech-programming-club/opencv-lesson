#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/imgproc.hpp>

class ImageConverter
{
  ros::NodeHandle nh_ {};
  ros::NodeHandle pnh_ {"~"};//ノードハンドル(プライベート)
  image_transport::ImageTransport it_ {nh_};//ImageTransportデータクラスの作成:ノードハンドルを渡して初期化
  image_transport::ImageTransport pit_ {pnh_};//ImageTransportデータクラスの作成:プライベートノードハンドルを渡して初期化
  image_transport::Subscriber image_sub_ {it_.subscribe("input_image", 1, &ImageConverter::imageCb, this)};//thisはあるクラス内でそのクラス自身を指す参照(ポインタなど),これを介してROSトピックによるinputに付随するコールバック処理"imageCb"の実行
  image_transport::Publisher image_pub_ {pit_.advertise("image_raw", 1)};//image_rawトピックへの吐き出しを指定して, データサイズを"1"に指定

public:
  void imageCb(const sensor_msgs::ImageConstPtr& msg)//15行目で指定したコールバック処理の具体的な記載:
  {
    try {
      cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);//openCV形式にコンバート
      // Draw an example circle on the video stream
      if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)//画像サイズがこれから書き込む↓のcircleに満足かを確認
        cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));//唯一のOpenCV処理:円形描画

      // Output modified video stream
      image_pub_.publish(cv_ptr->toImageMsg());//cv_ptrの指すMat構造体(openCV形式)をMessage(ROS形式)にしてパブリッシュ
    } catch (cv_bridge::Exception& e) {//例外処理
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
