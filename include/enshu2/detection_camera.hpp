#pragma once
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/String.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

const int FONT = cv::FONT_HERSHEY_SIMPLEX;
const double FONT_SIZE = 0.8;
const double FONT_THICKNESS = 2;
const cv::Scalar FONT_RED = cv::Scalar(0, 0, 255);
const cv::Scalar FONT_GREEN = cv::Scalar(0, 255, 0);
const cv::Scalar FONT_BLUE = cv::Scalar(255, 0, 0);
const cv::Scalar FONT_BLACK = cv::Scalar(0, 0, 0);
const cv::Scalar FONT_WHITE = cv::Scalar(255, 255, 255);
const std::vector<cv::Scalar> COLORS = { cv::Scalar(200, 0, 0),   cv::Scalar(0, 200, 0),   cv::Scalar(0, 0, 200),
                                         cv::Scalar(200, 0, 200), cv::Scalar(200, 200, 0), cv::Scalar(0, 200, 200) };

class DetectionCamera
{
private:
  image_transport::Subscriber image_sub_;
  cv::Mat img_;

  ros::Rate rate_;
  ros::Time t_start_;

  int width_;
  int height_;
  double center_i_;
  double center_j_;

  int font_baseline_;
  std::hash<std::string> hasher_;

  bool is_finished_ = false;

public:
  DetectionCamera(ros::NodeHandle& n, ros::Rate& rate) : rate_(rate)
  {
    image_transport::ImageTransport it(n);
    image_sub_ = it.subscribe("/camera/color/image_raw", 2, &DetectionCamera::image_callback, this,
                              image_transport::TransportHints("compressed"));
    ROS_INFO("Subscribe image: /camera/color/image_raw");
    t_start_ = ros::Time::now();
  }

  void image_callback(const sensor_msgs::ImageConstPtr& rgb_msg)
  {
    // Load RGB messages
    cv_bridge::CvImagePtr rgb_ptr;
    try
    {
      rgb_ptr = cv_bridge::toCvCopy(rgb_msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    img_ = rgb_ptr->image.clone();
    width_ = img_.cols;
    height_ = img_.rows;
    center_i_ = width_ / 2.0;
    center_j_ = height_ / 2.0;
  }


  double get_time()
  {
    return (ros::Time::now() - t_start_).toSec();
  }

  void add_command(double v, double omega)
  {
    // Command
    std::ostringstream ostr;
    ostr.precision(3);
    ostr << "[v:" << v << ", omega:" << omega << "]";
    std::string text_contents = ostr.str();
    int baseline;
    cv::Size font_size = cv::getTextSize(text_contents, FONT, 1.5 * FONT_SIZE, 3, &baseline);
    cv::Point2i font_loc = cv::Point2i(center_i_ - font_size.width / 2, center_j_ + font_size.height / 2 + baseline);
    cv::putText(img_, text_contents, font_loc, FONT, 1.5 * FONT_SIZE, cv::Scalar(255, 255, 255), FONT_THICKNESS);
  }


  void show_img()
  {
    // resize for visualization
    cv::Mat img;
    cv::resize(img_, img, cv::Size(), 2, 2);
    cv::imshow("img", img);
    if (cv::waitKey(10) == 27)
    {
      end();
    }
  }

  cv::Mat get_img()
  {
    return img_;
  }


  void end()
  {
    is_finished_ = true;
  }

  bool is_finished()
  {
    return is_finished_;
  }
};
