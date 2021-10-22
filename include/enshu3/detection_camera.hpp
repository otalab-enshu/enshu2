#pragma once
#include <cv_bridge/cv_bridge.h>
#include <enshu_msgs/BboxArray.h>
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

struct BBox
{
  cv::Point2i ul;
  cv::Point2i br;
  std::string label;
  double score;
};

class DetectionCamera
{
private:
  image_transport::Subscriber image_sub_;
  cv::Mat img_;

  ros::Subscriber result_sub_;
  std::vector<BBox> detection_;

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
    result_sub_ = n.subscribe("/bbox_results", 2, &DetectionCamera::result_callback, this);
    ROS_INFO("Subscribe hand detection results: /bbox_results");
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

  void result_callback(const enshu_msgs::BboxArrayConstPtr& msg)
  {
    detection_.clear();
    for (int i = 0; i < msg->bbox.size(); i++)
    {
      // BBox struct
      // cv::Point2i ul;
      // cv::Point2i br;
      // std::string label;
      // double score;
      BBox bbox = {
        cv::Point2i(msg->bbox[i].ul.x, msg->bbox[i].ul.y),
        cv::Point2i(msg->bbox[i].br.x, msg->bbox[i].br.y),
        msg->bbox[i].label,
        msg->bbox[i].score,
      };
      detection_.push_back(bbox);
    }
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

  void add_detection()
  {
    // Detection
    for (int i = 0; i < detection_.size(); i++)
    {
      cv::Scalar color = COLORS[hasher_(detection_[i].label) % COLORS.size()];
      cv::Point2i font_loc = cv::Point2i(detection_[i].ul.x, detection_[i].ul.y - 5);
      cv::putText(img_, detection_[i].label, font_loc, FONT, 0.8, color, 2);
      cv::rectangle(img_, cv::Rect(detection_[i].ul, detection_[i].br), color, 2);
    }
  }

  void show_img()
  {
    // resize for visualization
    cv::resize(img_, img_, cv::Size(), 2, 2);
    cv::imshow("img", img_);
    if (cv::waitKey(10) == 27)
    {
      end();
    }
  }

  cv::Mat get_img()
  {
    return img_;
  }

  std::vector<BBox> get_detection()
  {
    return detection_;
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
