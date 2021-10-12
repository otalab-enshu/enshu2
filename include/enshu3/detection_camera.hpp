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

  ros::Time t_start_;

public:
  DetectionCamera()
  {
    ros::NodeHandle n;
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
    ros::shutdown();
  }
};
