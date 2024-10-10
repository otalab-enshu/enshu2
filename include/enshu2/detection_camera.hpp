#pragma once
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/String.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <enshu2/MarkerInfoArray.h>

#include <vector>

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

  enshu2::MarkerInfoArray markerinfo_array_; 
  ros::Subscriber markerinfo_sub_;
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
    markerinfo_sub_ = n.subscribe("/marker_info_array", 2, &DetectionCamera::markerinfo_callback, this); // MarkerInfoArray subscriber
    ROS_INFO("Subscribe results: /marker_info_array");
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

  void markerinfo_callback(const enshu2::MarkerInfoArray::ConstPtr& markerinfo_msg)
  {
    
    // Check if there are any markers
    if (markerinfo_msg->markers.empty())
    {
        ROS_WARN("No markers detected.");
        return;
    }

    // Store received marker info array
    markerinfo_array_ = *markerinfo_msg;
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

  void show_markers()
{

    // Debug: Check if markerinfo_array_ is populated
    //ROS_INFO("Markers received: %lu", markerinfo_array_.markers.size());

    // Iterate through the markers and display them on the image
    for (const auto& marker_info : markerinfo_array_.markers)
    {
        //ROS_INFO("Processing Marker ID: %d", marker_info.id);

        // Draw the marker ID at the center of the marker
        std::ostringstream id_str;
        id_str << "ID: " << marker_info.id;
        cv::Point2i center(0, 0);

        // Check if the marker has 4 corners (which is expected)
        if (marker_info.corners.size() != 4)
        {
            ROS_WARN("Marker ID %d does not have 4 corners, skipping...", marker_info.id);
            continue;
        }

        // Convert geometry_msgs::Point32 to cv::Point2i and calculate the center
        for (const auto& corner : marker_info.corners)
        {
            //ROS_INFO("Corner: (%f, %f)", corner.x, corner.y);

            // Check if the corner is within the image bounds
            if (corner.x < 0 || corner.x >= img_.cols || corner.y < 0 || corner.y >= img_.rows)
            {
                ROS_WARN("Corner (%f, %f) is out of image bounds", corner.x, corner.y);
            }

            // Convert Point32 to Point2i (rounding the floating-point values to integers)
            center += cv::Point2i(static_cast<int>(corner.x), static_cast<int>(corner.y));
        }
        center /= 4;  // Average of 4 corners to get the center point

        // Draw the marker's ID on the image
        cv::putText(img_, id_str.str(), center, FONT, FONT_SIZE, FONT_GREEN, FONT_THICKNESS);

        // Draw the marker corners on the image
        for (int i = 0; i < marker_info.corners.size(); ++i)
        {
            cv::Point2i pt1(static_cast<int>(marker_info.corners[i].x), static_cast<int>(marker_info.corners[i].y));
            cv::Point2i pt2(static_cast<int>(marker_info.corners[(i + 1) % 4].x), static_cast<int>(marker_info.corners[(i + 1) % 4].y));
            cv::line(img_, pt1, pt2, FONT_RED, 2);  // Draw lines between corners
        }

        // Optionally, show other marker info (distance, theta)
        std::ostringstream info_str;
        info_str.precision(2);
        info_str << "D: " << marker_info.distance << "m, Theta: " << marker_info.theta;
        cv::putText(img_, info_str.str(), cv::Point2i(center.x, center.y + 20), FONT, FONT_SIZE/2, FONT_WHITE, FONT_THICKNESS/1.5);   
    } 
}



std::vector<enshu2::MarkerInfo> get_marker()
{
    std::vector<enshu2::MarkerInfo> marker_infos;

    // Check if markerinfo_array_ is populated
    if (markerinfo_array_.markers.empty()) 
    {
        //ROS_WARN("No markers available in the markerinfo array");
        return marker_infos;
    }

    // Iterate through the markers and extract information
    for (const auto& marker_info : markerinfo_array_.markers)
    {
        // Check if the marker has 4 corners
        if (marker_info.corners.size() != 4)
        {
            ROS_WARN("Marker ID %d does not have 4 corners, skipping...", marker_info.id);
            continue;
        }

        // Store the marker info in a vector
        marker_infos.push_back(marker_info);

        // Debugging output
        //ROS_INFO("Marker ID: %d, Distance: %.2f meters, Theta: %.2f radians", 
        //         marker_info.id, marker_info.distance, marker_info.theta);
    }
    markerinfo_array_.markers.clear();
    return marker_infos;
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
