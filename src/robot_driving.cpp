#include <ros/ros.h>

#include <functional>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "enshu3/detection_camera.hpp"
#include "enshu3/myrobot.h"

///////////////////////////////////////////////////
int main(int argc, char** argv)
{
  // ROS node
  ros::init(argc, argv, "enshu3_driving");
  DetectionCamera camera;
  MyRobot robot;

  // Font setting
  std::string text_contents;
  cv::Point2i font_loc;
  int font_baseline;
  int FONT = cv::FONT_HERSHEY_SIMPLEX;
  double FONT_SIZE = 0.8;
  double FONT_THICKNESS = 2;
  cv::Scalar FONT_RED = cv::Scalar(0, 0, 255);
  cv::Scalar FONT_GREEN = cv::Scalar(0, 255, 0);
  cv::Scalar FONT_BLUE = cv::Scalar(255, 0, 0);
  cv::Scalar FONT_BLACK = cv::Scalar(0, 0, 0);
  cv::Scalar FONT_WHITE = cv::Scalar(255, 255, 255);
  std::vector<cv::Scalar> COLORS = { cv::Scalar(200, 0, 0),   cv::Scalar(0, 200, 0),   cv::Scalar(0, 0, 200),
                                     cv::Scalar(200, 0, 200), cv::Scalar(200, 200, 0), cv::Scalar(0, 200, 200) };
  std::hash<std::string> hasher;

  // Main loop
  ros::Rate rate(10);
  while (ros::ok())
  {
    int ls = robot.get_ls();
    int rs = robot.get_rs();

    ROS_INFO("Left Sonar %i cm", ls);
    ROS_INFO("Right Sonar %i cm", rs);

    cv::Mat img = camera.get_img();
    std::vector<BBox> detection = camera.get_detection();
    if (!img.empty())
    {
      int width = img.cols;
      int height = img.rows;
      double center_i = width / 2.0;
      double center_j = height / 2.0;

      /// <write your code>

      // Command for robot
      double v = 0;
      double omega = 0;

      /////////////////////////////////////////////
      // Only three type of object can be detected
      // - stop sign
      // - person
      // - traffic light
      for (int i = 0; i < detection.size(); i++)
      {
        BBox bbox = detection[i];
        cv::Point2i center = (bbox.ul + bbox.br) / 2;
        cv::Point2i size = (bbox.br - bbox.ul);
        ROS_INFO("[%s] center:(%d, %d), width:%d, height:%d", bbox.label.c_str(), center.y, center.x, size.x, size.y);

        if (bbox.label == "stop sign")
        {
          if (center.x < center_i - 50)
          {
            v = 0.02;
            omega = 0.2;
          }
          else if (center.x > center_i + 50)
          {
            v = 0.02;
            omega = -0.2;
          }
          else
          {
            v = 0.04;
            omega = 0.0;
          }
        }
      }

      // Send command to robot
      robot.move(v, omega);
      /// </write your code>

      ///////////////////////////////////////
      // Command
      std::ostringstream ostr;
      ostr.precision(3);
      ostr << "[v:" << v << ", omega:" << omega << "]";
      text_contents = ostr.str();
      int baseline;
      cv::Size font_size = cv::getTextSize(text_contents, FONT, 1.5 * FONT_SIZE, 3, &baseline);
      font_loc = cv::Point2i(center_i - font_size.width / 2, center_j + font_size.height / 2 + baseline);
      cv::putText(img, text_contents, font_loc, FONT, 1.5 * FONT_SIZE, cv::Scalar(255, 255, 255), FONT_THICKNESS);

      // Detection
      for (int i = 0; i < detection.size(); i++)
      {
        cv::Scalar color = COLORS[hasher(detection[i].label) % COLORS.size()];
        font_loc = cv::Point2i(detection[i].ul.x, detection[i].ul.y - 5);
        cv::putText(img, detection[i].label, font_loc, FONT, 0.8, color, 2);
        cv::rectangle(img, cv::Rect(detection[i].ul, detection[i].br), color, 2);
      }

      // resize for visualization
      cv::resize(img, img, cv::Size(), 2, 2);
      cv::imshow("img", img);
      if (cv::waitKey(10) == 27)
      {
        break;
      }
    }
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
