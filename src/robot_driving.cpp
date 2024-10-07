#include <ros/ros.h>

#include <functional>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "enshu2/myrobot.h"
#include "enshu2/utils.hpp"
#include "enshu2/detection_camera.hpp"

int main(int argc, char** argv)
{
  // ROS node
  ros::init(argc, argv, "enshu2_driving");
  ros::NodeHandle n;
  ros::Rate rate(MAIN_RATE);
  DetectionCamera camera(n, rate);
  MyRobot robot(n, rate);

  // Main loop
  while (ros::ok() && !robot.is_finished() && !camera.is_finished())
  {
    //////////// <write your code from here> /////////////

    cv::Mat img = camera.get_img();
    if (!img.empty())
    {
      int ls = robot.get_ls();
      int rs = robot.get_rs();

      double v = 0.0;
      double omega = 0;

      // Send command to robot
      robot.move(v, omega);

      /// Display robot command
      camera.add_command(v, omega);
      /// Show image
      camera.show_img();
    }

    ////////////////////////////////////////////////////////
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
