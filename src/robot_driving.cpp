#include <ros/ros.h>

#include <functional>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "enshu2/myrobot.h"
#include "enshu2/utils.hpp"
#include "enshu2/detection_camera.hpp"
#include <enshu2/MarkerInfo.h>

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

      ROS_INFO("Left Sonar %i cm", ls);
      ROS_INFO("Right Sonar %i cm", rs);

      double v = 0.0;
      double omega = 0;

      std::vector<enshu2::MarkerInfo> markers=camera.get_marker();

      ROS_INFO("Markers received: %lu", markers.size());
      if(markers.size()==0&&ls!=0&&rs!=0){ //no marker detected, use sonar 
        v=0.0;
        omega=0.0;
        robot.move(v, omega);
      }
      else{ //marker detected, set robot speed based on marker
        for (const auto& marker : markers)
        {
          // Access marker information
          int marker_id = marker.id;
          double distance = marker.distance;
          double theta = marker.theta;
          double yaw = marker.yaw;

          // Print the marker's information
          ROS_INFO("Marker ID: %d, Distance: %.2f meters, Theta: %.2f radians, Yaw: %.2f radians", 
                    marker_id, distance, theta, yaw);


          if(marker_id==5){
            if(theta<-0.2){
              omega=-0.2;
            }
            else if(theta>0.2){
              omega=0.2;
            }
            else{
              omega=0;
            }
            
          }

        }
        // Send command to robot
        robot.move(v, omega);
      }

      /// Display robot command
      //camera.add_command(v, omega);
      /// Show image
      camera.show_markers();
      camera.show_img();
    }

    ////////////////////////////////////////////////////////
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
