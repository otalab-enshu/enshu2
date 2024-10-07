#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <turtlebot3_msgs/SensorState.h>
#include <turtlebot3_msgs/Sound.h>

const double BURGER_MAX_LIN_VEL = 0.22;
const double BURGER_MAX_ANG_VEL = 2.84;
const int MAIN_RATE = 10;
const unsigned char OFF = 0;
const unsigned char ON = 1;

class MyRobot
{
private:
  ros::Publisher pub_vel_;
  ros::Publisher reset_odom_;
  ros::Publisher pub_sound_;
  ros::Subscriber sensor_state_;
  ros::Subscriber odom_;
  int ls_ = 0;
  int rs_ = 0;
  double x_ = 0.0;
  double y_ = 0.0;
  double theta_ = 0.0;
  ros::Time t_start_;
  ros::Rate rate_;
  double v_ = 0.0;
  double omega_ = 0.0;

  bool is_finished_ = false;

  double clip(double n, double abs_max)
  {
    abs_max = std::abs(abs_max);
    return std::max(-abs_max, std::min(n, abs_max));
  }

  void setSensor(const turtlebot3_msgs::SensorState sensor)
  {
    ls_ = int(sensor.sonar) / 1000;
    rs_ = int(sensor.sonar) % 1000;
  }

  void setOdom(const nav_msgs::Odometry odom)
  {
    x_ = odom.pose.pose.position.x;
    y_ = odom.pose.pose.position.y;
    theta_ = odom.pose.pose.orientation.z;
  }

public:
  MyRobot(ros::NodeHandle& n, ros::Rate& rate) : rate_(rate)
  {
    pub_vel_ = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    reset_odom_ = n.advertise<std_msgs::Empty>("/reset", 1);
    pub_sound_ = n.advertise<turtlebot3_msgs::Sound>("/sound", 1);
    sensor_state_ = n.subscribe("/sensor_state", 1, &MyRobot::setSensor, this);
    odom_ = n.subscribe("/odom", 1, &MyRobot::setOdom, this);
    wait(0.3);
    reset();
    start();
    t_start_ = ros::Time::now();
  }

  int get_ls()
  {
    return ls_;
  }

  int get_rs()
  {
    return rs_;
  }

  double get_x()
  {
    return x_;
  }

  double get_y()
  {
    return y_;
  }

  double get_theta()
  {
    return theta_;
  }

  void move(double v, double omega)
  {
    v_ = v;
    omega_ = omega;
    geometry_msgs::Twist msg;
    msg.linear.x = clip(v, BURGER_MAX_LIN_VEL);
    msg.angular.z = clip(omega, BURGER_MAX_ANG_VEL);
    pub_vel_.publish(msg);
    ros::spinOnce();
    rate_.sleep();
  }

  void stop()
  {
    v_ = 0.0;
    omega_ = 0.0;
    geometry_msgs::Twist msg;
    msg.linear.x = 0.;
    msg.angular.z = 0.;
    pub_vel_.publish(msg);
    ros::spinOnce();
    rate_.sleep();
  }

  void wait(double t)
  {
    int count = int(t * MAIN_RATE);
    for (int i = 0; i < count; i++)
    {
      geometry_msgs::Twist msg;
      msg.linear.x = clip(v_, BURGER_MAX_LIN_VEL);
      msg.angular.z = clip(omega_, BURGER_MAX_ANG_VEL);
      pub_vel_.publish(msg);
      ros::spinOnce();
      rate_.sleep();
    }
  }

  void reset()
  {
    stop();
    ROS_INFO("Resetting Odometry...");
    std_msgs::Empty empty;
    reset_odom_.publish(empty);
    for (int i = 0; i < MAIN_RATE * 3.; i++)
    {
      ros::spinOnce();
      rate_.sleep();
    }
    ROS_INFO("Done");
  }

  double get_time()
  {
    return (ros::Time::now() - t_start_).toSec();
  }

  void start()
  {
    turtlebot3_msgs::Sound s;
    s.value = ON;
    pub_sound_.publish(s);
  }

  void end()
  {
    // Stop first
    stop();
    // Make goal sound
    turtlebot3_msgs::Sound s;
    s.value = OFF;
    pub_sound_.publish(s);
    is_finished_ = true;
  }

  bool is_finished()
  {
    return is_finished_;
  }
};
