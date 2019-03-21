// This file will publish a 2D pose wheel odometry measurement ('odom' topic), a 3D orientation IMU sensor measurement
// ('imu_data' topic) and a 3D pose Visual Odometry measurement ('vo' topic) to test the robot_pose_ekf localization package

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>

#include <sstream>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "robot_pose_ekf_localization_tester");
  ros::NodeHandle nh;
  ros::Publisher pub_wheel_odom = nh.advertise<nav_msgs::Odometry>("odom",1000);
  ros::Publisher pub_imu = nh.advertise<sensor_msgs::Imu>("imu_data",1000);
  ros::Publisher pub_visual_odom = nh.advertise<nav_msgs::Odometry>("vo",1000);

  ros::Rate loop_rate(10);

  int count = 0;
  while (ros::ok())
  {
    nav_msgs::Odometry msg_wheel_odom;
    sensor_msgs::Imu msg_imu;
    nav_msgs::Odometry msg_visual_odom;


    msg1.data = ss.str();
    ROS_INFO("String sent: %s", msg1.data.c_str());

    exercise2::Num msg2;
    msg2.num = 14;
    ROS_INFO("Number sent: %d", msg2.num);

    pub_wheel_odom.publish(msg_wheel_odom);
    pub_imu.publish(msg_imu);
    pub_visual_odom.publish(msg_visual_odom);

    ros::spinOnce();
    loop_rate.sleep();
    ++count;

  }

return 0;
}
