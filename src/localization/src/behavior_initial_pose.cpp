#include "ros/ros.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/tf.h>

#include <sstream>


int main(int argc, char **argv)
{
  ros::init(argc, argv, "behavior_initial_pose");
  ros::NodeHandle nh;
  ros::Publisher pub_initial_pose = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("behavior/initial_pose",10);

  geometry_msgs::PoseWithCovarianceStamped msg_initial_pose;

  msg_initial_pose.header.stamp = ros::Time::now();
  msg_initial_pose.pose.pose.position.x = 0.0;
  msg_initial_pose.pose.pose.position.y = 0.0;
  msg_initial_pose.pose.pose.orientation.x = 0.0;
  msg_initial_pose.pose.pose.orientation.y = 0.0;
  msg_initial_pose.pose.pose.orientation.z = 0.0;
  msg_initial_pose.pose.pose.orientation.w = 1.0;

  ROS_INFO("behavior_initial_pose:Publishing initial pose of (%f,%f,%f)", msg_initial_pose.pose.pose.position.x,  msg_initial_pose.pose.pose.position.y,
           tf::getYaw(msg_initial_pose.pose.pose.orientation));
//  pub_initial_pose.publish(msg_initial_pose);

  ros::Rate loop_rate(0.5); // less than 1.3
  loop_rate.sleep(); // wait 1 sec before giving the command to publish the initial pose

  pub_initial_pose.publish(msg_initial_pose);

  int count = 0;
  while (ros::ok())
  {

    ros::spinOnce();
    loop_rate.sleep();
    ++count;

  }

return 0;
}
