// Test file for the gait of robot, to test with localization
// Main purpose: publish transform from "/odom" to "/robot_base"
// Last update: 2014.07.13

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include "gait/Step_msg.h"

tf::Vector3 robot_displacement(0,0,0);
double robot_body_rotation = 0;

void stepUpdateCallback(const gait::Step_msgConstPtr& step_msg)
{
  robot_displacement = tf::Vector3(step_msg->displacement_x,step_msg->displacement_y,step_msg->displacement_x);
  robot_body_rotation = step_msg->body_rotation;
  //ROS_INFO("odom trans: Got new step dis:(%f,%f) rot:%f",
  //          robot_displacement.x(),robot_displacement.y(),robot_body_rotation);
}

int main(int argc, char** argv){

  ros::init(argc, argv, "gait_odom_broadcaster");

  ros::NodeHandle node;
  ros::Subscriber sub_step_update = node.subscribe("gait/step_update",100,stepUpdateCallback);

  //double dt = 0.05; // time between step control commands

  tf::TransformBroadcaster broadcaster;
  tf::TransformListener listener;

  tf::StampedTransform odom;
  odom.frame_id_ = "/odom";
  odom.child_frame_id_ = "/robot_base";
  odom.setIdentity();
  odom.stamp_ = ros::Time::now();

  ROS_INFO("Publishing initial transform odom to robot_base");

  // broadcast the initial transform
  broadcaster.sendTransform(odom);

  ros::Rate rate(20); // because step period is 1/20 = 0.05sec
    while (node.ok()){

      try{
          // wait to listen the old transform from "/odom" to "/robot_base"
          ros::Time previous_frame_time = odom.stamp_;
          listener.waitForTransform("/odom", "/robot_base",
                                    previous_frame_time, ros::Duration(0.5));
          listener.lookupTransform("/odom", "/robot_base",
                                   previous_frame_time, odom);
      }
      catch (tf::TransformException ex){
        ROS_ERROR("localization odom trans:%s",ex.what());
      }

      // set the displacement of each step

      tf::Transform stepTransform(
                  tf::createQuaternionFromYaw(robot_body_rotation),robot_displacement);

      // multiply the old transform with the new displacement transform
      odom.setData(odom * stepTransform);
      odom.stamp_ = ros::Time::now();

      //ROS_INFO("New transform has data %f, %f",odom.getOrigin().getX(),odom.getOrigin().getY());

      //ROS_INFO("Publishing new transform odom to robot_base");

      // broadcast the new transform
      broadcaster.sendTransform(odom);

      ros::spinOnce();
    }

  return 0;
};

