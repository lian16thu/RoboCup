// Test file for the gait of robot, to test with localization
// Main purpose: send message with step update for the gait_odom_publisher to
// publish transform from "/odom" to "/robot_base"
// Last update: 2014.07.15

#include <ros/ros.h>
#include "gait/Step_msg.h"

int main(int argc, char** argv){

  ros::init(argc, argv, "test_odom_step");

  ros::NodeHandle node;

  ros::Publisher pub_step_update = node.advertise<gait::Step_msg>("gait/step_update",100);

  gait::Step_msg new_step;

  new_step.displacement_x = 0.2;
  new_step.displacement_y = 0.0;
  new_step.displacement_z = 0.0;
  new_step.body_rotation = 0.0;

  ros::Rate rate(20); // because step period is 1/20 = 0.05sec, this can change for head turn
    while (node.ok()){

        pub_step_update.publish(new_step);
        ROS_INFO("test_odom: published new step info x=%f",new_step.displacement_x);

        ros::spinOnce();
    }

  return 0;
};

