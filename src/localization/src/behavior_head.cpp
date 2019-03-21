// Test file for the behavior of robot, to test with localization
// Main purpose: publish transform from "/robot_base" to "/robot_head"

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <decision/UDPReceived.h>

double yaw,pitch;

void HeadAngleSubCallback(const decision::UDPReceived::ConstPtr udp_msg)
{
//    m_pMotionState->headPitch = headAngle->angle_head_up;
//    m_pMotionState->headYaw = headAngle->angle_head_down;

    pitch = udp_msg->received_data.at(0);
    yaw = udp_msg->received_data.at(1);

}


int main(int argc, char** argv){

  ros::init(argc, argv, "behavior_head_broadcaster");

  ros::NodeHandle node;

  //double dt = 0.05; // time between control commands


  ros::Subscriber HeadAngleSub = node.subscribe("decision/head_rotation", 1000, HeadAngleSubCallback);

  tf::TransformBroadcaster broadcaster;
  tf::TransformListener listener;
  
  yaw = 0;
  pitch = 0;

  tf::StampedTransform head_turn;
  head_turn.frame_id_ = "/robot_base";
  head_turn.child_frame_id_ = "/robot_head";
  head_turn.setData(tf::Transform(tf::createQuaternionFromYaw(0),tf::Vector3(0,0,1.25)));
  head_turn.stamp_ = ros::Time::now();

  //ROS_INFO("behavior_head:Publishing initial transform '/robot_base' to 'robot_head'");

  // broadcast the initial transform
  broadcaster.sendTransform(head_turn);

  ros::Rate rate(20); // because step period is 1/20 = 0.05sec, this can change for head turn
    while (node.ok()){

      //this part can be used if we want each time to use the change in turning dtheta
//      try{
//          // wait to listen the old transform from "/robot_base" to "/robot_head"
//          ros::Time previous_frame_time = head_turn.stamp_;
//          listener.waitForTransform("/robot_base", "/robot_head",
//                                    previous_frame_time, ros::Duration(1.0));
//          listener.lookupTransform("/robot_base", "/robot_head",
//                                   previous_frame_time, head_turn);
//      }
//      catch (tf::TransformException ex){
//        ROS_ERROR("behavior head turn:%s",ex.what());
//      }

//      // set the angular displacement of each command
//      double dtheta = 0;

//      tf::Transform turnTransform(
//          tf::createQuaternionFromYaw(dtheta),
//          tf::Vector3(0,0,0)
//      );

//      // multiply the old transform with the new displacement transform
//      odom.setData(head_turn * turnTransform);

        //this part is used if we know the exact angular position of the head with respect to the body
        //double theta = 0; //theta in rads, - to the right, + to the left, test with 90degrees~1.57rads

        head_turn.setData(tf::Transform(tf::createQuaternionFromRPY(0,pitch,yaw),tf::Vector3(0,0,1.30)));

        head_turn.stamp_ = ros::Time::now();


        //ROS_INFO("behavior_head:Publishing new transform '/robot_base' to '/robot_head' with angle:%f",theta);

      // broadcast the new transform
      broadcaster.sendTransform(head_turn);

      rate.sleep();
    }

  return 0;
};
