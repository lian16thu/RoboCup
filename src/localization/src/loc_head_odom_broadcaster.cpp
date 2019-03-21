// Main purpose: publish transform from "/odom" to "/base_link" and "/base_link" to "/robot_head"
// Last update: 2018.04.20

#include <iostream>
#include <tiff.h>
#include <cmath>
#include <string>
#include <sstream>
#include <fstream>
#include "std_msgs/String.h"
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

#include "decision/SerialReceived.h"
#include "localization/MeanPoseConfStamped.h"

#include <head_motion/head_pose.h>

#include <ros/ros.h>

nav_msgs::Odometry received_odom;
double front_to_waist_x, front_to_waist_y, front_to_waist_theta;
double head_pitch,head_yaw, waist_roll, waist_pitch, waist_height;

void CBonSerialReceived(const decision::SerialReceived::ConstPtr serial_input_msg)
{
    ROS_INFO("Loc: Odometry data received from serial node: %f %f %f %f %f %f %f %f %f %f\n",serial_input_msg->received_data[0],serial_input_msg->received_data[1],
            serial_input_msg->received_data[2],serial_input_msg->received_data[3],serial_input_msg->received_data[4],
            serial_input_msg->received_data[5],serial_input_msg->received_data[6],serial_input_msg->received_data[7],
            serial_input_msg->received_data[8],serial_input_msg->received_data[9]);

    received_odom.pose.pose.position.x = serial_input_msg->received_data[0];
    received_odom.pose.pose.position.y = serial_input_msg->received_data[1];
    received_odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(serial_input_msg->received_data[2]);
   waist_roll = serial_input_msg->received_data[3];

   waist_pitch = serial_input_msg->received_data[4];
    front_to_waist_x = serial_input_msg->received_data[5];
    front_to_waist_y = serial_input_msg->received_data[6];
    front_to_waist_theta = serial_input_msg->received_data[7];
    waist_height = serial_input_msg->received_data[8];
}



void CBonHeadReceived(const head_motion::head_pose::ConstPtr& msg)
{
    //ROS_ERROR("head_pitch:%d, head_yaw:%d",msg->pitch,msg->yaw);
    head_pitch = ((double)(msg->pitch))/180*M_PI;
    //head_yaw = ((double)(msg->yaw))/180*M_PI;
    head_yaw = ((double)(msg->yaw))/180*M_PI;

    //ROS_ERROR("head_pitch:%f, head_yaw:%f", head_pitch,head_yaw);


}


int main(int argc, char** argv){

    ros::init(argc, argv, "loc_head_odom_broadcaster");
    ros::NodeHandle nodeHandle;

    ros::Subscriber subscriber_serial_receiver = nodeHandle.subscribe("decision/serial_receiver", 10, CBonSerialReceived);
    ros::Subscriber subscriber_head = nodeHandle.subscribe("decision/head_command", 10, CBonHeadReceived);

    std::string odomFrame;
    std::string baseFrame;
    std::string headFrame;

    nodeHandle.param<std::string>("odom", odomFrame, "odom");
    nodeHandle.param<std::string>("base_link", baseFrame, "base_link");
    nodeHandle.param<std::string>("robot_head", headFrame, "robot_head");

    tf::TransformBroadcaster broadcaster;

    received_odom.pose.pose.position.x = 0.0;
    received_odom.pose.pose.position.y = 0.0;
    received_odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(0);

    double odom_x,odom_y,odom_orientation;
    waist_roll = 0;
    waist_pitch = 0;
    front_to_waist_x = 0;
    front_to_waist_y = 0;
    front_to_waist_theta = 0;
    waist_height = 0.8;  //check this!
    head_pitch = 0;
    head_yaw = 0;

    tf::StampedTransform odom;
    odom.frame_id_ = odomFrame;
    odom.child_frame_id_ = baseFrame;
    odom.setData(tf::Transform(tf::createQuaternionFromYaw(0),tf::Vector3(0,0,0)));
    odom.stamp_ = ros::Time::now();

    // broadcast the initial transform
    broadcaster.sendTransform(odom);

    tf::StampedTransform head_turn;
    head_turn.frame_id_ = baseFrame;
    head_turn.child_frame_id_ = headFrame;
    head_turn.setData(tf::Transform(tf::createQuaternionFromYaw(0),tf::Vector3(0,0,waist_height+0.533)));
    head_turn.stamp_ = ros::Time::now();

    // broadcast the initial transform
    broadcaster.sendTransform(head_turn);


    ROS_INFO("Go into main loop!");
    ros::Rate rate(10); // because step period is 1/20 = 0.05sec, this can change for head turn

    int step_counter = 0;

    while (ros::ok())
    {
        ros::spinOnce();

        odom_x = received_odom.pose.pose.position.x;
        odom_y = received_odom.pose.pose.position.y;
        odom_orientation = tf::getYaw(received_odom.pose.pose.orientation);

//        // Debugging, change odom manually
        //odom_x = step_counter*0.01;
        step_counter++;

        odom.setData(tf::Transform(tf::createQuaternionFromRPY(0,0,odom_orientation),tf::Vector3(odom_x,odom_y,0)));

        odom.stamp_ = ros::Time::now();


        ROS_INFO("loc_odom:New transform '/odom' to '/base_link'with x,y,theta:%f,%f,%f",
                 odom_x,odom_y,odom_orientation/M_PI*180);

        // broadcast the new odom transform
        broadcaster.sendTransform(odom);


        // base to head like front to waist
        head_turn.setData(tf::Transform(tf::createQuaternionFromRPY(waist_roll,waist_pitch+head_pitch,head_yaw+front_to_waist_theta), // check this
                                        tf::Vector3(front_to_waist_x + 0.160 * sin(head_pitch) * cos(head_yaw),
                                                    front_to_waist_y + 0.160 * sin(head_pitch) * sin(head_yaw),
                                                    waist_height+0.533-(0.160-0.160*cos(head_pitch)))));

        head_turn.stamp_ = ros::Time::now();


        ROS_INFO("loc_head: New transform '/base_link' to '/robot_head' with x,y,theta:%f,%f,%f",
                 head_turn.getOrigin().x(),head_turn.getOrigin().y(),(head_yaw-front_to_waist_theta)/M_PI*180);

        // broadcast the new base to head transform
        broadcaster.sendTransform(head_turn);

        rate.sleep();

    }

    return 0;
};

