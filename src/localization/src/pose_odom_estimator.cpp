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
#include <tf2_ros/transform_broadcaster.h>

#include <ros/ros.h>

#include "decision/SerialReceived.h"
#include "localization/MeanPoseConfStamped.h"


nav_msgs::Odometry received_odom;
localization::MeanPoseConfStamped received_localization;
double last_received_loc_time;
double waist_to_front_x, waist_to_front_y, waist_to_front_theta;
int decision_cmd_type;

void CBonSerialReceived(const decision::SerialReceived::ConstPtr serial_input_msg)
{
    ROS_INFO("SLAM: Odometry data received from serial node: %f %f %f %f %f %f %f %f %f %f\n",serial_input_msg->received_data[0],serial_input_msg->received_data[1],
            serial_input_msg->received_data[2],serial_input_msg->received_data[3],serial_input_msg->received_data[4],
            serial_input_msg->received_data[5],serial_input_msg->received_data[6],serial_input_msg->received_data[7],
            serial_input_msg->received_data[8],serial_input_msg->received_data[9]);

    received_odom.pose.pose.position.x = serial_input_msg->received_data[0];
    received_odom.pose.pose.position.y = serial_input_msg->received_data[1];
    received_odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(serial_input_msg->received_data[2]);
    waist_to_front_x = serial_input_msg->received_data[5];
    waist_to_front_y = serial_input_msg->received_data[6];
    waist_to_front_theta = serial_input_msg->received_data[7];
}

void CBonDecisionReceived(const decision::SerialReceived::ConstPtr decision_cmd_msg)
{

    decision_cmd_type = decision_cmd_msg->received_data[0];

}


void CBOnLocalizationReceived(const localization::MeanPoseConfStamped::ConstPtr &msg)
{
    if ((fabs(msg->pose.pose.position.x) < 30) && (fabs(msg->pose.pose.position.y) < 30))
    {
        received_localization = *msg;

    }
    else
    {
        ROS_INFO("INVALID LOCALIZATION, leave the same");
    }
    last_received_loc_time = msg->header.stamp.toSec();

    ROS_INFO("Received localization from fiducials: x=%f, y=%f, theta=%f, cov=%f",
             received_localization.pose.pose.position.x,received_localization.pose.pose.position.y,
             tf::getYaw(received_localization.pose.pose.orientation)/M_PI*180,received_localization.pose.covariance);

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pose_odom_estimator");
    ros::NodeHandle nodeHandle;

    ros::Subscriber subscriber_serial_receiver = nodeHandle.subscribe("decision/serial_receiver", 10, CBonSerialReceived);
    ros::Subscriber subscriber_localization = nodeHandle.subscribe("fiducial_pose", 10, CBOnLocalizationReceived);
    ros::Subscriber subscriber_decision = nodeHandle.subscribe("/decision/command_to_serial", 10, CBonDecisionReceived);

    ros::Publisher nav_goal_pub = nodeHandle.advertise<geometry_msgs::PoseWithCovarianceStamped>("fiducial_pose_odom", 10);

    std::string mapFrame;
    std::string odomFrame;
    std::string baseFrame;

    nodeHandle.param<std::string>("map_frame", mapFrame, "map");
    nodeHandle.param<std::string>("odom_frame", odomFrame, "odom");
    nodeHandle.param<std::string>("base_frame", baseFrame, "base_link");

    double robotLoc_x = 0.0;
    double robotLoc_y = 0.0;
    double robotLoc_theta = 0.0;
    double locConfidence = 0.0;

    double previous_robotLoc_x = 0.0;
    double previous_robotLoc_y = 0.0;
    double previous_robotLoc_theta = 0.0;

    double odom_x = 0.0;
    double odom_y = 0.0;
    double odom_orientation = 0.0;

    double previous_odom_x = 0.0;
    double previous_odom_y = 0.0;
    double previous_odom_orientation = 0.0;

    double odom_dx = 0.0;
    double odom_dy = 0.0;
    double odom_dorientation = 0.0;


    double odom_clear_start_x = 0.0;
    double odom_clear_start_y = 0.0;
    double odom_clear_start_theta = 0.0;

    decision_cmd_type = 0;

    received_localization.pose.pose.position.x = 0.0;
    received_localization.pose.pose.position.y = 0.0;
    received_localization.pose.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);

    received_odom.pose.pose.position.x = 0.0;
    received_odom.pose.pose.position.y = 0.0;
    received_odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);

    last_received_loc_time = ros::Time::now().toSec();


    ROS_INFO("Go into main loop!");
    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        ros::spinOnce();

        odom_x = received_odom.pose.pose.position.x;
        odom_y = received_odom.pose.pose.position.y;
        odom_orientation = tf::getYaw(received_odom.pose.pose.orientation);

         // Calculate odometry from the previous position the localization calculated

        odom_dx = odom_x - previous_odom_x;
        odom_dy = odom_y - previous_odom_y;
        odom_dorientation = odom_orientation - previous_odom_orientation;

        //if ((fabs(odom_dx)>0.3) || (fabs(odom_dy)>0.3) || (fabs(odom_dorientation)>0.5))  //for the case the odom gets cleared
        if (decision_cmd_type == 0) // if command is 0 then odometry gets cleared
        {
            odom_dx = 0;
            odom_dy = 0;
            odom_dorientation = 0;

            odom_clear_start_x = robotLoc_x;
            odom_clear_start_y = robotLoc_y;
            odom_clear_start_theta = robotLoc_theta;
            ROS_INFO("Cleared odom, changing start point (%f,%f,%f)",odom_clear_start_x,odom_clear_start_y,odom_clear_start_theta);
        }

        robotLoc_x = received_localization.pose.pose.position.x;
        robotLoc_y = received_localization.pose.pose.position.y;
        robotLoc_theta = tf::getYaw(received_localization.pose.pose.orientation);

        ROS_INFO("Localization before waist to front (%f,%f,%f)",robotLoc_x,robotLoc_y,robotLoc_theta);

        // Add the waist_to_front for the localization
        robotLoc_x += waist_to_front_x * cos(robotLoc_theta) - waist_to_front_y * sin(robotLoc_theta);
        robotLoc_y += waist_to_front_x * sin(robotLoc_theta) + waist_to_front_y * cos(robotLoc_theta);
        robotLoc_theta += waist_to_front_theta;

        ROS_INFO("Localization after waist to front (%f,%f,%f)",robotLoc_x,robotLoc_y,robotLoc_theta);

        // If the localization result has not been refreshed for some time, we probably have no new fiducials detected and
        // the localization is stuck to the previous result. Then, replace the localization result with the odometry result

        if ((ros::Time::now().toSec() - last_received_loc_time) > 0.4)
        {
            robotLoc_x = previous_robotLoc_x + odom_dx * cos(odom_clear_start_theta) - odom_dy * sin(odom_clear_start_theta);
            robotLoc_y = previous_robotLoc_y + odom_dx * sin(odom_clear_start_theta) + odom_dy * cos(odom_clear_start_theta);
            robotLoc_theta = previous_robotLoc_theta + odom_dorientation;

            // robotLoc_x = odom_clear_start_x + odom_x * cos(odom_clear_start_theta) - odom_y * sin(odom_clear_start_theta);
            // robotLoc_y = odom_clear_start_y + odom_x * sin(odom_clear_start_theta) + odom_y * cos(odom_clear_start_theta);
            // robotLoc_theta = odom_clear_start_theta + odom_orientation;

            if ((robotLoc_theta<-1000)||(robotLoc_theta>1000))
            {
                robotLoc_theta = 0;
                ROS_INFO("Invalid value for orientation");
            }

            //ROS_INFO("RobotLoc_theta after odom %f", robotLoc_theta);
            locConfidence = 0; // because localization cannot be trusted
            ROS_INFO("Localization failed, using odometry, last cleared odom orientation:%f",odom_clear_start_theta);
        }
        else
        {
            ROS_INFO("Localization with fiducials used");
        }

        // Correct theta out of boundaries
        if (robotLoc_theta<-M_PI)
        {
            robotLoc_theta += 2*M_PI;
        }
        else if (robotLoc_theta>M_PI)
        {
            robotLoc_theta -= 2*M_PI;
        }

        previous_robotLoc_x = robotLoc_x;
        previous_robotLoc_y = robotLoc_y;
        previous_robotLoc_theta = robotLoc_theta;

        previous_odom_x = odom_x;
        previous_odom_y = odom_y;
        previous_odom_orientation = odom_orientation;

        // Publish the new localization topic

        geometry_msgs::PoseWithCovarianceStamped robotLoc_msg;
        robotLoc_msg.header.stamp = ros::Time::now();
        robotLoc_msg.header.frame_id = mapFrame;
        robotLoc_msg.pose.pose.position.x = robotLoc_x;
        robotLoc_msg.pose.pose.position.y = robotLoc_y;
        robotLoc_msg.pose.pose.orientation = tf::createQuaternionMsgFromYaw(robotLoc_theta);
        robotLoc_msg.pose.covariance.at(0) = locConfidence;

        nav_goal_pub.publish(robotLoc_msg);

        ROS_INFO("Localization after odom (%f,%f,%f)",robotLoc_x,robotLoc_y,robotLoc_theta);

        // Publish the new transform

        tf2_ros::TransformBroadcaster broadcaster;

        tf2::Transform basePose;

        basePose.setOrigin(tf2::Vector3(robotLoc_x,robotLoc_y,0));
        basePose.getBasis().setRPY(0, 0, robotLoc_theta);

        tf2::Transform outPose = basePose;

        std::string outFrame = odomFrame;
        tf::StampedTransform odomTransform;
        tf2::Transform odomTransform2;

        geometry_msgs::TransformStamped transform;

        tf::TransformListener listener;
        try{
            // wait to listen the old transform from odomFrame to baseFrame

            listener.waitForTransform(odomFrame, baseFrame,
                                      ros::Time(0), ros::Duration(1.0));
            listener.lookupTransform(odomFrame, baseFrame,
                                     ros::Time(0), odomTransform);

            odomTransform2.setOrigin(tf2::Vector3(odomTransform.getOrigin().getX(),odomTransform.getOrigin().getY(),odomTransform.getOrigin().getZ()));
            odomTransform2.getBasis().setRotation(tf2::Quaternion(odomTransform.getRotation().getX(),odomTransform.getRotation().getY(),odomTransform.getRotation().getZ(),odomTransform.getRotation().getW()));

            outPose = basePose * odomTransform2.inverse();
            outFrame = odomFrame;

            tf2::Vector3 c = odomTransform2.getOrigin();
            ROS_INFO("odom   %lf %lf %lf",
                     c.x(), c.y(), c.z());
        }
        catch (tf::TransformException ex){
            ROS_ERROR("odom_publisher odom transform:%s",ex.what());
        }

        //        tf2_ros::Buffer tfBuffer;
        //        try {

        //            transform = tfBuffer.lookupTransform(baseFrame,odomFrame, ros::Time(0));

        //            tf2::fromMsg(transform.transform, odomTransform);

        //            outPose = basePose * odomTransform.inverse();
        //            outFrame = odomFrame;

        //            tf2::Vector3 c = odomTransform.getOrigin();
        //            ROS_INFO("odom   %lf %lf %lf",
        //                     c.x(), c.y(), c.z());

        //        }
        //        catch (tf2::TransformException &ex) {
        //            ROS_WARN("%s",ex.what());

        //        }



        // Make outgoing transform make sense - ie only consist of x, y, yaw


        tf2::Vector3 translation = outPose.getOrigin();
        translation.setZ(0);
        outPose.setOrigin(translation);
        double roll, pitch, yaw;
        outPose.getBasis().getRPY(roll, pitch, yaw);
        outPose.getBasis().setRPY(0, 0, yaw);

        geometry_msgs::TransformStamped ts;
        ts.transform.translation.x = outPose.getOrigin().getX();
        ts.transform.translation.y = outPose.getOrigin().getY();
        ts.transform.translation.z = 0.0;
        ts.transform.rotation = tf::createQuaternionMsgFromYaw(yaw);
        ts.header.frame_id = mapFrame;
        ts.child_frame_id = outFrame;
        ts.header.stamp = ros::Time::now();
        broadcaster.sendTransform(ts);


        loop_rate.sleep();

    }

    // See if something needs to be added in case we exit the program
    return 0;
}
