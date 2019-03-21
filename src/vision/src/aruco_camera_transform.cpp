#include <math.h>
#include <iostream>
#include <tiff.h>
#include <cmath>
#include <string>
#include <sstream>
#include "std_msgs/String.h"

#include <ros/ros.h>
#include <fstream>

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "fiducial_msgs/FiducialTransformArray.h"


class arucoTransformer {

    ros::NodeHandle nh;

    int fiducial_msg_counter;

public:
    ros::Publisher pub_trans_aruco;
    ros::Subscriber sub_aruco;


    arucoTransformer() {

        fiducial_msg_counter = 0;

        ROS_INFO("Aruco transformer initialized");

        sub_aruco = nh.subscribe("/fiducial_transforms_camera", 10, &arucoTransformer::CBonArucoReceived,this);
        pub_trans_aruco = ros::Publisher(nh.advertise<fiducial_msgs::FiducialTransformArray>("/fiducial_transforms", 1));
    }

    void CBonArucoReceived(const fiducial_msgs::FiducialTransformArray::ConstPtr fiducial_msg);
};

void arucoTransformer::CBonArucoReceived(const fiducial_msgs::FiducialTransformArray::ConstPtr fiducial_msg)
{
    fiducial_msgs::FiducialTransformArray temp_fiducials = *fiducial_msg;

    tf::TransformListener listener;
    tf::StampedTransform transform;
    
    if (fiducial_msg_counter>2)
    {
        try{
            listener.waitForTransform(fiducial_msg->header.frame_id, "/base_link",
                                      ros::Time(0), ros::Duration(1));
            listener.lookupTransform(fiducial_msg->header.frame_id, "/base_link",
                                     ros::Time(0), transform);
        }
        catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
        }

    
    
    for(int i=0;i<fiducial_msg->transforms.size();i++)
    {

        tf2::Transform aruco_pose_transform;
        tf2::Transform fixed_transform;

        fixed_transform.setIdentity();
        fixed_transform.setRotation(tf2::Quaternion(0,0,M_PI/2+62.5*M_PI/180));

        tf2::fromMsg(temp_fiducials.transforms.at(i).transform,aruco_pose_transform);


//        fixed_transform.rotation.x = tf::Quaternion(M_PI/2+62.5*M_PI/180,0,0).x();
//        fixed_transform.rotation.y = tf::Quaternion(M_PI/2+62.5*M_PI/180,0,0).y();
//        fixed_transform.rotation.z = tf::Quaternion(M_PI/2+62.5*M_PI/180,0,0).z();
//        fixed_transform.rotation.w = tf::Quaternion(M_PI/2+62.5*M_PI/180,0,0).w();

//        aruco_pose_transformed = fiducial_msg->transforms.at(i).transform;

//        listener.transformPose("/base_link",ros::Time(0),left_aruco_pose_camera,
//                                   left_aruco_pose_camera.header.frame_id,left_aruco_pose_base);

//        aruco_pose_transform = fixed_transform  fiducial_msg->transforms.at(i).transform;

        aruco_pose_transform = aruco_pose_transform * fixed_transform;

        ROS_INFO("Tranformed aruco");

//        temp_fiducials.transforms.at(i).transform.rotation.x += fixed_transform.rotation.x;
//        temp_fiducials.transforms.at(i).transform.rotation.y += fixed_transform.rotation.y;
//        temp_fiducials.transforms.at(i).transform.rotation.z += fixed_transform.rotation.z;
//        temp_fiducials.transforms.at(i).transform.rotation.w += fixed_transform.rotation.w;

        temp_fiducials.transforms.at(i).transform = tf2::toMsg(aruco_pose_transform);

    }

    }

    pub_trans_aruco.publish(temp_fiducials);
    fiducial_msg_counter++;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "aruco_camera_transformer");

    arucoTransformer at;

    while(ros::ok())
    {

        ros::spinOnce();

        //rate.sleep();
    }

    return 0;

}
