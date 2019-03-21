///////////////////////////////////////////////////////////////////////////////////////////////
//// Name         : bumblebee_pcl_process.cpp                                              ////
//// Description  : Executable ROS .cpp file containing the detection with Bumblebee PCL   ////
//// Function     : This program receives the Point Cloud data published by the stereo_    ////
////                image_proc node after the process of the bumblebee stereo camera image ////
////                stream and processes the point cloud to extract the surface plane and  ////
////                segment the surrounding environment to clusters to recognize objects   ////
////                like the ball, robots/humans or the goalposts.                         ////
//// Dependencies : bumblebee_pcl_process.h, assisting header file                         ////
////                pcl library headers, for the use of the RANSAC algorithm               ////
//// Maintainer   : Stasinopoulos Sotirios     email:sotstas@gmail.com                     ////
//// Last update  : 2017.05.10                                                             ////
///////////////////////////////////////////////////////////////////////////////////////////////


#include <ros/ros.h>
#include <limits>
#include <fstream>
#include <vector>
#include <Eigen/Core>
#include <iostream>
#include <ctime>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <cv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/image_encodings.h>

#include "stereo_process/DepthRequest.h"


int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "test_depth_request");
    ros::NodeHandle nh;

    while(ros::ok())
    {
        ros::spinOnce();
        sleep(1);


        ros::ServiceClient depth_request_client;
        depth_request_client = nh.serviceClient<stereo_process::DepthRequest>("depth_request");

        stereo_process::DepthRequest srv;
        srv.request.u = 400;
        srv.request.v = 400;

        if(depth_request_client.call(srv))
        {
            ROS_INFO("Depth received is:%f for u:%d, v:%d", srv.response.depth,srv.request.u,srv.request.v);
        }
        else
        {
            ROS_ERROR("Failed to call service DepthRequest");
        }
    }

    return 0;
}

