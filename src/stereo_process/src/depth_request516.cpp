///////////////////////////////////////////////////////////////////////////////////////////////
//// Name         : bumblebee_pcl_process.cpp                                              ////
//// Description  : Executable ROS .cpp file containing the detection with Bumblebee PCL   ////
//// Function     : This program receives the Point Cloud data published by the stereo_    ////
////                image_proc node after the process of the bumblebee stereo camera image ////
////                stream and processes the point cloud to extract the surface plane and  ////
////                segment the surrounding environment to sters to recognize objects   ////
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
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Imu.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/point_types_conversion.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/octree/octree.h>
#include <pcl/surface/gp3.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/kdtree/kdtree.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <cv.hpp>
#include "cv_bridge/cv_bridge.h"
#include <image_transport/image_transport.h>
#include "image_geometry/pinhole_camera_model.h"
#include <sensor_msgs/image_encodings.h>

//#include "bumblebee_pcl_process.h"
#include "stereo_process/ObjectOnImage.h"
#include "stereo_process/Ball.h"
#include "stereo_process/Obstacles.h"
#include "stereo_process/Goalpost.h"
#include "stereo_process/DepthRequest.h"
#include "boost/array.hpp"

#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/BoundingBox.h>

#include <vision/Ball.h>
#include <vision/Landmarks.h>
#include <head_motion/head_pose.h>
#include <vision/Goalpost.h>


#define MIN_CLUSTER_SIZE 15
#define CAMERA_HEIGHT 1.27

//camera parameters
double fx = 700.178;
double PI = 3.1415926;

class PointCloudProcess
{
    ros::NodeHandle pcl_nh;

    image_transport::ImageTransport it_;

    ros::Publisher pub, pub_above_field,pub_plane,pub_cluster_field,pub_field,pub_cluster_ball,pub_ball,pub_cluster_obstacle_1,pub_cluster_obstacle_2,
    pub_cluster_obstacle_3,pub_cluster_obstacle_4,pub_cluster_obstacle_5,pub_obstacle,pub_cluster_goalpost,pub_goalpost;

    //    ros::Publisher pub_vision_ball;
    // Create a ROS subscriber for the input point cloud and imu orientation
    ros::Subscriber sub_cloud, sub_imu, sub_image_info;
    ros::ServiceServer depth_request_server;
    ros::Subscriber sub_detection_boxes;
    ros::Publisher ball_msg_pub,landmarks_msg_pub,goalpost_msg_pub;

    ros::Subscriber sub_found_object;
    ros::Subscriber head_pose;

    //msg
    vision::Ball ball_msg;
    vision::Landmarks landmarks_msg;
    vision::Goalpost goalpost_msg;
    int landmark_number_counter;
    int goalpost_number_counter;
    darknet_ros_msgs::BoundingBoxes BBox_array;
    darknet_ros_msgs::BoundingBox BBox;

    cv::Point2i BBox_center;
    bool found_ball;
    bool found_goalpost;
    bool found_landmark;

    std::vector<double> goalpost_temp_bearing;
    std::vector<double> goalpost_temp_range;


public:

    // transform declarations
    tf::TransformBroadcaster odom_broadcaster, lidar_broadcaster;
    tf::TransformListener listener;
    tf::StampedTransform odom;
    tf::StampedTransform lidar, init_lidar;


    tf::Quaternion imu_q;
    double init_imu_roll, init_imu_pitch, init_imu_yaw;
    bool first_imu_orient;
    double pitch_dif, yaw_dif;
    double pc_timestamp;

    image_geometry::PinholeCameraModel cam_model_;
    sensor_msgs::PointCloud2Ptr current_pcl;

    bool pcl_has_data;

    sensor_msgs::CameraInfo camera_info;

    PointCloudProcess()
        : it_(pcl_nh)
    {
        sub_cloud = pcl_nh.subscribe ("/zed/point_cloud/cloud_registered", 1, &PointCloudProcess::cloudCb,this);// /zed/point_cloud/cloud_registered
        //sub_found_object = pcl_nh.subscribe("/darknet_ros/found_object", 1, &PointCloudProcess::ObjectFoundCb, this);
        sub_detection_boxes = pcl_nh.subscribe("/darknet_ros/bounding_boxes", 1, &PointCloudProcess::detectionCb, this);
        //sub_image_info = pcl_nh.subscribe("/zed/rgb/camera_info", 1, &PointCloudProcess::imageCb, this); // check this topic's name

        head_pose = pcl_nh.subscribe ("/decision/head_command", 1, &PointCloudProcess::headCb,this);
        // Create ROS publishers for the output point cloud
        pub = pcl_nh.advertise<sensor_msgs::PointCloud2> ("output", 1);

        ball_msg_pub = pcl_nh.advertise<vision::Ball>("/vision/ball", 1);
        landmarks_msg_pub = pcl_nh.advertise<vision::Landmarks>("/vision/landmarks", 1);
        goalpost_msg_pub = pcl_nh.advertise<vision::Goalpost>("/vision/goalpost", 1);

        //        depth_request_server = pcl_nh.advertiseService("depth_request", &PointCloudProcess::depthRequestSrv, this);
        ROS_INFO("Ready to send out the response to service");

        pcl_has_data = false;

        landmark_number_counter = 0;

        pitch_dif = 0.0;
        //        yaw_dif = 0.0;

        pc_timestamp = ros::Time::now().toSec();

        //        ROS_INFO("bumblebee_pcl_process: Started node!");

        //camera_info
        camera_info.height = 720;

        camera_info.width = 1280;
        camera_info.distortion_model = "plumb_bob";
        std::vector<double> a;
        a.push_back(0.0);a.push_back(0.0);a.push_back(0.0);a.push_back(0.0);a.push_back(0.0);
        camera_info.D = a;

        boost::array<double, 9ul> b;
        b.at(0) = 682.5270385742188; b.at(1) = 0.0; b.at(2) = 667.1209106445312;
        b.at(3) = 0.0; b.at(4) = 682.5270385742188; b.at(5) = 385.3529052734375;
        b.at(6) = 0.0; b.at(7) = 0.0; b.at(8) = 1.0;
        camera_info.K = b;

        b.at(0) = 1.0; b.at(1) = 0.0; b.at(2) = 0.0;
        b.at(3) = 0.0; b.at(4) = 1.0; b.at(5) = 0.0;
        b.at(6) = 0.0; b.at(7) = 0.0; b.at(8) = 1.0;
        camera_info.R = b;

        boost::array<double, 12ul> c;
        c.at(0) = 682.5270385742188; c.at(1) = 0.0; c.at(2) = 667.1209106445312; c.at(3) = 0.0;
        c.at(4) = 0.0; c.at(5) = 682.5270385742188; c.at(6) = 385.3529052734375; c.at(7) = 0.0;
        c.at(8) = 0.0; c.at(9) = 0.0; c.at(10) = 1.0; c.at(11) = 0.0;
        camera_info.P = c;

        camera_info.binning_x = 0;
        camera_info.binning_y = 0;
        camera_info.roi.x_offset = 0;
        camera_info.roi.y_offset = 0;
        camera_info.roi.height= 0;
        camera_info.roi.width= 0;
        camera_info.roi.do_rectify= false;

        cam_model_.fromCameraInfo(camera_info);

        //Initialize ball msg
        ball_msg.header.stamp = ros::Time::now();
        ball_msg.ball_detected = false;
        ball_msg.ball_range = 0.0;
        ball_msg.ball_bearing = 0.0;
        ball_msg.ball_center_x = 0;
        ball_msg.ball_center_y = 0;

        //ROS_INFO("Ready.........................1");

        //Initialize landmark msg
        landmarks_msg.landmark_number = 0;

        //Initialize goalpost msg
        goalpost_msg.header.stamp = ros::Time::now();
        goalpost_msg.goalpost_number = 0;
        goalpost_msg.goalpost_detected = false;
        goalpost_msg.goalpost_left_range = 0.0;
        goalpost_msg.goalpost_left_bearing = 0.0;
        goalpost_msg.goalpost_right_range = 0.0;
        goalpost_msg.goalpost_right_bearing = 0.0;

    }

    ~PointCloudProcess()
    {

    }


    void headCb(const head_motion::head_pose::ConstPtr& msg)
    {
        pitch_dif = msg->pitch;
    }

    void detectionCb(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg)
    {
        found_ball = false;
        found_goalpost = false;
        found_landmark = false;
        landmark_number_counter = 0;

        goalpost_temp_bearing.clear();
        goalpost_temp_range.clear();

        goalpost_number_counter = 0;
        double temp_bearing;

        cv::Point3d pt_cv,pcl_pt;
        //ROS_INFO("Ready.........................01");
        // Use current_pcl to find the depth ?
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
        // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
        double point_range = 0;
        double point_depth = 0;

        if(pcl_has_data)
        {
            pcl::fromROSMsg(*current_pcl,*cloud);



            for(int i=0; i< msg->bounding_boxes.size(); i++)
            {
                const darknet_ros_msgs::BoundingBox &BBox = msg->bounding_boxes[i];

                //found ball, then calculate the bearing and range
                if(BBox.Class == "Ball")
                {
                    found_ball = true;
                    BBox_center.x = (BBox.xmax - BBox.xmin) / 2 + BBox.xmin;
                    BBox_center.y = (BBox.ymax - BBox.ymin) / 2 + BBox.ymin;

                    std::cout << BBox.Class <<" detected." << " Center position is "<< BBox_center.x << "," << BBox_center.y << std::endl;
                    cv::Point2d uv(BBox_center.x,BBox_center.y);

                    //pt_cv = cam_model_.projectPixelTo3dRay(uv);

                    if (cloud->isOrganized())
                    {
                        while (!((point_range>0)&&(point_range<100)))
                        {
                            //ROS_INFO("Before use, cloud x range:%f, y range:%f",cloud->width, cloud->height);
                            pcl_pt.x = cloud->at(BBox_center.x,BBox_center.y).y;
                            pcl_pt.y = -cloud->at(BBox_center.x,BBox_center.y).z;
                            pcl_pt.z = cloud->at(BBox_center.x,BBox_center.y).x;
                            //ROS_INFO("After use of x,y");
                            //pitch_dif = 25;
                            point_depth = (pcl_pt.z) * cos(pitch_dif/180*M_PI) - pcl_pt.y * sin(pitch_dif/180*M_PI);
                            //point_range = sqrt(pow(pcl_pt.x,2)+pow(pcl_pt.z,2));
                            point_range = sqrt(pow(pcl_pt.x,2)+pow(point_depth,2));
                            //ROS_INFO("Temp point range %f with image x:%d,y:%d",point_range,image_x,image_y);

                            BBox_center.x += 1;
                            BBox_center.y += 1;

                            //                        if (pitch_dif>50)
                            //                        {
                            //                            point_range = 2*point_range - 0.56; // after studying the real offset
                            //                            if (point_range<0)
                            //                                point_range = 0;
                            //                        }
                        }

                    }

                    //publish ball message to ROS
                    ball_msg.header.stamp = ros::Time::now();
                    ball_msg.ball_detected = true;
                    ball_msg.ball_range = point_range;

                    if(BBox_center.x >= 640)
                    {
                        //bearing direction is positive
                        ball_msg.ball_bearing = atan((BBox_center.x - 640)*4.0/fx);
                    }
                    else
                    {
                        //bearing direction is negative
                        ball_msg.ball_bearing = -atan((640 - BBox_center.x)*4.0/fx);

                    }
                    if (pitch_dif>50)
                    {
                        ball_msg.ball_range = ball_msg.ball_range - 0.1; // after studying the real offset
                        if (point_range<0)
                            ball_msg.ball_range = 0;
                        ball_msg.ball_bearing = ball_msg.ball_bearing -35/180*M_PI;
                    }

                    ROS_ERROR("Corrected ball bearing is %f, range is %f,front_x:%f, front_y:%f", ball_msg.ball_bearing*180/PI, ball_msg.ball_range,
                              ball_msg.ball_range*cos(ball_msg.ball_bearing),ball_msg.ball_range*sin(ball_msg.ball_bearing));


                    ball_msg_pub.publish(ball_msg);


                }


                if( (BBox.Class == "T_cross") || (BBox.Class == "L_cross") || (BBox.Class == "X_cross")
                        || (BBox.Class == "Penalty_point") )

                {
                    found_landmark = true;


                    BBox_center.x = (BBox.xmax - BBox.xmin) / 2 + BBox.xmin;
                    BBox_center.y = (BBox.ymax - BBox.ymin) / 2 + BBox.ymin;

                    std::cout << BBox.Class <<" detected." << " Center position is "<< BBox_center.x << "," << BBox_center.y << std::endl;
                    cv::Point2d uv(BBox_center.x,BBox_center.y);

                    //pt_cv = cam_model_.projectPixelTo3dRay(uv);

                    if (cloud->isOrganized())
                    {
                        while (!((point_range>0)&&(point_range<100)))
                        {
                            //ROS_INFO("Before use, cloud x range:%f, y range:%f",cloud->width, cloud->height);
                            pcl_pt.x = cloud->at(BBox_center.x,BBox_center.y).y;
                            pcl_pt.y = -cloud->at(BBox_center.x,BBox_center.y).z;
                            pcl_pt.z = cloud->at(BBox_center.x,BBox_center.y).x;
                            //ROS_INFO("After use of x,y");
                            //pitch_dif = 25;
                            point_depth = (pcl_pt.z) * cos(pitch_dif/180*M_PI) - pcl_pt.y * sin(pitch_dif/180*M_PI);
                            //point_range = sqrt(pow(pcl_pt.x,2)+pow(pcl_pt.z,2));
                            point_range = sqrt(pow(pcl_pt.x,2)+pow(point_depth,2));
                            //ROS_INFO("Temp point range %f with image x:%d,y:%d",point_range,image_x,image_y);

                            BBox_center.x += 1;
                            BBox_center.y += 1;

                        }

                    }

                    landmarks_msg.landmark_range[landmark_number_counter] = point_range;

                    if(BBox_center.x >= 640)
                    {
                        //bearing direction is positive
                        landmarks_msg.landmark_bearing[landmark_number_counter] = atan((BBox_center.x - 640)*4.0/fx);
                    }
                    else
                    {
                        //bearing direction is negative
                        landmarks_msg.landmark_bearing[landmark_number_counter] = -atan((640 - BBox_center.x)*4.0/fx);

                    }
                    landmarks_msg.landmark_confidence[landmark_number_counter] = 1;
                    if (BBox.Class == "T_cross")
                        landmarks_msg.landmark_type[landmark_number_counter] = 2;
                    else if (BBox.Class == "X_cross")
                        landmarks_msg.landmark_type[landmark_number_counter] = 1;
                    else if (BBox.Class == "L_cross")
                        landmarks_msg.landmark_type[landmark_number_counter] = 3;
                    else if (BBox.Class == "Penalty_point")
                        landmarks_msg.landmark_type[landmark_number_counter] = 5;

                    ROS_ERROR("Landmark bearing is %f, range is %f, type:%d",
                              landmarks_msg.landmark_bearing[landmark_number_counter]*180/PI,
                              landmarks_msg.landmark_range[landmark_number_counter],
                              landmarks_msg.landmark_type[landmark_number_counter]);

                    landmark_number_counter++;
                }

                if(BBox.Class == "Goalpost")
                {
                    found_goalpost = true;

                    BBox_center.x = (BBox.xmax - BBox.xmin) / 2 + BBox.xmin;
                    BBox_center.y = (BBox.ymax - BBox.ymin) / 2 + BBox.ymin;

                    std::cout << BBox.Class <<" detected." << " Center position is "<< BBox_center.x << "," << BBox_center.y << std::endl;
                    cv::Point2d uv(BBox_center.x,BBox_center.y);

                    //pt_cv = cam_model_.projectPixelTo3dRay(uv);

                    if (cloud->isOrganized())
                    {
                        while (!((point_range>0)&&(point_range<100)))
                        {
                            //ROS_INFO("Before use, cloud x range:%f, y range:%f",cloud->width, cloud->height);
                            pcl_pt.x = cloud->at(BBox_center.x,BBox_center.y).y;
                            pcl_pt.y = -cloud->at(BBox_center.x,BBox_center.y).z;
                            pcl_pt.z = cloud->at(BBox_center.x,BBox_center.y).x;
                            //ROS_INFO("After use of x,y");
                            //pitch_dif = 25;
                            point_depth = (pcl_pt.z) * cos(pitch_dif/180*M_PI) - pcl_pt.y * sin(pitch_dif/180*M_PI);
                            //point_range = sqrt(pow(pcl_pt.x,2)+pow(pcl_pt.z,2));
                            point_range = sqrt(pow(pcl_pt.x,2)+pow(point_depth,2));
                            //ROS_INFO("Temp point range %f with image x:%d,y:%d",point_range,image_x,image_y);

                            BBox_center.x += 1;
                            BBox_center.y += 1;

                        }

                    }

                    if(BBox_center.x >= 640)
                    {
                        //bearing direction is positive
                        temp_bearing = atan((BBox_center.x - 640)*4.0/fx);
                        goalpost_temp_bearing.push_back(temp_bearing);

                    }
                    else
                    {
                        //bearing direction is negative
                        temp_bearing = -atan((640 - BBox_center.x)*4.0/fx);
                        goalpost_temp_bearing.push_back(temp_bearing);

                    }

                    goalpost_temp_range.push_back(point_range);
                    goalpost_number_counter++;

                }



                //ROS_ERROR("Requested x:%d,y:%d",req.u,req.v);
                ROS_ERROR("Point response:x:%f, y:%f, z:%f, range:%f req u:%d, req v:%d, pitch:%f, point_depth:%f",pcl_pt.x,
                          pcl_pt.y,pcl_pt.z,point_range,BBox_center.x,BBox_center.y,pitch_dif,point_depth);

                //  }

            }





            if(found_landmark)
            {
                //publish landmarks message to ROS
                landmarks_msg.header.stamp = ros::Time::now();
                landmarks_msg.landmark_number = landmark_number_counter;


            }
            else
            {
                landmarks_msg.landmark_number = 0;
            }



            landmarks_msg_pub.publish(landmarks_msg);

            //publish goalpost info to ROS
            if(found_goalpost)
            {
                //publish landmarks message to ROS
                goalpost_msg.header.stamp = ros::Time::now();
                goalpost_msg.goalpost_detected = true;
                goalpost_msg.goalpost_number = goalpost_number_counter;

                if(goalpost_number_counter == 2)
                {
                    if(goalpost_temp_bearing[0] < goalpost_temp_bearing[1])
                    {
                        goalpost_msg.goalpost_left_bearing = goalpost_temp_bearing[0];
                        goalpost_msg.goalpost_left_range = goalpost_temp_range[0];

                        goalpost_msg.goalpost_right_bearing = goalpost_temp_bearing[1];
                        goalpost_msg.goalpost_right_range = goalpost_temp_range[1];

                    }
                    else if(goalpost_temp_bearing[0] > goalpost_temp_bearing[1])
                    {
                        goalpost_msg.goalpost_right_bearing = goalpost_temp_bearing[0];
                        goalpost_msg.goalpost_right_range = goalpost_temp_range[0];

                        goalpost_msg.goalpost_left_bearing = goalpost_temp_bearing[1];
                        goalpost_msg.goalpost_left_range = goalpost_temp_range[1];

                    }

                }
                else if(goalpost_number_counter == 1)
                {
                    goalpost_msg.goalpost_left_bearing = goalpost_temp_bearing[0];
                    goalpost_msg.goalpost_left_range = goalpost_temp_range[0];

                }

                ROS_INFO("%d goalpost founded", goalpost_number_counter);

            }
            else
            {
                goalpost_msg.goalpost_number = 0;
            }

            goalpost_msg_pub.publish(goalpost_msg);




        }

    }

    // Callback function for the process of the point cloud from the bumblebee camera
    void cloudCb (const sensor_msgs::PointCloud2Ptr& input)
    {
        pc_timestamp = input->header.stamp.toSec();

        //ROS_INFO("Ready.........................cloud");

        if (input->width == 0)
        {
            ROS_INFO("pcl_process: No points to process in Point Cloud");
            pcl_has_data = false;
            return;
        }

        current_pcl = input;
        pcl_has_data = true;

        //ROS_INFO("New pcl data");


    }

};

int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "bumblebee_pcl_process");
    ros::NodeHandle nh;
    //ros::AsyncSpinner spinner(0);



    PointCloudProcess bumblebee_pcl_process;

    if (strcmp(argv[1],"25")==0)
        bumblebee_pcl_process.pitch_dif = 25;
    else if (strcmp(argv[1],"60")==0)
        bumblebee_pcl_process.pitch_dif = 60;

    while(ros::ok())
    {
        ros::spinOnce();
        //spinner.start();
        //rate.sleep(); // check if we can add the delay here to control the frequency or if this stalls even responses to service requests
        //        if (bumblebee_pcl_process.pcl_has_data)
        //            bumblebee_pcl_process.cloudProcess();
    }

    //spinner.stop();
    return 0;
}

