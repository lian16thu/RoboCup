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
#include <decision/SerialReceived.h>


#define MIN_CLUSTER_SIZE 15
#define CAMERA_HEIGHT 1.27

#define D_CAM_LEFT 0.06
#define HEAD_YAW_OFFSET 0.12

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
    ros::Subscriber head_pose,serial_listener;

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
    double head_pitch, head_yaw;
    double pc_timestamp;

    image_geometry::PinholeCameraModel cam_model_;
    sensor_msgs::PointCloud2Ptr current_pcl;

    bool pcl_has_data;

    sensor_msgs::CameraInfo camera_info;

    double front_to_waist_x,front_to_waist_y;

    PointCloudProcess()
        : it_(pcl_nh)
    {
        sub_cloud = pcl_nh.subscribe ("/zed/point_cloud/cloud_registered", 1, &PointCloudProcess::cloudCb,this);// /zed/point_cloud/cloud_registered
        //sub_found_object = pcl_nh.subscribe("/darknet_ros/found_object", 1, &PointCloudProcess::ObjectFoundCb, this);
        sub_detection_boxes = pcl_nh.subscribe("/darknet_ros/bounding_boxes", 1, &PointCloudProcess::detectionCb, this);
        //sub_image_info = pcl_nh.subscribe("/zed/rgb/camera_info", 1, &PointCloudProcess::imageCb, this); // check this topic's name

        head_pose = pcl_nh.subscribe ("/decision/head_command", 1, &PointCloudProcess::headCb,this);
        serial_listener = pcl_nh.subscribe ("/decision/serial_receiver", 1, &PointCloudProcess::serialCb,this);
        // Create ROS publishers for the output point cloud
        pub = pcl_nh.advertise<sensor_msgs::PointCloud2> ("output", 1);

        ball_msg_pub = pcl_nh.advertise<vision::Ball>("/vision/ball", 1);
        landmarks_msg_pub = pcl_nh.advertise<vision::Landmarks>("/vision/landmarks", 1);
        goalpost_msg_pub = pcl_nh.advertise<vision::Goalpost>("/vision/goalpost", 1);

        //        depth_request_server = pcl_nh.advertiseService("depth_request", &PointCloudProcess::depthRequestSrv, this);
        ROS_INFO("Ready to send out the response to service");

        pcl_has_data = false;

        landmark_number_counter = 0;

        head_pitch = 0.0;
        head_yaw = 0.0;

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

    void serialCb(const decision::SerialReceived::ConstPtr serial_input_msg)
    {
        front_to_waist_x = serial_input_msg->received_data[5];;
        front_to_waist_y = serial_input_msg->received_data[6];;

    }


    void headCb(const head_motion::head_pose::ConstPtr& msg)
    {
        //ROS_ERROR("head_pitch:%d, head_yaw:%d",msg->pitch,msg->yaw);
        head_pitch = ((double)(msg->pitch))/180*M_PI;
        //head_yaw = ((double)(msg->yaw))/180*M_PI;
        head_yaw = ((double)(msg->yaw))/180*M_PI;

        //ROS_ERROR("head_pitch:%f, head_yaw:%f", head_pitch,head_yaw);


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
                    //found_ball = true;
                    BBox_center.x = (BBox.xmax - BBox.xmin) / 2 + BBox.xmin;
                    BBox_center.y = (BBox.ymax - BBox.ymin) / 2 + BBox.ymin;

                    std::cout << BBox.Class <<" detected." << " Center position is "<< BBox_center.x << "," << BBox_center.y << std::endl;
                    cv::Point2d uv(BBox_center.x,BBox_center.y);

                    //pt_cv = cam_model_.projectPixelTo3dRay(uv);

                    if(head_pitch < 50.0/180*M_PI)
                    {

                        double temp_head_yaw = head_yaw + HEAD_YAW_OFFSET;

                        if (cloud->isOrganized())
                        {
                            while (!((point_range>0)&&(point_range<13))
                                   && ((BBox_center.x <= BBox.xmax + 10) && (BBox_center.x<1280))
                                   && ((BBox_center.y <= BBox.ymax + 10) && (BBox_center.y<720)))
                            {
                                //ROS_INFO("Before use, cloud x range:%f, y range:%f",cloud->width, cloud->height);
                                pcl_pt.x = cloud->at(BBox_center.x,BBox_center.y).y;
                                pcl_pt.y = -cloud->at(BBox_center.x,BBox_center.y).z;
                                pcl_pt.z = cloud->at(BBox_center.x,BBox_center.y).x;
                                //ROS_INFO("After use of x,y");
                                //head_pitch = 25;

                                if (pcl_pt.z > 1.3)
                                {
                                    found_ball = true;

                                    point_depth = (pcl_pt.z) * cos(pitch_dif/180*M_PI) - pcl_pt.y * sin(pitch_dif/180*M_PI);

                                    //point_range = sqrt(pow(pcl_pt.x,2)+pow(pcl_pt.z,2));
                                    point_range = sqrt(pow(pcl_pt.x,2)+pow(point_depth,2));
                                    //ROS_INFO("Temp point range %f with image x:%d,y:%d",point_range,image_x,image_y);

                                }
                                else
                                {
                                    found_ball = false;
                                    ROS_INFO("Found part of body, not real ball");

                                }
                                BBox_center.x += 1;
                                BBox_center.y += 1;

                            }

                        }

                        //publish ball message to ROS
                        if(found_ball)
                        {
                            //publish ball message to ROS
                            ball_msg.header.stamp = msg->header.stamp;
                            ball_msg.ball_detected = true;
                            ball_msg.ball_range = point_range;

                            if(BBox_center.x >= 640)
                            {
                                //bearing direction is positive
                                ball_msg.ball_bearing = -atan((BBox_center.x - 640)/fx);
                            }
                            else
                            {
                                //bearing direction is negative
                                ball_msg.ball_bearing = atan((640 - BBox_center.x)/fx);

                            }

                            ROS_ERROR("Ball before correction to front, range:%f, bearing:%f",ball_msg.ball_range,ball_msg.ball_bearing);

                            ROS_ERROR("f2w_x:%f, f2w_y:%f, head_pitch:%f, head_yaw:%f",front_to_waist_x,front_to_waist_y,head_pitch,temp_head_yaw);
                            double front_to_camera_center_x = front_to_waist_x + 0.16 * sin(head_pitch) * cos(temp_head_yaw);
                            double camera_center_to_left_x = - D_CAM_LEFT *sin(temp_head_yaw) + ball_msg.ball_range * cos(temp_head_yaw+ball_msg.ball_bearing);
                            double front_to_camera_center_y = front_to_waist_y + 0.16 * sin(head_pitch) * sin(temp_head_yaw);
                            double camera_center_to_left_y = + D_CAM_LEFT *cos(temp_head_yaw) + ball_msg.ball_range * sin(temp_head_yaw+ball_msg.ball_bearing);

                            double front_to_ball_x = front_to_camera_center_x + camera_center_to_left_x;
                            double front_to_ball_y = front_to_camera_center_y + camera_center_to_left_y;

                            ball_msg.ball_range = sqrt(pow(front_to_ball_x,2)+pow(front_to_ball_y,2));
                            ball_msg.ball_bearing = atan2(front_to_ball_y,front_to_ball_x);

                            ROS_ERROR("Corrected ball bearing is %f, range is %f,front_x:%f, front_y:%f", ball_msg.ball_bearing*180/PI, ball_msg.ball_range,
                                      ball_msg.ball_range*cos(ball_msg.ball_bearing),ball_msg.ball_range*sin(ball_msg.ball_bearing));

                        //                        ROS_ERROR("front_to_camera_center_x:%f, front_to_camera_center_y:%f, camera_center_to_left_x:%f, camera_center_to_left_y:%f",
                        //                                  front_to_camera_center_x,front_to_camera_center_y,camera_center_to_left_x,camera_center_to_left_y);

                            ball_msg.header.stamp = msg->header.stamp;
                            ball_msg_pub.publish(ball_msg);

                        }
                    }
                    else
                    {
                        //in kick position
                        cv::Mat coord_2d(3, 1, CV_64FC1);
                        cv::Mat coord_3d(3, 1, CV_64FC1);
                        cv::Point2f ball_position;
                        cv::Mat homo(3, 3, CV_64FC1);

                        double ball_radius = 0.1f;

//                        point_range = 0;
//                        point_depth = 0;

                        coord_2d.at<double>(0) = BBox_center.x;
                        coord_2d.at<double>(1) = BBox_center.y;
                        coord_2d.at<double>(2) = 1;

                        found_ball = false;

                        if (cloud->isOrganized())
                        {
                            while (!((point_range>0)&&(point_range<13))&&((BBox_center.x <= BBox.xmax+10) && (BBox_center.x < 1280))&&((BBox_center.y <= BBox.ymax+10) && (BBox_center.y < 720)))
                            {
                                //ROS_INFO("Before use, cloud x range:%f, y range:%f",cloud->width, cloud->height);
                                pcl_pt.x = cloud->at(BBox_center.x,BBox_center.y).y;
                                pcl_pt.y = -cloud->at(BBox_center.x,BBox_center.y).z;
                                pcl_pt.z = cloud->at(BBox_center.x,BBox_center.y).x;

                                if (pcl_pt.z > 1.3)
                                {
                                    found_ball = true;

//                                    point_depth = (pcl_pt.z) * cos(pitch_dif/180*M_PI) - pcl_pt.y * sin(pitch_dif/180*M_PI);

//                                    //point_range = sqrt(pow(pcl_pt.x,2)+pow(pcl_pt.z,2));
//                                    point_range = sqrt(pow(pcl_pt.x,2)+pow(point_depth,2));
//                                    //ROS_INFO("Temp point range %f with image x:%d,y:%d",point_range,image_x,image_y);

                                    BBox_center.x += 1;
                                    BBox_center.y += 1;


                                }
                                else
                                {
                                    found_ball = false;
                                    ROS_ERROR("Found part of body, not real ball");
                                    break;

                                }

                            }

                        }

                        if(found_ball)
                        {
                            homo.at<double>(0,0) = 0.270078;
                            homo.at<double>(1,0) = -0.00865311;
                            homo.at<double>(2,0) = 2.14638e-06;
                            homo.at<double>(0,1) = 0.0633644;
                            homo.at<double>(1,1) = 0.341872;
                            homo.at<double>(2,1) = 0.00116483;
                            homo.at<double>(0,2) = -138.097;
                            homo.at<double>(1,2) = -77.0586;
                            homo.at<double>(2,2) = 1;

                            coord_3d.at<double>(0) = (homo.at<double>(0,0) * coord_2d.at<double>(0) + homo.at<double>(0,1) * coord_2d.at<double>(1) + homo.at<double>(0,2) * coord_2d.at<double>(2));
                            coord_3d.at<double>(1) = (homo.at<double>(1,0) * coord_2d.at<double>(0) + homo.at<double>(1,1) * coord_2d.at<double>(1) + homo.at<double>(1,2) * coord_2d.at<double>(2));
                            coord_3d.at<double>(2) = (homo.at<double>(2,0) * coord_2d.at<double>(0) + homo.at<double>(2,1) * coord_2d.at<double>(1) + homo.at<double>(2,2) * coord_2d.at<double>(2));

                            coord_3d.at<double>(0) = coord_3d.at<double>(0) / coord_3d.at<double>(2);
                            coord_3d.at<double>(1) = coord_3d.at<double>(1) / coord_3d.at<double>(2);

                            //x coordinate w.r.t robot home position
                            ball_position.x = -(coord_3d.at<double>(1) - 92);
                            //y coordinate w.r.t robot home position
                            ball_position.y = (coord_3d.at<double>(0) - 54);

                            std::cout << "Ball center in robot coord system before correction :" << ball_position.x/100 << "," << ball_position.y/100 <<std::endl;

                            double ball_bearing;
                            double ball_range;

                            ball_bearing = - atan2(ball_position.y , ball_position.x);
                            ball_range = sqrt(ball_position.x * ball_position.x + ball_position.y * ball_position.y) / 100;//  - ball_radius;

                            ROS_ERROR("Before correction :%f,b:%f",ball_range,ball_bearing);

                            double front_to_ball_x = front_to_waist_x * (1-cos(head_yaw)) + front_to_waist_y*sin(head_yaw)
                                    + ball_range * cos(ball_bearing+head_yaw);

                            double front_to_ball_y = front_to_waist_y * (1-cos(head_yaw)) - front_to_waist_x*sin(head_yaw)
                                    + ball_range * sin(ball_bearing+head_yaw);

                            ball_bearing = atan2(front_to_ball_y, front_to_ball_x);
                            ball_range = sqrt(front_to_ball_x * front_to_ball_x + front_to_ball_y * front_to_ball_y);//  - ball_radius;

                            ROS_ERROR("After correction :%f,b:%f",ball_range,ball_bearing);

                            std::cout << "Ball center in robot coord system after correction :" << front_to_ball_x << "," << front_to_ball_y <<std::endl;

                            //cout << "ready for kick ball position is (bearing, range) is (" << ball_bearing << ", " << ball_range <<")" << endl;


                            //cout << "ready for kick ball position is (bearing, range) is (" << ball_bearing << ", " << ball_range <<")" << endl;

                            ball_msg.ball_detected = true;
                            ball_msg.ball_bearing = ball_bearing;
                            ball_msg.ball_range = ball_range;
                            ball_msg.ball_center_x = front_to_ball_x;
                            ball_msg.ball_center_y = front_to_ball_y;

                            ROS_ERROR("Ball detected, ready for kick position is %f, %f", ball_msg.ball_bearing*180/PI, ball_msg.ball_range);

                            ball_msg.header.stamp = msg->header.stamp;
                            ball_msg_pub.publish(ball_msg);

                        }
                    }


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
                        while (!((point_range>0)&&(point_range<13))
                               && ((BBox_center.x <= BBox.xmax + 10) && (BBox_center.x<1280))
                               && ((BBox_center.y <= BBox.ymax + 10) && (BBox_center.y<720)))
                        {
                            //ROS_INFO("Before use, cloud x range:%f, y range:%f",cloud->width, cloud->height);
                            pcl_pt.x = cloud->at(BBox_center.x,BBox_center.y).y;
                            pcl_pt.y = -cloud->at(BBox_center.x,BBox_center.y).z;
                            pcl_pt.z = cloud->at(BBox_center.x,BBox_center.y).x;
                            //ROS_INFO("After use of x,y");
                            //head_pitch = 25;
                            point_depth = (pcl_pt.z) * cos(head_pitch) - pcl_pt.y * sin(head_pitch);
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
                        landmarks_msg.landmark_bearing[landmark_number_counter] = atan((BBox_center.x - 640)/fx);
                    }
                    else
                    {
                        //bearing direction is negative
                        landmarks_msg.landmark_bearing[landmark_number_counter] = -atan((640 - BBox_center.x)/fx);

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
                    //BBox_center.y = (BBox.ymax - BBox.ymin) / 2 + BBox.ymin;
                    BBox_center.y = BBox.ymax;

                    std::cout << BBox.Class <<" detected." << " Center position is "<< BBox_center.x << "," << BBox_center.y << std::endl;
                    cv::Point2d uv(BBox_center.x,BBox_center.y);

                    //pt_cv = cam_model_.projectPixelTo3dRay(uv);

                    if (cloud->isOrganized())
                    {
                        while (!((point_range>0)&&(point_range<13))
                               && ((BBox_center.x <= BBox.xmax + 10) && (BBox_center.x<1280))
                               && ((BBox_center.y <= BBox.ymax + 10) && (BBox_center.y<720)))
                        {
                            //ROS_INFO("Before use, cloud x range:%f, y range:%f",cloud->width, cloud->height);
                            pcl_pt.x = cloud->at(BBox_center.x,BBox_center.y).y;
                            pcl_pt.y = -cloud->at(BBox_center.x,BBox_center.y).z;
                            pcl_pt.z = cloud->at(BBox_center.x,BBox_center.y).x;
                            //ROS_INFO("After use of x,y");
                            //head_pitch = 25;
                            point_depth = (pcl_pt.z) * cos(head_pitch) - pcl_pt.y * sin(head_pitch);
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
                        temp_bearing = atan((BBox_center.x - 640)/fx);
                        goalpost_temp_bearing.push_back(temp_bearing);

                    }
                    else
                    {
                        //bearing direction is negative
                        temp_bearing = -atan((640 - BBox_center.x)/fx);
                        goalpost_temp_bearing.push_back(temp_bearing);

                    }

                    if((BBox.xmin == 0) && (BBox.ymin == 0))
                    {
                        //right goalpost is seen
                        goalpost_number_counter = 2;
                    }
                    else if((BBox.xmax == 1280) && (BBox.ymin == 0))
                    {
                        //left goalpost is seen
                        goalpost_number_counter = 1;

                    }
                    else
                    {
                        //whole goalpost is seen
                        goalpost_number_counter = 3;

                    }

                    goalpost_temp_range.push_back(point_range);
                    //goalpost_number_counter++;

                }



                //ROS_ERROR("Requested x:%d,y:%d",req.u,req.v);
                ROS_ERROR("Point response:x:%f, y:%f, z:%f, range:%f req u:%d, req v:%d, pitch:%f, point_depth:%f",pcl_pt.x,
                          pcl_pt.y,pcl_pt.z,point_range,BBox_center.x,BBox_center.y,head_pitch,point_depth);

                //  }

            }





            if(found_landmark)
            {
                //publish landmarks message to ROS
                landmarks_msg.header.stamp = msg->header.stamp;
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
                goalpost_msg.header.stamp = msg->header.stamp;
                goalpost_msg.goalpost_detected = true;
                goalpost_msg.goalpost_number = goalpost_number_counter;

                goalpost_msg.goalpost_center_bearing = goalpost_temp_bearing[0];
                goalpost_msg.goalpost_center_range = goalpost_temp_range[0];

                ROS_INFO("%d goalpost found", goalpost_number_counter);

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
            //return;
        }
        else
        {
            current_pcl = input;
            pcl_has_data = true;
        }


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
        bumblebee_pcl_process.head_pitch = 25.0/180*M_PI;
    else if (strcmp(argv[1],"57")==0)
        bumblebee_pcl_process.head_pitch = 57.0/180*M_PI;

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


