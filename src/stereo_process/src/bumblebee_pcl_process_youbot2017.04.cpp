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
//// Last update  : 2017.04.17                                                             ////
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
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/image_encodings.h>

#include "bumblebee_pcl_process.h"
#include "stereo_process/Ball.h"
#include "stereo_process/Obstacles.h"
#include "stereo_process/Goalpost.h"
#include "stereo_process/ObjectOnImage.h"
#include "stereo_process/DepthRequest.h"


#define MIN_CLUSTER_SIZE 15

class PointCloudProcess
{
    ros::NodeHandle pcl_nh;

    image_transport::ImageTransport it_;

    ros::Publisher pub, pub_above_field,pub_plane,pub_cluster_field,pub_field,pub_cluster_ball,pub_ball,pub_cluster_obstacle_1,pub_cluster_obstacle_2,
    pub_cluster_obstacle_3,pub_cluster_obstacle_4,pub_obstacle,pub_cluster_goalpost,pub_goalpost;

    // Create a ROS subscriber for the input point cloud and imu orientation
    ros::Subscriber sub_cloud, sub_imu;
    image_transport::CameraSubscriber sub_image_info;

    ros::ServiceServer depth_request_server;

public:

    // transform declarations
    tf::TransformBroadcaster odom_broadcaster, lidar_broadcaster;
    tf::TransformListener listener;
    tf::StampedTransform odom;
    tf::StampedTransform lidar, init_lidar;

    stereo_process::Ball ball;
    stereo_process::Obstacles obstacles;
    stereo_process::Goalpost goalpost;


    tf::Quaternion imu_q;
    double init_imu_roll, init_imu_pitch, init_imu_yaw;
    bool first_imu_orient;
    double pc_timestamp;

    image_geometry::PinholeCameraModel cam_model_;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr current_pcl;


    PointCloudProcess()
        : it_(pcl_nh)
    {


        sub_cloud = pcl_nh.subscribe ("/camera/points2", 1, &PointCloudProcess::cloudCb,this);
        sub_imu = pcl_nh.subscribe ("/mti/sensor/imu", 1, &PointCloudProcess::imuCb,this);

        sub_image_info = it_.subscribeCamera("/camera/right/image_rect_color", 1, &PointCloudProcess::imageCb, this); // check this topic's name

        // Create ROS publishers for the output point cloud
        pub = pcl_nh.advertise<sensor_msgs::PointCloud2> ("output", 1);
        pub_above_field = pcl_nh.advertise<sensor_msgs::PointCloud2> ("above_field", 1);
        pub_plane = pcl_nh.advertise<sensor_msgs::PointCloud2> ("plane", 1);
        pub_cluster_field = pcl_nh.advertise<sensor_msgs::PointCloud2> ("field", 1);
        pub_field = pcl_nh.advertise<stereo_process::ObjectOnImage> ("/stereo_process/field", 1);

        pub_cluster_ball = pcl_nh.advertise<sensor_msgs::PointCloud2> ("cluster_ball", 1);
        pub_ball = pcl_nh.advertise<stereo_process::ObjectOnImage> ("/stereo_process/ball", 1);

        pub_cluster_obstacle_1 = pcl_nh.advertise<sensor_msgs::PointCloud2> ("cluster_obstacle_1", 1);
        pub_cluster_obstacle_2 = pcl_nh.advertise<sensor_msgs::PointCloud2> ("cluster_obstacle_2", 1);
        pub_cluster_obstacle_3 = pcl_nh.advertise<sensor_msgs::PointCloud2> ("cluster_obstacle_3", 1);
        pub_cluster_obstacle_4 = pcl_nh.advertise<sensor_msgs::PointCloud2> ("cluster_obstacle_4", 1);
        pub_obstacle = pcl_nh.advertise<stereo_process::ObjectOnImage> ("/stereo_process/obstacle", 1);

        pub_cluster_goalpost = pcl_nh.advertise<sensor_msgs::PointCloud2> ("cluster_goalpost", 1);
        pub_goalpost = pcl_nh.advertise<stereo_process::ObjectOnImage> ("/stereo_process/goalpost", 1);

        depth_request_server = pcl_nh.advertiseService("depth_request", &PointCloudProcess::depthRequestSrv, this);


        odom.stamp_ = ros::Time::now();

        //Intialize all the values for the objects to 0
        ball.header.stamp = odom.stamp_;
        ball.ball_detected = false;
        ball.ball_range = 0.0;
        ball.ball_bearing = 0.0;

        obstacles.header.stamp = odom.stamp_;
        obstacles.obstacles_detected = false;
        obstacles.obstacle_range[0] = obstacles.obstacle_range[1] = obstacles.obstacle_range[2] = obstacles.obstacle_range[3] = 0.0;
        obstacles.obstacle_bearing[0] = obstacles.obstacle_bearing[1] = obstacles.obstacle_bearing[2] = obstacles.obstacle_bearing[3] = 0.0;

        goalpost.header.stamp = odom.stamp_;
        goalpost.goalpost_detected = false;
        goalpost.goalpost_left_range = goalpost.goalpost_right_range = 0.0;
        goalpost.goalpost_left_bearing = goalpost.goalpost_right_bearing = 0.0;


        imu_q.setRPY(0,0,0);
        init_imu_roll = 0.0;
        init_imu_pitch = 0.0;
        init_imu_yaw = 0.0;
        first_imu_orient = true;
        pc_timestamp = ros::Time::now().toSec();

        ROS_INFO("bumblebee_pcl_process: Started node!");


    }

    ~PointCloudProcess()
    {

    }

    stereo_process::ObjectOnImage convert3dTo2d(pcl::PointCloud<pcl::PointXYZRGB> inputPC,stereo_process::ObjectOnImage input2dSet)
    {

        stereo_process::ObjectOnImage output2dSet = input2dSet;
        for (int pnt_count=0;pnt_count<inputPC.size();pnt_count++)
        {
            pcl::PointXYZRGB searchPoint = inputPC.points.at(pnt_count);

            cv::Point3d pt_cv(searchPoint.x, searchPoint.y, searchPoint.z);
            cv::Point2d uv;

            uv = cam_model_.project3dToPixel(pt_cv);

            output2dSet.image_u.push_back(uv.x);
            output2dSet.image_v.push_back(uv.y);
        }
        return output2dSet;
    }

    bool depthRequestSrv(stereo_process::DepthRequest::Request  &req,
                         stereo_process::DepthRequest::Response &res)
    {
        cv::Point3d pt_cv;
        cv::Point2d uv(req.u,req.v);

        pt_cv = cam_model_.projectPixelTo3dRay(uv);

        // Use current_pcl to find the depth ?



        // Or use the already calculated pt_cv.z ?

        res.depth = pt_cv.z;


        return true;
    }

    void imageCb(const sensor_msgs::ImageConstPtr& image_msg,
                 const sensor_msgs::CameraInfoConstPtr& info_msg)
    {
        // Only use the new info if it is close to the newly acquired pointcloud
        if ((pc_timestamp - info_msg->header.stamp.toSec())<0.03)
        {
            cam_model_.fromCameraInfo(info_msg);
            ROS_INFO("Assigned new camera model");
        }
        

    }

    // Callback function that receives the imu data and assigns it to a local variable
    void imuCb (const sensor_msgs::ImuPtr& imu_orient)
    {\

        imu_q.setW(imu_orient->orientation.w);
        imu_q.setX(imu_orient->orientation.x);
        imu_q.setY(imu_orient->orientation.y);
        imu_q.setZ(imu_orient->orientation.z);
        //ROS_INFO("Received Imu data");

        if (first_imu_orient)
        {
            tf::Matrix3x3 init_imu_m(imu_q);
            init_imu_m.getRPY(init_imu_roll, init_imu_pitch, init_imu_yaw);
            first_imu_orient = false;
        }
    }

    // Function that evaluates if a 3D point belongs to the green field
    bool isField (const pcl::PointXYZRGB& ground_point)
    {\
        pcl::PointXYZRGB point_rgb = ground_point;
        pcl::PointXYZHSV point_hsv;
        //Convert the point from RGB to HSV
        pcl::PointXYZRGBtoXYZHSV(point_rgb,point_hsv);
        if ((point_hsv.h>=90 && point_hsv.h<=150)
                && (point_hsv.s>=0.31 && point_hsv.s<=0.69)
                && (point_hsv.v>=0.02 && point_hsv.v<=0.67)
                )
            return true;
        else
        {   //ROS_INFO("Ground point-no field, hsv:(%lf,%lf,%lf)",point_hsv.h,point_hsv.s,point_hsv.v);
            return false;
        }
    }

    // Callback function for the process of the point cloud from the bumblebee camera
    void cloudCb (const sensor_msgs::PointCloud2Ptr& input)
    {
        pc_timestamp = input->header.stamp.toSec();

        if (input->width == 0)
        {
            ROS_INFO("pcl_process: No points to process in Point Cloud");
            return;
        }

        // Create a container for the data.
        sensor_msgs::PointCloud2 temp_pcl,output,output_above_field,output_plane,output_field,
                output_ball,output_obstacle1,output_obstacle2,output_obstacle3,output_obstacle4,
                output_goalpost;

        // All the objects needed
        pcl::PassThrough<pcl::PointXYZRGB> pass;
        pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
        pcl::SACSegmentationFromNormals<pcl::PointXYZRGB, pcl::Normal> seg;
        pcl::PCDWriter writer;
        pcl::ExtractIndices<pcl::PointXYZRGB> extract;
        pcl::ExtractIndices<pcl::Normal> extract_normals;
        pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());

        // Datasets
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZRGB> ());
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloud_field (new pcl::PointCloud<pcl::PointXYZRGB> ());
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_field (new pcl::PointCloud<pcl::PointXYZRGB> ());
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_above_field (new pcl::PointCloud<pcl::PointXYZRGB> ());

        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
        pcl::PointCloud<pcl::Normal>::Ptr cloud_plane_normals (new pcl::PointCloud<pcl::Normal>);
        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2 (new pcl::PointCloud<pcl::Normal>);

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered2 (new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr coefficients_sphere (new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers_sphere (new pcl::PointIndices);
        pcl::PointIndices::Ptr cloud_above_field_indices (new pcl::PointIndices);


        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster_ball (new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster_obstacle1 (new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster_obstacle2 (new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster_obstacle3 (new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster_obstacle4 (new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster_goalpost (new pcl::PointCloud<pcl::PointXYZRGB>);


        /// Initialize point clouds

        // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
        pcl::fromROSMsg(*input,*cloud);

        current_pcl = cloud;

        /// First, rotate according to the camera's orientation given by the Imu

        //ROS_INFO("Rotate by yaw:%f",(-1)*tf::getYaw(lidar_init_q)*180/M_PI);

        tf::Matrix3x3 imu_m(imu_q);
        double imu_roll, imu_pitch, imu_yaw;
        imu_m.getRPY(imu_roll, imu_pitch, imu_yaw);
        double pitch_dif = -(imu_roll-init_imu_roll);
        double yaw_dif = -(imu_pitch-init_imu_pitch);
        //ROS_INFO("Camera rotation from initial pose (r,p,y)=(%f,%f,%f)",pitch_dif*180/M_PI, yaw_dif*180/M_PI, (imu_yaw-init_imu_yaw)*180/M_PI);
        //        //tf::Quaternion reverse_orient_q(tf::createQuaternionFromRPY(0.0,0.0,(-1)*tf::getYaw(lidar_init_q)));
        //        tf::Quaternion reverse_orient_q(tf::createQuaternionFromRPY(0.0,0.0,0.0));
        //        //tf::Quaternion reverse_orient_q(tf::createQuaternionFromRPY(0.0,0.0,-5*M_PI/180));

        //        //tf::Quaternion reverse_orient_q(tf::createQuaternionFromRPY(-imu_roll,-imu_pitch,0.0));
        //        //tf::Quaternion reverse_orient_q(tf::createQuaternionFromRPY(0.0,-(imu_roll-init_imu_roll),-(imu_pitch-init_imu_pitch))); //imu roll is camera pitch, imu pitch is camera yaw, imu yaw is camera roll
        //        //tf::Quaternion reverse_orient_q(tf::createQuaternionFromRPY(0.0,-(imu_roll-init_imu_roll)*M_PI/180,-(imu_pitch-init_imu_pitch)*M_PI/180)); // in rad?

        //        for(int i=0;i<cloud->width;i++)
        //        {
        //            tf::Vector3 oldXYZ(cloud->points[i].x,cloud->points[i].y,cloud->points[i].z);
        //            tf::Vector3 rotatedXYZ(tf::quatRotate(reverse_orient_q,oldXYZ));
        //            cloud->points[i].x = rotatedXYZ.x();
        //            cloud->points[i].y = rotatedXYZ.y();
        //            cloud->points[i].z = rotatedXYZ.z();
        //        }

        //    pcl::PointXYZRGB test_point = cloud->points.back();

        //ROS_INFO("Initial PointCloud has: %d data points, (%f,%f,%f,%f)",cloud->points.size());

        /// Fix the following passthrough filters to work with the /robot_base frame's position
        /// but also consider the orientation of the bicycle in space

        //        // Create the odom transform
        //        try{
        //            // wait to listen the old transform from "/fixed_frame" to "/robot_base"
        //            ros::Time previous_frame_time = odom.stamp_;
        //            listener.waitForTransform("/fixed_frame", "/robot_base",
        //                                      previous_frame_time, ros::Duration(1.0));
        //            listener.lookupTransform("/fixed_frame", "/robot_base",
        //                                     previous_frame_time, odom);
        //        }
        //        catch (tf::TransformException ex){
        //            ROS_ERROR("odom_publisher odom transform:%s",ex.what());
        //        }

        //        ROS_INFO("Robot base from fixed frame:(x=%f,y=%f,z=%f)",odom.getOrigin().getX(),odom.getOrigin().getY(),odom.getOrigin().getZ());

        //        // Build a passthrough filter to remove spurious NaNs
        //        pass.setInputCloud (cloud);
        //        pass.setFilterFieldName("z");
        //        pass.setFilterLimits (odom.getOrigin().getZ()-.1, odom.getOrigin().getZ()+1.0);
        //        pass.filter (*cloud_filtered);

        //        pass.setInputCloud (cloud_filtered);
        //        pass.setFilterFieldName("x");
        //        pass.setFilterLimits (odom.getOrigin().getX()-1.0, odom.getOrigin().getX()+20.0);
        //        pass.filter (*cloud_filtered);

        //        pass.setInputCloud (cloud_filtered);
        //        pass.setFilterFieldName("y");
        //        pass.setFilterLimits (odom.getOrigin().getY()-5.0, odom.getOrigin().getY()+5.0);
        //        pass.filter (*cloud_filtered);

        // Build a passthrough filter to remove spurious NaNs
        pass.setInputCloud (cloud);
        pass.setFilterFieldName("z");
        pass.setFilterLimits (-0.1, 11);
        pass.filter (*cloud_filtered);

        pass.setInputCloud (cloud_filtered);
        pass.setFilterFieldName("x");
        pass.setFilterLimits (-4.0, 4.0);
        pass.filter (*cloud_filtered);

        pass.setInputCloud (cloud_filtered);
        pass.setFilterFieldName("y");
        pass.setFilterLimits (-0.7, +1.5);
        pass.filter (*cloud_filtered);

        ROS_INFO("pcl_process: PointCloud after filtering has: %d data points.",cloud_filtered->width);

//        // Downsampling
//        pcl::PCLPointCloud2::Ptr pcl_cloud (new pcl::PCLPointCloud2 ());
//        pcl::toROSMsg(*cloud_filtered,temp_pcl);
//        pcl_conversions::toPCL(temp_pcl, *pcl_cloud);
//        pcl::VoxelGrid<pcl::PCLPointCloud2> downsample;
//        downsample.setInputCloud (pcl_cloud);
//        downsample.setLeafSize (0.01, 0.01, 0.01);
//        downsample.filter (*pcl_cloud);
//        pcl_conversions::fromPCL(*pcl_cloud,temp_pcl);
//        pcl::fromROSMsg(temp_pcl,*cloud_filtered);
//        ROS_INFO("PointCloud after downsampling has: %d data points.",cloud_filtered->width);

        // Remove outliers from the main body
//        pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor1;
//        sor1.setInputCloud (cloud_filtered);
//        sor1.setMeanK (50);
//        sor1.setStddevMulThresh (1.0);
//        sor1.filter (*cloud_filtered);
//        // ROS_INFO("PointCloud after statistical outlier removal has: %d data points.",cloud_filtered->points.size ());


        /// Perform Plane Segmentation

        // Estimate point normals
        ne.setSearchMethod (tree);
        ne.setInputCloud (cloud_filtered);
        ne.setKSearch (5);  // the bigger the neighborhood, the smoother it gets, original 50
        ne.compute (*cloud_normals);

        ROS_INFO("Estimated normals");

        // Create the segmentation object for the planar model and set all the parameters
        seg.setOptimizeCoefficients (true);
        seg.setModelType (pcl::SACMODEL_NORMAL_PLANE);
        seg.setNormalDistanceWeight (0.1);
        seg.setMethodType (pcl::SAC_RANSAC);
        seg.setMaxIterations (50);
        seg.setDistanceThreshold (0.08);  // the largest this is the more inclusive this is for points near the plane
        seg.setInputCloud (cloud_filtered);
        seg.setInputNormals (cloud_normals);
        // Obtain the plane inliers and coefficients
        seg.segment (*inliers_plane, *coefficients_plane);
        //std::cerr << "Plane coefficients: " << *coefficients_plane << std::endl;

        // Extract the planar inliers from the input cloud
        extract.setInputCloud (cloud_filtered);
        extract.setIndices (inliers_plane);
        extract.setNegative (false);

        // Write the planar inliers to disk
        extract.filter (*cloud_plane);

        //std::cerr << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;
        // writer.write ("table_scene_mug_stereo_textured_plane.pcd", *cloud_plane, false)

        // Remove the planar inliers, extract the rest
        extract.setNegative (true);
        extract.filter (*cloud_filtered2); //cloud_filtered2 contains the remaining areas after the plane extraction
        extract_normals.setNegative (true);
        extract_normals.setInputCloud (cloud_normals);
        extract_normals.setIndices (inliers_plane);
        extract_normals.filter (*cloud_normals2);
        extract_normals.setNegative(false);
        extract_normals.filter(*cloud_plane_normals);

        ROS_INFO("Found plane");

        /// Field detection on plane point cloud

        temp_cloud_field->header = cloud_filtered2->header;
        temp_cloud_field->width = 0;
        temp_cloud_field->height = 1;
        temp_cloud_field->is_dense = true;
        temp_cloud_field->sensor_orientation_ = cloud_filtered2->sensor_orientation_;
        temp_cloud_field->sensor_origin_ = cloud_filtered2->sensor_origin_;

        for (int pnt_count=0;pnt_count<cloud_plane->size();pnt_count++)
        {
            pcl::PointXYZRGB searchPoint = cloud_plane->points.at(pnt_count);


            // Make use of the isField function to detect if a ground point belongs to the green field
            if (isField(searchPoint))
            {
                // for field green points
                temp_cloud_field->points.push_back(searchPoint);
                temp_cloud_field->width++;

            }
        }

        ROS_INFO("Found field green");

        /// Keep only the connected field area to create the cloud_field

        // Remove outliers from the main body of the detected field

        pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
        sor.setInputCloud (temp_cloud_field);
        sor.setMeanK (50);
        sor.setStddevMulThresh (1.0);
        sor.filter (*temp_cloud_field);
        // ROS_INFO("PointCloud after statistical outlier removal has: %d data points.",cloud_filtered->points.size ());

        ROS_INFO("Removed field outliers");

        // Convert the field area from 3D to the corresponding area on the 2D image
        stereo_process::ObjectOnImage field_image;
        field_image.header.stamp = ros::Time::now();
        field_image.object_type = 0;
        field_image = convert3dTo2d(*temp_cloud_field,field_image);

        pub_field.publish(field_image);

        ROS_INFO("Published field");


        double z_min = 0., z_max = 2.00; // we want the points above the plane, no farther than 2m from the surface

        
        pcl::ConvexHull<pcl::PointXYZRGB> hull;
        // hull.setDimension (2); // not necessarily needed, but we need to check the dimensionality of the output
        hull.setInputCloud (temp_cloud_field);
        hull.reconstruct (*cloud_field);


        /// Only keep the part of cloud_filtered2 above the green field, to make cloud_above_field

        // Prism segmentation

        if (hull.getDimension () == 2)
        {
            pcl::ExtractPolygonalPrismData<pcl::PointXYZRGB> prism;
            prism.setInputCloud (cloud_filtered2);
            prism.setInputPlanarHull (cloud_field);
            prism.setHeightLimits (z_min, z_max);
            prism.segment (*cloud_above_field_indices);

            pcl::ExtractIndices<pcl::PointXYZRGB> extract;
            extract.setInputCloud(cloud_filtered2);
            extract.setIndices(cloud_above_field_indices);
            extract.setNegative(false);
            extract.filter(*cloud_above_field);
        }
        else
            PCL_ERROR ("The input cloud does not represent a planar surface.\n");

        ROS_INFO("Filtered PC has:%d points, Field PC has:%d points, PC above it has:%d points",cloud_filtered2->width,temp_cloud_field->width,cloud_above_field->width);



        /// Perform Sphere Segmentation

        // Estimate point normals
        ne.setSearchMethod (tree);
        ne.setInputCloud (cloud_above_field);
        ne.setKSearch (50);  // the bigger the neighborhood, the smoother it gets, original 50
        ne.compute (*cloud_normals2);

        // Create the segmentation object for the planar model and set all the parameters
        seg.setOptimizeCoefficients (true);
        seg.setModelType (pcl::SACMODEL_NORMAL_SPHERE);
        seg.setNormalDistanceWeight (0.05);
        seg.setMethodType (pcl::SAC_RANSAC);
        seg.setMaxIterations (100);
        seg.setRadiusLimits(0.05,0.16);  // The radius min-max limits for the detected spherical surface, official ball radius=11cm
        seg.setDistanceThreshold (0.10);  // the largest this is the more inclusive this is for points near the plane
        seg.setInputCloud (cloud_above_field);
        seg.setInputNormals (cloud_normals2);
        // Obtain the sphere inliers and coefficients
        seg.segment (*inliers_sphere, *coefficients_sphere);

        // Extract the sphere inliers from the input cloud
        extract.setInputCloud (cloud_above_field);
        extract.setIndices (inliers_sphere);
        extract.setNegative (false);

        // Write the sphere inliers to disk
        extract.filter (*cloud_cluster_ball);

        // Remove the planar inliers, extract the rest
        extract.setNegative (true);
        extract.filter (*cloud_above_field); //cloud_above_field contains the remaining areas after the plane and sphere extraction
        //        extract_normals.setNegative (true);
        //        extract_normals.setInputCloud (cloud_normals2);
        //        extract_normals.setIndices (inliers_sphere);
        //        extract_normals.filter (*cloud_normals2);
        //        extract_normals.setNegative(false);
        //        extract_normals.filter(*cloud_plane_normals);

        // Use this only if we need the ball information from here

        double cluster_ball_xmax=0.0, cluster_ball_ymax=0.0, cluster_ball_xmin=0.0, cluster_ball_ymin=0.0;
        double cluster_ball_zmax=0.0, cluster_ball_zmin=0.0;
        double cluster_ball_xmean = 0.0, cluster_ball_ymean = 0.0, cluster_ball_zmean = 0.0;
        double cluster_ball_range = 0.0, cluster_ball_bearing = 0.0, cluster_ball_height = 0.0;


        // Initialize ball message
        stereo_process::Ball ball_msg;
        ball_msg.ball_detected = false;
        ball_msg.ball_range = 0.0;
        ball_msg.ball_bearing = 0.0;
        if (cloud_cluster_ball->width>0)
        {
        // Initialize the bounding box parameters
        cluster_ball_xmax=cloud_cluster_ball->points[0].x;
        cluster_ball_ymax=cloud_cluster_ball->points[0].y;
        cluster_ball_zmax=cloud_cluster_ball->points[0].z;
        cluster_ball_xmin=cloud_cluster_ball->points[0].x;
        cluster_ball_ymin=cloud_cluster_ball->points[0].y;
        cluster_ball_zmin=cloud_cluster_ball->points[0].z;


        for (int pnt_count=0;pnt_count<cloud_cluster_ball->size();pnt_count++)
        {

            if (cloud_cluster_ball->points[pnt_count].x>cluster_ball_xmax)
                cluster_ball_xmax = cloud_cluster_ball->points[pnt_count].x;
            if (cloud_cluster_ball->points[pnt_count].y>cluster_ball_ymax)
                cluster_ball_ymax = cloud_cluster_ball->points[pnt_count].y;
            if (cloud_cluster_ball->points[pnt_count].z>cluster_ball_zmax)
                cluster_ball_zmax = cloud_cluster_ball->points[pnt_count].z;
            if (cloud_cluster_ball->points[pnt_count].x<cluster_ball_xmin)
                cluster_ball_xmin = cloud_cluster_ball->points[pnt_count].x;
            if (cloud_cluster_ball->points[pnt_count].y<cluster_ball_ymin)
                cluster_ball_ymin = cloud_cluster_ball->points[pnt_count].y;
            if (cloud_cluster_ball->points[pnt_count].z<cluster_ball_zmin)
                cluster_ball_zmin = cloud_cluster_ball->points[pnt_count].z;
        }


        cluster_ball_xmean = cluster_ball_xmin + (cluster_ball_xmax - cluster_ball_xmin)/2;
        cluster_ball_ymean = cluster_ball_ymin + (cluster_ball_ymax - cluster_ball_ymin)/2;
        cluster_ball_zmean = cluster_ball_zmin + (cluster_ball_zmax - cluster_ball_zmin)/2;


        //Transform this according to the rotation angle

        cluster_ball_zmin = cluster_ball_zmin * cos(pitch_dif);
        cluster_ball_height = (cluster_ball_ymax - cluster_ball_ymin) * cos(pitch_dif);


        cluster_ball_range = sqrt(pow(cluster_ball_xmean,2)+pow(cluster_ball_zmin,2));
        if (cluster_ball_xmean!=0)
            cluster_ball_bearing = atan2(cluster_ball_xmean,cluster_ball_zmin)* 180 / M_PI; //range [0,360]
        else
            cluster_ball_bearing = 0.0;

        ROS_INFO("Camera pitch difference:%f, cluster ball height:%f, range:%f, bearing:%f",pitch_dif,cluster_ball_height,cluster_ball_range,cluster_ball_bearing);

//        if ((cluster_ball_height<=0.30) && (cluster_ball_height>0.10)) // ball of height ~0.25m
//        {
            pcl::toROSMsg(*cloud_cluster_ball,output_ball);
            pub_cluster_ball.publish(output_ball);
//            ROS_INFO("Ball found at range:%f and bearing:%f",cluster_ball_range,cluster_ball_bearing);
//            ROS_INFO("Ball size:%d",cloud_cluster_ball->width);

//            ball_msg.ball_detected = true;
//            ball_msg.ball_range = cluster_ball_range;
//            ball_msg.ball_bearing = cluster_ball_bearing;


//        }

        // Convert the field area from 3D to the corresponding area on the 2D image
        stereo_process::ObjectOnImage ball_image;
        ball_image.header.stamp = ros::Time::now();
        ball_image.object_type = 1;
        ball_image = convert3dTo2d(*cloud_cluster_ball,ball_image);

        pub_ball.publish(ball_image);

}


        /// Here, is the code for segmentation of the other obstacles around the robot

//        double cluster_xmax=0.0, cluster_ymax=0.0, cluster_xmin=0.0, cluster_ymin=0.0;
//        double cluster_zmax=0.0, cluster_zmin=0.0;
//        double cluster_xmean = 0.0, cluster_ymean = 0.0, cluster_zmean = 0.0;
//        double cluster_range = 0.0, cluster_bearing = 0.0, cluster_width = 0.0, cluster_height = 0.0;


//        // Creating the KdTree object for the search method of the extraction
//        tree->setInputCloud (cloud_above_field);

//        std::vector<pcl::PointIndices> cluster_indices;
//        pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
//        ec.setClusterTolerance (0.05); // 5cm
//        ec.setMinClusterSize (300);
//        ec.setMaxClusterSize (10000);
//        ec.setSearchMethod (tree);
//        ec.setInputCloud (cloud_above_field);
//        ec.extract (cluster_indices);

//        extract.filter(*cloud_cluster);

//        int j = 0;

//        //std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clusters;
//        for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
//        {
//            cloud_cluster->points.clear();

////            // Initialize the bounding box parameters
////            cluster_xmax=cloud_above_field->points[*(it->indices.begin ())].x;
////            cluster_ymax=cloud_above_field->points[*(it->indices.begin ())].y;
////            cluster_zmax=cloud_above_field->points[*(it->indices.begin ())].z;
////            cluster_xmin=cloud_above_field->points[*(it->indices.begin ())].x;
////            cluster_ymin=cloud_above_field->points[*(it->indices.begin ())].y;
////            cluster_zmin=cloud_above_field->points[*(it->indices.begin ())].z;

//            for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
//            {
//                cloud_cluster->points.push_back (cloud_above_field->points[*pit]);

//                // Calculate the cluster's bounding box

////                if (cloud_above_field->points[*pit].x>cluster_xmax) cluster_xmax = cloud_above_field->points[*pit].x;
////                if (cloud_above_field->points[*pit].y>cluster_ymax) cluster_ymax = cloud_above_field->points[*pit].y;
////                if (cloud_above_field->points[*pit].z>cluster_zmax) cluster_zmax = cloud_above_field->points[*pit].z;
////                if (cloud_above_field->points[*pit].x<cluster_xmin) cluster_xmin = cloud_above_field->points[*pit].x;
////                if (cloud_above_field->points[*pit].y<cluster_ymin) cluster_ymin = cloud_above_field->points[*pit].y;
////                if (cloud_above_field->points[*pit].z<cluster_zmin) cluster_zmin = cloud_above_field->points[*pit].z;
//            }


//            cloud_cluster->width = cloud_cluster->points.size ();
//            cloud_cluster->height = 1;
//            cloud_cluster->is_dense = true;
//            cloud_cluster->sensor_orientation_ = cloud_above_field->sensor_orientation_;
//            cloud_cluster->sensor_origin_ = cloud_above_field->sensor_origin_;

////            cluster_xmean = cluster_xmin + (cluster_xmax - cluster_xmin)/2;
////            //cluster_xmean = cluster_xmin ;
////            cluster_ymean = cluster_ymin + (cluster_ymax - cluster_ymin)/2;
////            //            cluster_zmean = cluster_zmin + (cluster_zmax - cluster_zmin)/2;


////            //Transform this according to the rotation angle

////            cluster_zmin = cluster_zmin * cos(pitch_dif);
////            cluster_height = (cluster_ymax - cluster_ymin) * cos(pitch_dif);


////            cluster_range = sqrt(pow(cluster_xmean,2)+pow(cluster_zmin,2));
////            if (cluster_xmean!=0)
////                cluster_bearing = atan2(cluster_xmean,cluster_zmin)* 180 / M_PI; //range [0,360]
////            else
////                cluster_bearing = 0.0;

//            //ROS_INFO("Camera pitch difference:%f, cluster height:%f, range:%f",pitch_dif,cluster_height,cluster_range);
//            //ROS_INFO("PointCloud representing the Cluster: %d data points, x_min:%f, x_max:%f, y_min:%f, y_max:%f, z_min:%f, z_max:%f",cloud_cluster->points.size(),cluster_xmin,cluster_xmax,cluster_ymin,cluster_ymax,cluster_zmin,cluster_zmax);

//            // No classification

//            if (j==0)
//            {
//                pcl::toROSMsg(*cloud_cluster,output_obstacle1);
//                pub_cluster_obstacle_1.publish(output_obstacle1);
//                // ROS_INFO("Obstacle1 at range:%f, bearing:%f, size:%d",cluster_range,cluster_bearing,cloud_cluster->width);
//                ROS_INFO("Obstacle1 size:%d",cloud_cluster->width);

//            }
//            if (j==1)
//            {
//                pcl::toROSMsg(*cloud_cluster,output_obstacle2);
//                pub_cluster_obstacle_2.publish(output_obstacle2);
//                // ROS_INFO("Obstacle2 at range:%f, bearing:%f, size:%d",cluster_range,cluster_bearing,cloud_cluster->width);
//                ROS_INFO("Obstacle2 size:%d",cloud_cluster->width);
//            }
//            if (j==2)
//            {
//                pcl::toROSMsg(*cloud_cluster,output_obstacle3);
//                pub_cluster_obstacle_3.publish(output_obstacle3);
//                // ROS_INFO("Obstacle3 at range:%f, bearing:%f, size:%d",cluster_range,cluster_bearing,cloud_cluster->width);
//            }
//            if (j==3)
//            {
//                pcl::toROSMsg(*cloud_cluster,output_obstacle4);
//                pub_cluster_obstacle_4.publish(output_obstacle4);
//                // ROS_INFO("Obstacle4 at range:%f, bearing:%f, size:%d",cluster_range,cluster_bearing,cloud_cluster->width);
//            }


//            // Convert the field area from 3D to the corresponding area on the 2D image
//            stereo_process::ObjectOnImage object_image;
//            object_image.header.stamp = ros::Time::now();
//            object_image.object_type = 3;
//            object_image = convert3dTo2d(*cloud_cluster,object_image);

//            pub_obstacle.publish(ball_image);

//            j++;

//        }



        // Convert the pcl/PointCloud data to sensor_msgs/PointCloud2
        pcl::toROSMsg(*cloud_filtered,output);
        pcl::toROSMsg(*cloud_above_field,output_above_field);
        pcl::toROSMsg(*cloud_plane,output_plane);
        pcl::toROSMsg(*temp_cloud_field,output_field);



        /// Publish the data.
        pub.publish (output);
        pub_above_field.publish (output_above_field);
        pub_plane.publish (output_plane);
        pub_cluster_field.publish (output_field);


        /// Publish all the info for the ball, obstacles and goalpost

        //pub_ball.publish(ball_msg);

        // ROS_INFO("Finished publishing");

    }

    /// Part for the service that returns the depth of objects within the original pointcloud on demand

};

int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "bumblebee_pcl_process");
    ros::NodeHandle nh;

    PointCloudProcess bumblebee_pcl_process;

    while(ros::ok())
    {
        ros::spinOnce();
        //rate.sleep(); // check if we can add the delay here to control the frequency or if this stalls even responses to service requests
    }

    return 0;
}

