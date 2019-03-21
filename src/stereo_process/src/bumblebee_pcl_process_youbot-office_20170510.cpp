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
//// Last update  : 2017.05.04                                                             ////
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

//#include "bumblebee_pcl_process.h"
#include "stereo_process/ObjectOnImage.h"
#include "stereo_process/Ball.h"
#include "stereo_process/Obstacles.h"
#include "stereo_process/Goalpost.h"
#include "stereo_process/DepthRequest.h"


#define MIN_CLUSTER_SIZE 15
#define CAMERA_HEIGHT 1.15

class PointCloudProcess
{
    ros::NodeHandle pcl_nh;

    image_transport::ImageTransport it_;

    ros::Publisher pub, pub_above_field,pub_plane,pub_cluster_field,pub_field,pub_cluster_ball,pub_ball,pub_cluster_obstacle_1,pub_cluster_obstacle_2,
    pub_cluster_obstacle_3,pub_cluster_obstacle_4,pub_cluster_obstacle_5,pub_obstacle,pub_cluster_goalpost,pub_goalpost;

    ros::Publisher pub_vision_ball;
    // Create a ROS subscriber for the input point cloud and imu orientation
    ros::Subscriber sub_cloud, sub_imu, sub_image_info;


    ros::ServiceServer depth_request_server;

public:

    // transform declarations
    tf::TransformBroadcaster odom_broadcaster, lidar_broadcaster;
    tf::TransformListener listener;
    tf::StampedTransform odom;
    tf::StampedTransform lidar, init_lidar;


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

        sub_image_info = pcl_nh.subscribe("/camera/right/camera_info", 1, &PointCloudProcess::imageCb, this); // check this topic's name

        // Create ROS publishers for the output point cloud
        pub = pcl_nh.advertise<sensor_msgs::PointCloud2> ("output", 1);
        pub_above_field = pcl_nh.advertise<sensor_msgs::PointCloud2> ("above_field", 1);
        pub_plane = pcl_nh.advertise<sensor_msgs::PointCloud2> ("plane", 1);
        pub_cluster_field = pcl_nh.advertise<sensor_msgs::PointCloud2> ("field", 1);
        pub_field = pcl_nh.advertise<stereo_process::ObjectOnImage> ("/stereo_process/field", 1);

        pub_cluster_ball = pcl_nh.advertise<sensor_msgs::PointCloud2> ("cluster_ball", 1);
        pub_ball = pcl_nh.advertise<stereo_process::ObjectOnImage> ("/stereo_process/ball", 1);
        pub_vision_ball = pcl_nh.advertise<stereo_process::Ball> ("/vision/ball", 1);

        pub_cluster_obstacle_1 = pcl_nh.advertise<sensor_msgs::PointCloud2> ("cluster_obstacle_1", 1);
        pub_cluster_obstacle_2 = pcl_nh.advertise<sensor_msgs::PointCloud2> ("cluster_obstacle_2", 1);
        pub_cluster_obstacle_3 = pcl_nh.advertise<sensor_msgs::PointCloud2> ("cluster_obstacle_3", 1);
        pub_cluster_obstacle_4 = pcl_nh.advertise<sensor_msgs::PointCloud2> ("cluster_obstacle_4", 1);
        pub_cluster_obstacle_5 = pcl_nh.advertise<sensor_msgs::PointCloud2> ("cluster_obstacle_5", 1);
        pub_obstacle = pcl_nh.advertise<stereo_process::ObjectOnImage> ("/stereo_process/obstacle", 1);

        pub_cluster_goalpost = pcl_nh.advertise<sensor_msgs::PointCloud2> ("cluster_goalpost", 1);
        pub_goalpost = pcl_nh.advertise<stereo_process::ObjectOnImage> ("/stereo_process/goalpost", 1);

        depth_request_server = pcl_nh.advertiseService("depth_request", &PointCloudProcess::depthRequestSrv, this);


        odom.stamp_ = ros::Time::now();


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

            output2dSet.image_u.push_back(int(uv.x));
            output2dSet.image_v.push_back(int(uv.y));
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

    void imageCb(const sensor_msgs::CameraInfoConstPtr& info_msg)
    {
        // Only use the new info if it is close to the newly acquired pointcloud
        if ((pc_timestamp - info_msg->header.stamp.toSec())<0.01)
        {
            cam_model_.fromCameraInfo(info_msg);
            //ROS_INFO("Assigned new camera model");
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
        if ((point_hsv.h>=94 && point_hsv.h<=170)
                && (point_hsv.s>=0.00 && point_hsv.s<=1.00)
                && (point_hsv.v>=0.00 && point_hsv.v<=0.97)
                )
            return true;
        else
        {   //if (point_hsv.h>250)
            //ROS_INFO("Ground point-no field, hsv:(%lf,%lf,%lf)",point_hsv.h,point_hsv.s,point_hsv.v);
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
                output_obstacle5,output_goalpost;

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
        pitch_dif += 0.436; // 25 degrees offset for now
        double yaw_dif = -(imu_pitch-init_imu_pitch);
        ROS_INFO("Camera rotation from initial pose (r,p,y)=(%f,%f,%f)",imu_yaw*180/M_PI, pitch_dif*180/M_PI, yaw_dif*180/M_PI);
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

        // Build a passthrough filter to check only a specific area of interest in our pointcloud
        // z and y should depend on the pitch angle of the camera 'theta'
        pass.setInputCloud (cloud);
        pass.setFilterFieldName("z");
        pass.setFilterLimits (-0.1, 11/cos(pitch_dif)); // to avoid missing points further
        pass.filter (*cloud_filtered);

        pass.setInputCloud (cloud_filtered);
        pass.setFilterFieldName("x");
        pass.setFilterLimits (-4.0, 4.0);
        pass.filter (*cloud_filtered);

        pass.setInputCloud (cloud_filtered);
        pass.setFilterFieldName("y");
        pass.setFilterLimits (-11*sin(pitch_dif), +CAMERA_HEIGHT); // to avoid missing points higher
        pass.filter (*cloud_filtered);

        ROS_INFO("pcl_process: PointCloud after filtering with zmax:%f, ymin:%f has: %d data points.",11/cos(pitch_dif),-11*sin(pitch_dif),cloud_filtered->width);

        // Publish pcl after filtering
        pcl::toROSMsg(*cloud_filtered,output);
        pub.publish (output);

        //                // Downsampling
        //                pcl::PCLPointCloud2::Ptr pcl_cloud (new pcl::PCLPointCloud2 ());
        //                pcl::toROSMsg(*cloud_filtered,temp_pcl);
        //                pcl_conversions::toPCL(temp_pcl, *pcl_cloud);
        //                pcl::VoxelGrid<pcl::PCLPointCloud2> downsample;
        //                downsample.setInputCloud (pcl_cloud);
        //                downsample.setLeafSize (0.01, 0.01, 0.01);
        //                downsample.filter (*pcl_cloud);
        //                pcl_conversions::fromPCL(*pcl_cloud,temp_pcl);
        //                pcl::fromROSMsg(temp_pcl,*cloud_filtered);
        //                ROS_INFO("PointCloud after downsampling has: %d data points.",cloud_filtered->width);


        /// Perform Plane Segmentation

        // Estimate point normals
        ne.setSearchMethod (tree);
        ne.setInputCloud (cloud_filtered);
        ne.setKSearch (5);  // the bigger the neighborhood, the smoother it gets, original 50
        // lower it to compute faster
        ne.compute (*cloud_normals);

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


        // Publish plane pcl
        pcl::toROSMsg(*cloud_plane,output_plane);
        pub_plane.publish (output_plane);

        ROS_INFO("After plane segmentation");

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

        ROS_INFO("After field detection");


        /// Keep only the connected field area to create the cloud_field

        //        // Remove outliers from the main body of the detected field

        //        pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
        //        sor.setInputCloud (temp_cloud_field);
        //        sor.setMeanK (50);
        //        sor.setStddevMulThresh (1.0);
        //        sor.filter (*temp_cloud_field);
        //        // ROS_INFO("PointCloud after statistical outlier removal has: %d data points.",cloud_filtered->points.size ());

        ROS_INFO("After field outliers removal");

        // Convert the field area from 3D to the corresponding area on the 2D image
        stereo_process::ObjectOnImage field_image;
        //field_image.header.stamp = ros::Time::now();
        field_image.header.stamp = input->header.stamp;
        field_image.object_type = 0;
        field_image = convert3dTo2d(*temp_cloud_field,field_image);


        // Publish field image and field pcl
        pub_field.publish(field_image);



        // Publish field point cloud before convex hull
        pcl::toROSMsg(*temp_cloud_field,output_field);
        pub_cluster_field.publish (output_field);

        ROS_INFO("After field publish");

        double z_min = 0.08, z_max = CAMERA_HEIGHT; // we want the points above the plane, but no ground. no further than 1.3m from the surface

        
        pcl::ConvexHull<pcl::PointXYZRGB> hull;
        // hull.setDimension (2); // not necessarily needed, but we need to check the dimensionality of the output
        hull.setInputCloud (temp_cloud_field);
        hull.reconstruct (*cloud_field);

//        // Publish field point cloud after convex hull
//        pcl::toROSMsg(*cloud_field,output_field);
//        pub_cluster_field.publish (output_field);

//        ROS_INFO("After field publish");


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

        ROS_INFO("Filtered PC has:%d points, Field PC has:%d points, PC above it has:%d points",cloud_filtered2->width,cloud_field->width,cloud_above_field->width);



        //        /// Perform Sphere Segmentation

        //        // Estimate point normals
        //        ne.setSearchMethod (tree);
        //        ne.setInputCloud (cloud_above_field);
        //        ne.setKSearch (50);  // the bigger the neighborhood, the smoother it gets, original 50
        //        ne.compute (*cloud_normals2);

        //        // Create the segmentation object for the planar model and set all the parameters
        //        seg.setOptimizeCoefficients (true);
        //        seg.setModelType (pcl::SACMODEL_NORMAL_SPHERE);
        //        seg.setNormalDistanceWeight (0.15);
        //        seg.setMethodType (pcl::SAC_RANSAC);
        //        seg.setMaxIterations (100);
        //        seg.setRadiusLimits(0.01,0.18);  // The radius min-max limits for the detected spherical surface, official ball radius=11cm
        //        seg.setDistanceThreshold (0.10);  // the largest this is the more inclusive this is for points near the plane
        //        seg.setInputCloud (cloud_above_field);
        //        seg.setInputNormals (cloud_normals2);
        //        // Obtain the sphere inliers and coefficients
        //        seg.segment (*inliers_sphere, *coefficients_sphere);

        //        // Extract the sphere inliers from the input cloud
        //        extract.setInputCloud (cloud_above_field);
        //        extract.setIndices (inliers_sphere);
        //        extract.setNegative (false);

        //        // Write the sphere inliers to disk
        //        extract.filter (*cloud_cluster_ball);

        //        // Remove the planar inliers, extract the rest
        //        extract.setNegative (true);
        //        extract.filter (*cloud_above_field); //cloud_above_field contains the remaining areas after the plane and sphere extraction
        //        //        extract_normals.setNegative (true);
        //        //        extract_normals.setInputCloud (cloud_normals2);
        //        //        extract_normals.setIndices (inliers_sphere);
        //        //        extract_normals.filter (*cloud_normals2);
        //        //        extract_normals.setNegative(false);
        //        //        extract_normals.filter(*cloud_plane_normals);


        //        // Use this only if we need the ball information from here

        //        double cluster_ball_xmax=0.0, cluster_ball_ymax=0.0, cluster_ball_xmin=0.0, cluster_ball_ymin=0.0;
        //        double cluster_ball_zmax=0.0, cluster_ball_zmin=0.0;
        //        double cluster_ball_xmean = 0.0, cluster_ball_ymean = 0.0, cluster_ball_zmean = 0.0;
        //        double cluster_ball_range = 0.0, cluster_ball_bearing = 0.0, cluster_ball_height = 0.0;


        //        if (cloud_cluster_ball->width>0)
        //        {

        //            // Initialize the bounding box parameters
        //            cluster_ball_xmax=cloud_cluster_ball->points[0].x;
        //            cluster_ball_ymax=cloud_cluster_ball->points[0].y;
        //            cluster_ball_zmax=cloud_cluster_ball->points[0].z;
        //            cluster_ball_xmin=cloud_cluster_ball->points[0].x;
        //            cluster_ball_ymin=cloud_cluster_ball->points[0].y;
        //            cluster_ball_zmin=cloud_cluster_ball->points[0].z;


        //            for (int pnt_count=0;pnt_count<cloud_cluster_ball->size();pnt_count++)
        //            {

        //                if (cloud_cluster_ball->points[pnt_count].x>cluster_ball_xmax)
        //                    cluster_ball_xmax = cloud_cluster_ball->points[pnt_count].x;
        //                if (cloud_cluster_ball->points[pnt_count].y>cluster_ball_ymax)
        //                    cluster_ball_ymax = cloud_cluster_ball->points[pnt_count].y;
        //                if (cloud_cluster_ball->points[pnt_count].z>cluster_ball_zmax)
        //                    cluster_ball_zmax = cloud_cluster_ball->points[pnt_count].z;
        //                if (cloud_cluster_ball->points[pnt_count].x<cluster_ball_xmin)
        //                    cluster_ball_xmin = cloud_cluster_ball->points[pnt_count].x;
        //                if (cloud_cluster_ball->points[pnt_count].y<cluster_ball_ymin)
        //                    cluster_ball_ymin = cloud_cluster_ball->points[pnt_count].y;
        //                if (cloud_cluster_ball->points[pnt_count].z<cluster_ball_zmin)
        //                    cluster_ball_zmin = cloud_cluster_ball->points[pnt_count].z;
        //            }


        //            cluster_ball_xmean = cluster_ball_xmin + (cluster_ball_xmax - cluster_ball_xmin)/2;
        //            cluster_ball_ymean = cluster_ball_ymin + (cluster_ball_ymax - cluster_ball_ymin)/2;
        //            cluster_ball_zmean = cluster_ball_zmin + (cluster_ball_zmax - cluster_ball_zmin)/2;


        //            //Transform this according to the rotation angle

        //            cluster_ball_zmin = cluster_ball_zmin * cos(pitch_dif);
        //            cluster_ball_height = (cluster_ball_ymax - cluster_ball_ymin) * cos(pitch_dif);


        //            cluster_ball_range = sqrt(pow(cluster_ball_xmean,2)+pow(cluster_ball_zmin,2));
        //            if (cluster_ball_xmean!=0)
        //                cluster_ball_bearing = atan2(cluster_ball_xmean,cluster_ball_zmin)* 180 / M_PI; //range [0,360]
        //            else
        //                cluster_ball_bearing = 0.0;

        //            ROS_INFO("Camera pitch difference:%f, cluster ball height:%f, range:%f, bearing:%f, size:%d",
        //                     pitch_dif,cluster_ball_height,cluster_ball_range,cluster_ball_bearing,cloud_cluster_ball->width);

        //            //        if ((cluster_ball_height<=0.30) && (cluster_ball_height>0.10)) // ball of height ~0.25m
        //            //        {
        //            pcl::toROSMsg(*cloud_cluster_ball,output_ball);
        //            pub_cluster_ball.publish(output_ball);
        //            //            ROS_INFO("Ball found at range:%f and bearing:%f",cluster_ball_range,cluster_ball_bearing);
        //            //            ROS_INFO("Ball size:%d",cloud_cluster_ball->width);

        //            //        }

        //            ROS_INFO("After sphere segmentation");

        //            // Convert the field area from 3D to the corresponding area on the 2D image
        //            stereo_process::ObjectOnImage ball_image;
        //            ball_image.header.stamp = ros::Time::now();
        //            ball_image.object_type = 1;
        //            ball_image = convert3dTo2d(*cloud_cluster_ball,ball_image);

        //            pub_ball.publish(ball_image);

        //        }

        //        ROS_INFO("After ball publish");

        // Publish pcl above ground
        pcl::toROSMsg(*cloud_above_field,output_above_field);
        pub_above_field.publish (output_above_field);


        /// Here, is the code for segmentation of the other obstacles around the robot

        double cluster_xmax=0.0, cluster_ymax=0.0, cluster_xmin=0.0, cluster_ymin=0.0;
        double cluster_zmax=0.0, cluster_zmin=0.0;
        double cluster_xmean = 0.0, cluster_ymean = 0.0, cluster_zmean = 0.0;
        double cluster_range = 0.0, cluster_bearing = 0.0, cluster_width = 0.0, cluster_height = 0.0 , cluster_height_max=0.0;


        // Creating the KdTree object for the search method of the extraction
        tree->setInputCloud (cloud_above_field);

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
        ec.setClusterTolerance (0.05); // 5cm
        ec.setMinClusterSize (300);
        ec.setMaxClusterSize (50000);
        ec.setSearchMethod (tree);
        ec.setInputCloud (cloud_above_field);
        ec.extract (cluster_indices);

        extract.filter(*cloud_cluster);

        ROS_INFO("After object segmentation");

        int j = 0;
        bool ball_detected = false;

        for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
        {
            cloud_cluster->points.clear();

            // Initialize the bounding box parameters
            cluster_xmax=cloud_above_field->points[*(it->indices.begin ())].x;
            cluster_ymax=cloud_above_field->points[*(it->indices.begin ())].y;
            cluster_zmax=cloud_above_field->points[*(it->indices.begin ())].z;
            cluster_xmin=cloud_above_field->points[*(it->indices.begin ())].x;
            cluster_ymin=cloud_above_field->points[*(it->indices.begin ())].y;
            cluster_zmin=cloud_above_field->points[*(it->indices.begin ())].z;

            for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
            {
                cloud_cluster->points.push_back (cloud_above_field->points[*pit]);

                // Calculate the cluster's bounding box

                if (cloud_above_field->points[*pit].x>cluster_xmax) cluster_xmax = cloud_above_field->points[*pit].x;
                if (cloud_above_field->points[*pit].y>cluster_ymax) cluster_ymax = cloud_above_field->points[*pit].y;
                if (cloud_above_field->points[*pit].z>cluster_zmax) cluster_zmax = cloud_above_field->points[*pit].z;
                if (cloud_above_field->points[*pit].x<cluster_xmin) cluster_xmin = cloud_above_field->points[*pit].x;
                if (cloud_above_field->points[*pit].y<cluster_ymin) cluster_ymin = cloud_above_field->points[*pit].y;
                if (cloud_above_field->points[*pit].z<cluster_zmin) cluster_zmin = cloud_above_field->points[*pit].z;
            }


            cloud_cluster->width = cloud_cluster->points.size ();
            cloud_cluster->height = 1;
            cloud_cluster->is_dense = true;
            cloud_cluster->sensor_orientation_ = cloud_above_field->sensor_orientation_;
            cloud_cluster->sensor_origin_ = cloud_above_field->sensor_origin_;

            cluster_xmean = cluster_xmin + (cluster_xmax - cluster_xmin)/2;
            //cluster_xmean = cluster_xmin ;
            cluster_width = cluster_xmax - cluster_xmin;
            cluster_ymean = cluster_ymin + (cluster_ymax - cluster_ymin)/2;
            //            cluster_zmean = cluster_zmin + (cluster_zmax - cluster_zmin)/2;


            //Transform this according to the rotation angle

            cluster_zmin = cluster_zmin * cos(pitch_dif);
            cluster_height = (cluster_ymax - cluster_ymin) * cos(pitch_dif);

            cluster_height_max = CAMERA_HEIGHT -cluster_ymin * cos(pitch_dif) - cluster_zmin * sin(pitch_dif);


            cluster_range = sqrt(pow(cluster_xmean,2)+pow(cluster_zmin,2));
            if (cluster_xmean!=0)
                cluster_bearing = atan2(cluster_xmean,cluster_zmin)* 180 / M_PI; //range [0,360]
            else
                cluster_bearing = 0.0;

            //ROS_INFO("Camera pitch difference:%f, cluster height:%f, range:%f",pitch_dif,cluster_height,cluster_range);
            //ROS_INFO("PointCloud representing the Cluster: %d data points, x_min:%f, x_max:%f, y_min:%f, y_max:%f, z_min:%f, z_max:%f",cloud_cluster->points.size(),cluster_xmin,cluster_xmax,cluster_ymin,cluster_ymax,cluster_zmin,cluster_zmax);

            // Classification for ball on the ground

            if ((cluster_height<=0.30) && (cluster_height_max<=0.50) && (cluster_width<=0.40)
                    && (!ball_detected)
                    )
            {
                ROS_INFO("Ball found, cluster ball height:%f, width:%f, max_height:%f, range:%f, bearing:%f, size:%d",
                         cluster_height,cluster_width,cluster_height_max,cluster_range,cluster_bearing,cloud_cluster->width);

                pcl::toROSMsg(*cloud_cluster,output_ball);
                pub_cluster_ball.publish(output_ball);


                // Convert the field area from 3D to the corresponding area on the 2D image
                stereo_process::ObjectOnImage ball_image;
                //ball_image.header.stamp = ros::Time::now();
                ball_image.header.stamp = input->header.stamp;
                ball_image.object_type = 1;
                ball_image = convert3dTo2d(*cloud_cluster,ball_image);

                pub_ball.publish(ball_image);

                // Publish ball info as vision
                stereo_process::Ball ball_msg;
                ball_msg.header.stamp = input->header.stamp;
                ball_msg.ball_detected = true;
                ball_msg.ball_range = cluster_range;
                ball_msg.ball_bearing = cluster_bearing;

                pub_vision_ball.publish(ball_msg);

                ball_detected = true;

            }
            else if ((cluster_height>0.10)
                    && (cluster_height_max<CAMERA_HEIGHT)
                    // && (cluster_width<=0.80)
                     )
            {

                if (j==0)
                {
                    pcl::toROSMsg(*cloud_cluster,output_obstacle1);
                    pub_cluster_obstacle_1.publish(output_obstacle1);
                    ROS_INFO("Obstacle1 at range:%f, bearing:%f, size:%d, height:%f, width:%f, height_max:%f",cluster_range,cluster_bearing,cloud_cluster->width, cluster_height, cluster_width, cluster_height_max);


                }
                if (j==1)
                {
                    pcl::toROSMsg(*cloud_cluster,output_obstacle2);
                    pub_cluster_obstacle_2.publish(output_obstacle2);
                    ROS_INFO("Obstacle2 at range:%f, bearing:%f, size:%d, height:%f, width:%f, height_max:%f",cluster_range,cluster_bearing,cloud_cluster->width, cluster_height, cluster_width, cluster_height_max);
                }
                if (j==2)
                {
                    pcl::toROSMsg(*cloud_cluster,output_obstacle3);
                    pub_cluster_obstacle_3.publish(output_obstacle3);
                    ROS_INFO("Obstacle3 at range:%f, bearing:%f, size:%d, height:%f, width:%f, height_max:%f",cluster_range,cluster_bearing,cloud_cluster->width, cluster_height, cluster_width, cluster_height_max);
                }
                if (j==3)
                {
                    pcl::toROSMsg(*cloud_cluster,output_obstacle4);
                    pub_cluster_obstacle_4.publish(output_obstacle4);
                    ROS_INFO("Obstacle4 at range:%f, bearing:%f, size:%d, height:%f, width:%f, height_max:%f",cluster_range,cluster_bearing,cloud_cluster->width, cluster_height, cluster_width, cluster_height_max);
                }
                if (j==4)
                {
                    pcl::toROSMsg(*cloud_cluster,output_obstacle5);
                    pub_cluster_obstacle_5.publish(output_obstacle5);
                    ROS_INFO("Obstacle5 at range:%f, bearing:%f, size:%d, height:%f, width:%f, height_max:%f",cluster_range,cluster_bearing,cloud_cluster->width, cluster_height, cluster_width, cluster_height_max);
                }


                // Convert the field area from 3D to the corresponding area on the 2D image
                stereo_process::ObjectOnImage object_image;
                //object_image.header.stamp = ros::Time::now();
                object_image.header.stamp = input->header.stamp;
                object_image.object_type = 3;
                object_image = convert3dTo2d(*cloud_cluster,object_image);

                pub_obstacle.publish(object_image);

                j++;
            }







        }


    }


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

