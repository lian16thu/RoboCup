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
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
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

#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <cv.hpp>
#include <cv_bridge/cv_bridge.h>




class PointCloudProcess
{
    ros::NodeHandle pcl_nh;

    ros::Publisher pub,pub_1,pub_plane_intensity_grad,pub_plane_elevation,pub_plane_density,pub_plane_pedig_rgb,
    pub_cluster_0,pub_cluster_1,pub_cluster_2,pub_cluster_3,pub_cluster_4,pub_cluster_5,pub_cluster_6,pub_cluster_7,pub_cluster_8;
    ros::Publisher road_image_pub;

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub;

public:

    // transform declarations
    tf::TransformBroadcaster odom_broadcaster, lidar_broadcaster;
    tf::TransformListener listener;
    tf::StampedTransform odom;
    tf::StampedTransform lidar, init_lidar;


    PointCloudProcess()
    {
        sub = pcl_nh.subscribe ("assembled_cloud", 1, &PointCloudProcess::cloud_cb,this);

        // Create ROS publishers for the output point cloud
        pub = pcl_nh.advertise<sensor_msgs::PointCloud2> ("output", 1);
        pub_1 = pcl_nh.advertise<sensor_msgs::PointCloud2> ("plane", 1);
        pub_plane_intensity_grad = pcl_nh.advertise<sensor_msgs::PointCloud2> ("plane_intensity_grad", 1);
        pub_plane_elevation = pcl_nh.advertise<sensor_msgs::PointCloud2> ("plane_elevation", 1);
        pub_plane_density = pcl_nh.advertise<sensor_msgs::PointCloud2> ("plane_density", 1);
        pub_plane_pedig_rgb = pcl_nh.advertise<sensor_msgs::PointCloud2> ("plane_pedig_rgb", 1);

        pub_cluster_0 = pcl_nh.advertise<sensor_msgs::PointCloud2> ("cluster_0", 1);
        pub_cluster_1 = pcl_nh.advertise<sensor_msgs::PointCloud2> ("cluster_1", 1);
        pub_cluster_2 = pcl_nh.advertise<sensor_msgs::PointCloud2> ("cluster_2", 1);
        pub_cluster_3 = pcl_nh.advertise<sensor_msgs::PointCloud2> ("cluster_3", 1);
        pub_cluster_4 = pcl_nh.advertise<sensor_msgs::PointCloud2> ("cluster_4", 1);
        pub_cluster_5 = pcl_nh.advertise<sensor_msgs::PointCloud2> ("cluster_5", 1);
        pub_cluster_6 = pcl_nh.advertise<sensor_msgs::PointCloud2> ("cluster_6", 1);
        pub_cluster_7 = pcl_nh.advertise<sensor_msgs::PointCloud2> ("cluster_7", 1);
        pub_cluster_8 = pcl_nh.advertise<sensor_msgs::PointCloud2> ("cluster_8", 1);

        road_image_pub = pcl_nh.advertise<sensor_msgs::Image> ("road_image", 1);

        odom.stamp_ = ros::Time::now();
    }

    ~PointCloudProcess()
    {

    }



    void cloud_to_intensity_image (const pcl::PointCloud<pcl::PointXYZI> &cloud, sensor_msgs::Image& img)
    {
        // Use these dimensions for now
        img.height = 600;
        img.width = 400;

        // sensor_msgs::image_encodings::mono16;
        img.encoding = "mono16";
        img.step = img.width * sizeof (uint16_t);
        img.data.resize (img.step * img.height,0);  //automatically set to 0
        //ROS_INFO("image step:%d,size:%d,first elem:%d",img.step,img.data.size(),img.data[5]);
        int points_per_cell[img.width][img.height]; // array for points per cell
        for (int i=0;i<img.width;i++)
            for (int j=0;j<img.height;j++)
            {
                //img.data[j+img.step*i] = (uint16_t)0;
                points_per_cell[i][j]=0;
            }

        // Find (x,y) limits of points on point cloud
        double min_x=cloud.at(0).x,max_x=cloud.at(0).x,min_y=cloud.at(0).y,max_y=cloud.at(0).y;
        int min_intensity=cloud.at(0).intensity,max_intensity=cloud.at(0).intensity;

        for (int pnt_count=0;pnt_count<cloud.size();pnt_count++)
        {
            if (cloud.at(pnt_count).x<min_x)
                min_x = cloud.at(pnt_count).x;
            if (cloud.at(pnt_count).x>max_x)
                max_x = cloud.at(pnt_count).x;
            if (cloud.at(pnt_count).y<min_y)
                min_y = cloud.at(pnt_count).y;
            if (cloud.at(pnt_count).y>max_y)
                max_y = cloud.at(pnt_count).y;
            if (cloud.at(pnt_count).intensity<min_intensity)
                min_intensity = cloud.at(pnt_count).intensity;
            if (cloud.at(pnt_count).intensity>max_intensity)
                max_intensity = cloud.at(pnt_count).intensity;
        }

        ROS_INFO("Cloud min_x:%f,max_x:%f,min_y:%f,max_y:%f,min_int:%d,max_int:%d",min_x,max_x,min_y,max_y,min_intensity,max_intensity);


        for (int pnt_count=0;pnt_count<cloud.size();pnt_count++)
        {
            int temp_x = trunc(img.width*((cloud.at(pnt_count).x-min_x)/(max_x-min_x)));
            int temp_y = trunc(img.height*((cloud.at(pnt_count).y-min_y)/(max_y-min_y)));
            int ptr_pos = temp_y * img.step + temp_x;

            //        ROS_INFO("Cloud temp_x:%d,temp_y:%d, pointer:%d,intensity:%f",
            //                 temp_x,temp_y,ptr_pos,cloud.at(pnt_count).intensity);
            //        ROS_INFO("Cloud points per grid:%d and intensity:%d before",points_per_cell[temp_x][temp_y],img.data[ptr_pos]);


            // Multiply with previous value and get the new average after adding the new value
            img.data[ptr_pos] = (uint16_t)((img.data[ptr_pos]*points_per_cell[temp_x][temp_y]+
                                            (65536*(cloud.at(pnt_count).intensity-min_intensity)/(max_intensity-min_intensity)))/
                                           (points_per_cell[temp_x][temp_y]+1));
            points_per_cell[temp_x][temp_y]++;

            //ROS_INFO("Image temp_x:%d,temp_y:%d, intensity:%d, points per cell:%d",temp_x,temp_y,img.data[ptr_pos],points_per_cell[temp_x][temp_y]);
        }

        cv::flip(img.data,img.data,1); //flip around y-axis
        ROS_INFO("Converted pcl to image, cols:%d, rows:%d, step:%d, size:%d",img.height,img.width,img.step,img.data.size());

    }

    void cloud_cb (const sensor_msgs::PointCloud2Ptr& input)
    {

        // Create a container for the data.
        sensor_msgs::PointCloud2 temp_pcl,output,output_plane,output_plane_intensity_grad,
                output_plane_elevation,output_plane_density,output_plane_pedig_rgb,cluster_out;

        sensor_msgs::Image::Ptr road_image (new sensor_msgs::Image);

        /// Perform Plane Segmentation

        // All the objects needed
        pcl::PassThrough<pcl::PointXYZI> pass;
        pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> ne;
        pcl::SACSegmentationFromNormals<pcl::PointXYZI, pcl::Normal> seg;
        pcl::PCDWriter writer;
        pcl::ExtractIndices<pcl::PointXYZI> extract;
        pcl::ExtractIndices<pcl::Normal> extract_normals;
        pcl::search::KdTree<pcl::PointXYZI>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZI> ());

        // Datasets
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZI> ());
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ersa (new pcl::PointCloud<pcl::PointXYZI> ());
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_plane_intensity_grad (new pcl::PointCloud<pcl::PointXYZI> ());
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_plane_density (new pcl::PointCloud<pcl::PointXYZI> ());
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_plane_elevation (new pcl::PointCloud<pcl::PointXYZI> ());
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane_pedig_rgb (new pcl::PointCloud<pcl::PointXYZRGB> ());
        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
        pcl::PointCloud<pcl::Normal>::Ptr cloud_plane_normals (new pcl::PointCloud<pcl::Normal>);
        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2 (new pcl::PointCloud<pcl::Normal>);
        pcl::PointCloud<pcl::Normal>::Ptr cloud_ersa_normals (new pcl::PointCloud<pcl::Normal>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered2 (new pcl::PointCloud<pcl::PointXYZI>);
        pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients), coefficients_cylinder (new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices), inliers_cylinder (new pcl::PointIndices);

        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZI>);

        // Use this to get the fields to have the same name so that Intensity can be converted too
        input->fields[3].name = "intensity";

        // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
        pcl::fromROSMsg(*input,*cloud);


        //    pcl::PointXYZI test_point = cloud->points.back();

        //    ROS_INFO("PointCloud has: %d data points, (%f,%f,%f,%f)",cloud->points.size(),test_point.data[0],test_point.data[1],test_point.data[2],test_point.data[3]);

        /// Fix the following passthrough filters to work with the /robot_base frame's position
        /// but also consider the orientation of the bicycle in space

        //    // Create the odom transform
        //    try{
        //        // wait to listen the old transform from "/fixed_frame" to "/robot_base"
        //        ros::Time previous_frame_time = odom.stamp_;
        //        listener.waitForTransform("/fixed_frame", "/robot_base",
        //                                  previous_frame_time, ros::Duration(1.0));
        //        listener.lookupTransform("/fixed_frame", "/robot_base",
        //                                 previous_frame_time, odom);
        //    }
        //    catch (tf::TransformException ex){
        //        ROS_ERROR("odom_publisher odom transform:%s",ex.what());
        //    }

        //    ROS_INFO("Robot base from fixed frame:(x=%f,y=%f,z=%f)",odom.getOrigin().getX(),odom.getOrigin().getY(),odom.getOrigin().getZ());

        //    // Build a passthrough filter to remove spurious NaNs
        //    pass.setInputCloud (cloud);
        //    pass.setFilterFieldName("z");
        //    pass.setFilterLimits (odom.getOrigin().getZ()-.1, odom.getOrigin().getZ()+2.0);
        //    pass.filter (*cloud_filtered);

        //    pass.setInputCloud (cloud_filtered);
        //    pass.setFilterFieldName("x");
        //    pass.setFilterLimits (odom.getOrigin().getX()-2.0, odom.getOrigin().getX()+10.0);
        //    pass.filter (*cloud_filtered);

        //    pass.setInputCloud (cloud_filtered);
        //    pass.setFilterFieldName("y");
        //    pass.setFilterLimits (odom.getOrigin().getY()-5.0, odom.getOrigin().getY()+5.0);
        //    pass.filter (*cloud_filtered);

        // Build a passthrough filter to remove spurious NaNs
        pass.setInputCloud (cloud);
        pass.setFilterFieldName("z");
        pass.setFilterLimits (0.0,2.0);
        pass.filter (*cloud_filtered);

        pass.setInputCloud (cloud_filtered);
        pass.setFilterFieldName("x");
        pass.setFilterLimits (-10.0, 10.0);
        pass.filter (*cloud_filtered);

        pass.setInputCloud (cloud_filtered);
        pass.setFilterFieldName("y");
        pass.setFilterLimits (-10.0, +10.0);  //only check the ground for now
        pass.filter (*cloud_filtered);


        ROS_INFO("PointCloud after filtering has: %d data points.",cloud_filtered->points.size ());
        //std::cerr << "PointCloud after filtering has: " << cloud_filtered->points.size () << " data points." << std::endl;

        //    // Create the filtering object for statistical outlier removal
        //    pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
        //    sor.setInputCloud (cloud_filtered);
        //    sor.setMeanK (50);
        //    sor.setStddevMulThresh (1.0);
        //    sor.filter (*cloud_filtered);
        //    ROS_INFO("PointCloud after statistical outlier removal has: %d data points.",cloud_filtered->points.size ());
        //    //std::cerr << "PointCloud after statistical outlier removal has: " << cloud_filtered->points.size () << " data points." << std::endl;

        //        // Downsampling
        //        pcl::PCLPointCloud2::Ptr pcl_cloud (new pcl::PCLPointCloud2 ());
        //        pcl::toROSMsg(*cloud_filtered,temp_pcl);
        //        pcl_conversions::toPCL(temp_pcl, *pcl_cloud);
        //        pcl::VoxelGrid<pcl::PCLPointCloud2> downsample;
        //        downsample.setInputCloud (pcl_cloud);
        //        downsample.setLeafSize (0.05, 0.05, 0.05);
        //        downsample.filter (*pcl_cloud);
        //        pcl_conversions::fromPCL(*pcl_cloud,temp_pcl);
        //        pcl::fromROSMsg(temp_pcl,*cloud_filtered);
        //        ROS_INFO("PointCloud after downsampling has: %d data points.",cloud_filtered->points.size ());
        //        //std::cerr << "PointCloud after downsampling has: " << cloud_filtered->points.size () << " data points." << std::endl;

        // Estimate point normals
        ne.setSearchMethod (tree);
        ne.setInputCloud (cloud_filtered);
        ne.setKSearch (50);  // the bigger the neighborhood, the smoother it gets, original 50
        ne.compute (*cloud_normals);

        // Create the segmentation object for the planar model and set all the parameters
        seg.setOptimizeCoefficients (true);
        seg.setModelType (pcl::SACMODEL_NORMAL_PLANE);
        seg.setNormalDistanceWeight (0.1);
        seg.setMethodType (pcl::SAC_RANSAC);
        seg.setMaxIterations (100);
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
        extract.filter (*cloud_filtered2);
        extract_normals.setNegative (true);
        extract_normals.setInputCloud (cloud_normals);
        extract_normals.setIndices (inliers_plane);
        extract_normals.filter (*cloud_normals2);
        extract_normals.setNegative(false);
        extract_normals.filter(*cloud_plane_normals);


        /// Here, write the methodology to process the data of the ground surface (intensity, normals, elevation)
        /// in order to detect ponds of water and other dangerous surface conditions

//        // Method 1: Try to convert the plane point cloud into a 2D image

//        //cloud_to_intensity_image (*cloud_plane,*road_image);


        // Method 2: Proceed with the extraction of the Point Elevation Density intensity Gradient (PEDIG) descriptor
        // for the points of the Extended Road Surface Area point cloud, using the neighbors within radius search of the octree

        float resolution = 0.005f;  // 10cm

        float octree_search_radius;

        // Create the Extended Road Surface Area (ERSA) point cloud
        extract.setNegative (false);
        extract.filter (*cloud_ersa); // same as cloud_plane to begin with

        pcl::octree::OctreePointCloudSearch<pcl::PointXYZI> octree_ersa (resolution);

        // Neighbors within octree_search_radius search

        std::vector<int> pointIdxRadiusSearch;
        std::vector<float> pointRadiusSquaredDistance;

        // Use the ERSA point cloud as input
        octree_ersa.setInputCloud (cloud_filtered);
        octree_ersa.addPointsFromInputCloud ();

        octree_search_radius = 0.03f;  // 20cm search radius around points for the creation of ERSA

        // Create boolean table for the points of cloud_filtered to know if they have been used already
        bool used_point[cloud_filtered->points.size()];
        for (int i=0;i<=cloud_filtered->points.size();i++)
            used_point[i] = false;

        for (int pnt_count=0;pnt_count<cloud_plane->size();pnt_count++)
        {
            pcl::PointXYZI searchPoint = cloud_plane->points.at(pnt_count);

            /// Rotate all of the points of ERSA to become flat
            //tf::Quaternion lidar_orientation(tf::createQuaternionFromRPY(3*M_PI/180,-5*M_PI/180,-7*M_PI/180));   //exp1_1_2
            //tf::Quaternion lidar_orientation(tf::createQuaternionFromRPY(1*M_PI/180,-5*M_PI/180,12*M_PI/180));   //exp2_1_2
            //tf::Quaternion lidar_orientation(tf::createQuaternionFromRPY(-5*M_PI/180,-6*M_PI/180,-10*M_PI/180));   //exp3_1_2
            //tf::Quaternion lidar_orientation(tf::createQuaternionFromRPY(1*M_PI/180,-5*M_PI/180,5*M_PI/180));   //exp4_1_1
            //tf::Quaternion lidar_orientation(tf::createQuaternionFromRPY(5*M_PI/180,1*M_PI/180,27*M_PI/180));   //exp5_1_1
            //tf::Quaternion lidar_orientation(tf::createQuaternionFromRPY(-5*M_PI/180,-1*M_PI/180,165*M_PI/180));   //exp6_1_1
            //tf::Quaternion lidar_orientation(tf::createQuaternionFromRPY(3.5*M_PI/180,-3*M_PI/180,34*M_PI/180));   //exp6_3_1
            //tf::Quaternion lidar_orientation(tf::createQuaternionFromRPY(6*M_PI/180,1*M_PI/180,8*M_PI/180));   //exp7_1_2
            //tf::Quaternion lidar_orientation(tf::createQuaternionFromRPY(5*M_PI/180,0*M_PI/180,-150*M_PI/180));   //exp8_1_1
            //tf::Quaternion lidar_orientation(tf::createQuaternionFromRPY(5*M_PI/180,0*M_PI/180,26*M_PI/180));   //exp8_3_3
            //tf::Quaternion lidar_orientation(tf::createQuaternionFromRPY(-3*M_PI/180,1*M_PI/180,-150*M_PI/180));   //exp9_1_1
            tf::Quaternion lidar_orientation(tf::createQuaternionFromRPY(-5*M_PI/180,0*M_PI/180,10*M_PI/180));   //exp10_1_1  turning...
            //tf::Quaternion lidar_orientation(tf::createQuaternionFromRPY(-1*M_PI/180,-3*M_PI/180,147*M_PI/180));   //exp11_1_1


            tf::Vector3 oldXYZ(cloud_plane->points[ pnt_count ].x,
                               cloud_plane->points[ pnt_count ].y,cloud_plane->points[ pnt_count ].z);
            tf::Vector3 rotatedXYZ(tf::quatRotate(lidar_orientation,oldXYZ));
            cloud_ersa->points[ pnt_count ].x = rotatedXYZ.x();
            cloud_ersa->points[ pnt_count ].y = rotatedXYZ.y();
            cloud_ersa->points[ pnt_count ].z = rotatedXYZ.z();

            //ROS_INFO("Examining plane point:%d which in filtered pcl is %d",pnt_count,inliers_plane->indices[pnt_count]);
            used_point[inliers_plane->indices[pnt_count] ] = true; // setting the plane point as used-inserted already

            if (octree_ersa.radiusSearch (searchPoint, octree_search_radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
            {
                for (size_t i = 0; i < pointIdxRadiusSearch.size (); ++i)
                {
                    // Add it if is alone in its r=10cm neighborhood, otherwise don't add again
                    if (!used_point[ pointIdxRadiusSearch[i] ])
                    {
                        tf::Vector3 oldXYZ(cloud_filtered->points[ pointIdxRadiusSearch[i] ].x,
                                cloud_filtered->points[ pointIdxRadiusSearch[i] ].y,cloud_filtered->points[ pointIdxRadiusSearch[i] ].z);
                        tf::Vector3 rotatedXYZ(tf::quatRotate(lidar_orientation,oldXYZ));
                        pcl::PointXYZI rotatedERSAPoint = cloud_filtered->points[ pointIdxRadiusSearch[i] ];
                        rotatedERSAPoint.x = rotatedXYZ.x();
                        rotatedERSAPoint.y = rotatedXYZ.y();
                        rotatedERSAPoint.z = rotatedXYZ.z();
                        cloud_ersa->points.push_back(rotatedERSAPoint);  // add the points around the point
                        cloud_ersa->width++;
                        used_point[ pointIdxRadiusSearch[i] ] = true; // setting the added point as used already
                    }
                }
            }
        }

        ROS_INFO("PointCloud: planar component has:%d, rest has:%d, ERSA has:%f points, width:%d.",
                 cloud_plane->points.size (),cloud_filtered2->points.size(),cloud_ersa->points.size (),cloud_ersa->width);

        // Estimate point normals for ERSA cloud
        ne.setSearchMethod (tree);
        ne.setInputCloud (cloud_ersa);
        ne.setKSearch (50);  // the bigger the neighborhood, the smoother it gets, original 50
        ne.compute (*cloud_ersa_normals);

        // Create the intensity, elevation, density point clouds same as ERSA
        *cloud_plane_intensity_grad = *cloud_ersa;  //same as cloud_ersa
        *cloud_plane_elevation = *cloud_ersa;  //same as cloud_ersa
        *cloud_plane_density = *cloud_ersa;  //same as cloud_ersa

        cloud_plane_pedig_rgb->header = cloud_ersa->header;
        cloud_plane_pedig_rgb->width = cloud_ersa->width;
        cloud_plane_pedig_rgb->height = cloud_ersa->height;
        cloud_plane_pedig_rgb->is_dense = cloud_ersa->is_dense;

        resolution = 0.005f;  // 5mm
        //resolution = 0.01f;  // 10cm
        pcl::octree::OctreePointCloudSearch<pcl::PointXYZI> octree (resolution);

        // Use the ERSA point cloud as input
        octree.setInputCloud (cloud_ersa);
        octree.addPointsFromInputCloud ();


        std::vector<float> intensity_grad;
        std::vector<float> intensity_grad_theta;
        std::vector<int> points_density;
        std::vector<float> bilat_density_dif;
        std::vector<float> bilat_density_dif_theta;
        std::vector<float> mean_elevation_dif;
        //        std::vector<float> mean_elevation_dif_theta;

        float temp_bilat_density_dif = 0.0;
        float temp_intensity_grad = 0.0;
        float temp_elevation_grad = 0.0;
        float temp_mean_elevation_dif = 0.0;

        float intensity_grad_min, intensity_grad_max;
        float density_min, density_max;
        float elevation_min, elevation_max;


        //        // Limit ERSA for testings
        //        pass.setInputCloud (cloud_ersa);
        //        pass.setFilterFieldName("x");
        //        pass.setFilterLimits (-1.0, 1.0);
        //        pass.filter (*cloud_ersa);

        //        pass.setInputCloud (cloud_ersa);
        //        pass.setFilterFieldName("y");
        //        pass.setFilterLimits (-10.0, +10.0);
        //        pass.filter (*cloud_ersa);

        // Use the ERSA point cloud here too
        for (int pnt_count=0;pnt_count<cloud_ersa->size();pnt_count++)
        {

            pcl::PointXYZI searchPoint = cloud_ersa->points.at(pnt_count);

            /// Intensity part

            octree_search_radius = 0.04f;  // 40cm search radius around points for intensity

            float intensity_grad_x = 0.0, intensity_grad_y = 0.0;

            if (octree.radiusSearch (searchPoint, octree_search_radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
            {
                for (size_t i = 0; i < pointIdxRadiusSearch.size (); ++i)
                {
                    if (pointRadiusSquaredDistance[i]!=0.0)
                    {
                        // Calculation of extended gradient
                        // Calculate angle theta between the point and its neighbor on the xy-plane
                        float theta = atan2(searchPoint.y-cloud_ersa->points[ pointIdxRadiusSearch[i] ].y,searchPoint.x-cloud_ersa->points[ pointIdxRadiusSearch[i] ].x);
                        // Project their distance on the xy-plane
                        float dist_xy = sqrt(sqrt(pointRadiusSquaredDistance[i])-pow((searchPoint.z-cloud_ersa->points[ pointIdxRadiusSearch[i] ].z),2));
                        // Calculate the gradient for the x and y axes
                        //                        float temp_intensity_grad_x = pow((searchPoint.intensity-cloud_plane->points[ pointIdxRadiusSearch[i] ].intensity),2)*cos(theta)/dist_xy;
                        //                        float temp_intensity_grad_y = pow((searchPoint.intensity-cloud_plane->points[ pointIdxRadiusSearch[i] ].intensity),2)*sin(theta)/dist_xy;

                        float temp_intensity_grad_x = (searchPoint.intensity-cloud_ersa->points[ pointIdxRadiusSearch[i] ].intensity)*cos(theta)/dist_xy;
                        float temp_intensity_grad_y = (searchPoint.intensity-cloud_ersa->points[ pointIdxRadiusSearch[i] ].intensity)*sin(theta)/dist_xy;

                        // Calculate the resultant in x and y axes
                        intensity_grad_x += temp_intensity_grad_x;
                        intensity_grad_y += temp_intensity_grad_y;
                        //ROS_INFO("theta:%f, temp_intensity_grad_x:%f, temp_intensity_grad_y:%f at point:%d",theta,temp_intensity_grad_x,temp_intensity_grad_y,pnt_count);

                    }
                }

                // Push back the point density of the point (neighbor count)
                points_density.push_back(pointIdxRadiusSearch.size ());
            }

            // Calculate the resultant and its direction
            temp_intensity_grad = sqrt(pow(intensity_grad_x,2)+pow(intensity_grad_y,2));
            intensity_grad.push_back(temp_intensity_grad);
            intensity_grad_theta.push_back(atan2(intensity_grad_y,intensity_grad_x));

            cloud_plane_intensity_grad->points[pnt_count].intensity = temp_intensity_grad;

            //            if (cloud_plane_intensity_grad->points[pnt_count].intensity <15000)  // that's the threshold according to the search radius
            //                cloud_plane_intensity_grad->points.erase(cloud_plane_intensity_grad->points[pnt_count]);

            if (pnt_count==0)
            {
                intensity_grad_max = intensity_grad_min = temp_intensity_grad;
            }
            if (temp_intensity_grad>intensity_grad_max)
                intensity_grad_max = temp_intensity_grad;
            if (temp_intensity_grad<intensity_grad_min)
                intensity_grad_min = temp_intensity_grad;

            //            /// Elevation part

            //            octree_search_radius = 0.01f;  // 10cm search radius around points for elevation gradient calculation

            //            float elevation_grad_x = 0.0, elevation_grad_y = 0.0;

            //            if (octree.radiusSearch (searchPoint, octree_search_radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
            //            {
            //                for (size_t i = 0; i < pointIdxRadiusSearch.size (); ++i)
            //                {
            //                    if (pointRadiusSquaredDistance[i]!=0.0)
            //                    {
            //                        // Calculation of extended gradient
            //                        // Calculate angle theta between the point and its neighbor on the xy-plane
            //                        float theta = atan2(searchPoint.y-cloud_ersa->points[ pointIdxRadiusSearch[i] ].y,searchPoint.x-cloud_ersa->points[ pointIdxRadiusSearch[i] ].x);
            //                        // Project their distance on the xy-plane
            //                        float dist_xy = sqrt(sqrt(pointRadiusSquaredDistance[i])-pow((searchPoint.z-cloud_ersa->points[ pointIdxRadiusSearch[i] ].z),2));

            //                        float temp_elevation_grad_x = (searchPoint.z-cloud_ersa->points[ pointIdxRadiusSearch[i] ].z)*cos(theta)/dist_xy;
            //                        float temp_elevation_grad_y = (searchPoint.z-cloud_ersa->points[ pointIdxRadiusSearch[i] ].z)*sin(theta)/dist_xy;

            //                        // Calculate the resultant in x and y axes
            //                        elevation_grad_x += temp_elevation_grad_x;
            //                        elevation_grad_y += temp_elevation_grad_y;
            //                        //ROS_INFO("theta:%f, temp_elevation_grad_x:%f, temp_elevation_grad_y:%f at point:%d",theta,temp_elevation_grad_x,temp_elevation_grad_y,pnt_count);

            //                    }
            //                }
            //            }

            //            temp_elevation_grad = sqrt(pow(elevation_grad_x,2)+pow(elevation_grad_y,2));
            //            temp_elevation_grad = elevation_grad_y;
            //            mean_elevation_dif.push_back(temp_elevation_grad);

            //            cloud_plane_elevation->points[pnt_count].intensity = temp_elevation_grad;
            //            //cloud_plane_elevation->points[pnt_count].intensity = cloud_ersa_normals->at(pnt_count).curvature;

            //            if (pnt_count==0)
            //            {
            //                elevation_max = elevation_min = temp_elevation_grad;
            //            }
            //            if (temp_elevation_grad>elevation_max)
            //                elevation_max = temp_elevation_grad;
            //            if (temp_elevation_grad<elevation_min)
            //                elevation_min = temp_elevation_grad;

            /// Elevation part

            float mean_elevation;

            octree_search_radius = 0.001f;  // 1cm search radius around points for bilateral density difference calculation (and elevation for now)

            if (octree.radiusSearch (searchPoint, octree_search_radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
            {
                if (pointIdxRadiusSearch.size ()!=0)
                {
                    for (size_t i = 0; i < pointIdxRadiusSearch.size (); ++i)
                    {
                        if (pointRadiusSquaredDistance[i]!=0.0)
                        {
                            // Calculate the resultant elevation in x and y axes
                            mean_elevation += cloud_ersa->points[ pointIdxRadiusSearch[i] ].z;
                        }
                    }
                    mean_elevation = mean_elevation/pointIdxRadiusSearch.size ();
                }
                else
                {
                    mean_elevation =searchPoint.z;
                    ROS_INFO("Couldn't find any points near.");
                }
            }

            temp_mean_elevation_dif = 100*(searchPoint.z-mean_elevation);
            mean_elevation_dif.push_back(temp_mean_elevation_dif);  //increase distinguishability

            //ROS_INFO("point.z:%f, mean_elevation:%f, temp_mean_elevation_dif:%f at point:%d",searchPoint.z,mean_elevation,temp_mean_elevation_dif,pnt_count);

            cloud_plane_elevation->points[pnt_count].intensity = temp_mean_elevation_dif;

            if (pnt_count==0)
            {
                elevation_max = elevation_min = temp_mean_elevation_dif;
            }
            if (temp_mean_elevation_dif>elevation_max)
                elevation_max = temp_mean_elevation_dif;
            if (temp_mean_elevation_dif<elevation_min)
                elevation_min = temp_mean_elevation_dif;


            /// Bilateral Density Difference part

            float bilat_density_r = 0.0, bilat_density_rp = 0.0;
            float w_r = 0.9, w_rp = 0.1;  // weights for the density difference on the radius direction and the perpendicular to the radius direction

            octree_search_radius = 0.06f;  // 60cm search radius around points for density calculation

            if (octree.radiusSearch (searchPoint, octree_search_radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
            {
                for (size_t i = 0; i < pointIdxRadiusSearch.size (); ++i)
                {
                    if (pointRadiusSquaredDistance[i]!=0.0)
                    {
                        //                        float inclination = (odom.getOrigin().getY()-searchPoint.y)/(odom.getOrigin().getX()-searchPoint.x);

                        //                        // Calculation of bilateral point density on the direction rp, perpendicular to the range
                        //                        if ((cloud_ersa->points[ pointIdxRadiusSearch[i] ].y-searchPoint.y) >
                        //                                (inclination)*(cloud_ersa->points[ pointIdxRadiusSearch[i] ].x-searchPoint.x))
                        //                            bilat_density_rp ++;
                        //                        else
                        //                            bilat_density_rp --;

                        //                        // Calculation of bilateral point density on the direction r, of the range
                        //                        if ((cloud_ersa->points[ pointIdxRadiusSearch[i] ].y-searchPoint.y) >
                        //                                -(inclination)*(cloud_ersa->points[ pointIdxRadiusSearch[i] ].x-searchPoint.x))
                        //                            bilat_density_r ++;
                        //                        else
                        //                            bilat_density_r --;

                        // Calculation of bilateral point density on the x direction
                        if (cloud_ersa->points[ pointIdxRadiusSearch[i] ].x>searchPoint.x)
                            bilat_density_r ++;
                        else
                            bilat_density_r --;

                        // Calculation of bilateral point density on the y direction
                        if (cloud_ersa->points[ pointIdxRadiusSearch[i] ].y>searchPoint.y)
                            bilat_density_rp ++;
                        else
                            bilat_density_rp --;


                    }
                }
            }

            //ROS_INFO("bilat_density_rp:%f, bilat_density_r:%f at point:%d",bilat_density_rp,bilat_density_r,pnt_count);
            // Calculate the resultant and its direction
            float temp_bilat_density_dif = sqrt(w_r*pow(100*bilat_density_r/pointIdxRadiusSearch.size (),6)+w_rp*pow(100*bilat_density_rp/pointIdxRadiusSearch.size (),6));
            //            float temp_bilat_density_dif1 = sqrt(w_r*pow(bilat_density_r,10)+w_rp*pow(bilat_density_rp,10));
            //            float temp_bilat_density_dif2 = pow(100*bilat_density_rp/pointIdxRadiusSearch.size (),2);
            //            float temp_bilat_density_dif3 = pow(100*bilat_density_r/pointIdxRadiusSearch.size (),2);

            //            ROS_INFO("bilat_density_rp:%f, bilat_density_r:%f, temp_bilat_density_dif:%f, temp_bilat_density_dif:%f, temp_bilat_density_dif:%f, temp_bilat_density_dif:%f, at point:%d",
            //                     bilat_density_rp,bilat_density_r,temp_bilat_density_dif,temp_bilat_density_dif1,temp_bilat_density_dif2,temp_bilat_density_dif3,pnt_count);

            bilat_density_dif.push_back(temp_bilat_density_dif);  //check how to normalize?

            bilat_density_dif_theta.push_back(atan2(bilat_density_r,bilat_density_rp));

            cloud_plane_density->points[pnt_count].intensity = temp_bilat_density_dif;

            if (pnt_count==0)
            {
                density_max = density_min = temp_bilat_density_dif;
            }
            if (temp_bilat_density_dif>density_max)
                density_max = temp_bilat_density_dif;
            if (temp_bilat_density_dif<density_min)
                density_min = temp_bilat_density_dif;


            //            ROS_INFO("temp_intensity_grad:%f, temp_elevation_grad:%f, temp_bilat_density_dif:%f at point:%d",
            //                     temp_intensity_grad,temp_elevation_grad,temp_bilat_density_dif,pnt_count);

        }

        ROS_INFO("intensity_grad min:%f, max:%f, elevation_dif min:%f, max:%f, bilat_density_dif min:%f, max:%f at point:%d",
                 intensity_grad_min, intensity_grad_max, elevation_min, elevation_max, density_min, density_max);

        int added_blue=0;

        for (int pnt_count=0;pnt_count<cloud_ersa->size();pnt_count++)
        {

            // Create the PEDIG RGB transformation
            pcl::PointXYZRGB pedi_point;

            pedi_point.x = cloud_ersa->points[ pnt_count ].x;
            pedi_point.y = cloud_ersa->points[ pnt_count ].y;
            pedi_point.z = cloud_ersa->points[ pnt_count ].z;

            // Creation of the PEDI Gradient Descriptor
            // pack elevation/density/internsity gradient into rgb, they need to be normalized

            //Eliminate close points with not too high IG
            if ((cloud_ersa->points.at(pnt_count).x<1.3)&&(cloud_ersa->points.at(pnt_count).x>-3.5)
                    &&(cloud_ersa->points.at(pnt_count).y<1.5)&&(cloud_ersa->points.at(pnt_count).y>-1.5)
                    &&(intensity_grad[pnt_count]<2500))
                intensity_grad[pnt_count] = 0;

            uint8_t  r,g,b;

            // Color lookup table
            // black   rgb(0, 0, 0)
            // gray    rgb(128, 128, 128)
            // silver  rgb(192, 192, 192)
            // white   rgb(255, 255, 255)
            // maroon  rgb(128, 0, 0)
            // red     rgb(255, 0, 0)
            // olive   rgb(128, 128, 0)
            // yellow  rgb(255, 255, 0)
            // green   rgb(0, 128, 0)
            // lime    rgb(0, 255, 0)
            // teal    rgb(0, 128, 128)
            // aqua    rgb(0, 255, 255)
            // navy    rgb(0, 0, 128)
            // blue    rgb(0, 0, 255)
            // purple  rgb(128, 0, 128)
            // fuchsia rgb(255, 0, 255)
            // orange  rgb(255,93,0)

            //Don't map them to RGB directly, select only the areas that have high E-D-IG and give them specific colors

            pcl::PointXYZI searchPoint = cloud_ersa->points.at(pnt_count);
            octree_search_radius = 0.03f;  // 5cm search radius around points for neighbors calculation

            float IG_neighbors_ratio_threshold = 0.01;  // the ratio of neighbors with high intensity grad (IG) threshold to be considered a region of high IG
            float BDD_neighbors_ratio_threshold = 0.1;  // the ratio of neighbors with high bilat.density dif.(BDD) threshold to be considered a region of high BDD
            float MED_high_neighbors_ratio_threshold = 0.5;  // the ratio of neighbors with high mean elevation dif.(MED) threshold to be considered a region of high MED
            float MED_low_neighbors_ratio_threshold = 0.5;  // the ratio of neighbors with low mean elevation dif.(MED) threshold to be considered a region of low MED

            double IG_high_threshold = 800;
            double BDD_high_threshold = 100000;
            double MED_high_threshold = 8;
            double MED_low_threshold = -3;

            int IG_high_neighbors_counter = 0;
            int BDD_high_neighbors_counter = 0;
            int MED_high_neighbors_counter = 0;
            int MED_low_neighbors_counter = 0;

            if ((intensity_grad[pnt_count] >= IG_high_threshold-200) &&
                    (bilat_density_dif[pnt_count] >= BDD_high_threshold))
            {
                // purple for water pond
                r=(uint8_t)128;
                g=(uint8_t)0;
                b=(uint8_t)128;
            }
            else if ((intensity_grad[pnt_count] >= IG_high_threshold) &&
                     (bilat_density_dif[pnt_count] < BDD_high_threshold) &&
                     (mean_elevation_dif[pnt_count] < MED_high_threshold) &&
                     (mean_elevation_dif[pnt_count] >= MED_low_threshold))
            {
                // blue for road markings/patterns
                r=(uint8_t)0;
                g=(uint8_t)0;
                b=(uint8_t)255;
            }
//            else if ((intensity_grad[pnt_count] < IG_high_threshold) &&
//                     (bilat_density_dif[pnt_count] >= BDD_high_threshold) &&
//                     (mean_elevation_dif[pnt_count] < MED_high_threshold) &&
//                     (mean_elevation_dif[pnt_count] >= MED_low_threshold)
//                     )
//            {
//                // green for area around no data
//                r=(uint8_t)0;
//                g=(uint8_t)255;
//                b=(uint8_t)0;
//            }
            else if (mean_elevation_dif[pnt_count] >= MED_high_threshold)
            {
                // orange for elevated objects/obstacles
                r=(uint8_t)255;
                g=(uint8_t)93;
                b=(uint8_t)0;
            }
            else if (mean_elevation_dif[pnt_count] < MED_low_threshold)
            {
                // yellow for submerged objects/ditches/holes
                r=(uint8_t)255;
                g=(uint8_t)255;
                b=(uint8_t)0;
            }
            else
            {
//                // check the neighbors
//                if (octree.radiusSearch (searchPoint, octree_search_radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
//                {

//                    for (size_t i = 0; i < pointIdxRadiusSearch.size (); ++i)
//                    {
//                        if (intensity_grad[ pointIdxRadiusSearch[i] ] >= IG_high_threshold)
//                            IG_high_neighbors_counter++;
//                        if (bilat_density_dif[ pointIdxRadiusSearch[i] ] >= BDD_high_threshold)
//                            BDD_high_neighbors_counter++;
//                        if (mean_elevation_dif[ pointIdxRadiusSearch[i] ] >= MED_high_threshold)
//                            MED_high_neighbors_counter++;
//                        if (mean_elevation_dif[ pointIdxRadiusSearch[i] ] < MED_low_threshold)
//                            MED_low_neighbors_counter++;
//                    }

//                    // if they are gray, check if they are surrounded by other colors, then change them too

//                    //                if ((IG_high_neighbors_counter/pointIdxRadiusSearch.size ()>=IG_neighbors_ratio_threshold) &&
//                    //                    (BDD_high_neighbors_counter/pointIdxRadiusSearch.size ()>=BDD_neighbors_ratio_threshold))
//                    //                    {
//                    //                        // purple for water pond
//                    //                        r=(uint8_t)128;
//                    //                        g=(uint8_t)0;
//                    //                        b=(uint8_t)128;
//                    //                    }
////                    if ((IG_high_neighbors_counter/pointIdxRadiusSearch.size ()>=IG_neighbors_ratio_threshold)   // high IG
//                            //                           && (BDD_high_neighbors_counter/pointIdxRadiusSearch.size ()<BDD_neighbors_ratio_threshold)       // low BDD
//                            //                           && (MED_high_neighbors_counter/pointIdxRadiusSearch.size ()<MED_high_neighbors_ratio_threshold)  // not high MED
//                            //                           && (MED_low_neighbors_counter/pointIdxRadiusSearch.size ()<MED_low_neighbors_ratio_threshold)     // not low MED
//                           // )
//                    if (IG_high_neighbors_counter>=10)
//                    {
//                        // blue for road markings/patterns
//                        r=(uint8_t)0;
//                        g=(uint8_t)0;
//                        b=(uint8_t)255;
//                        added_blue++;
//                    }
//                    //                                else if ((IG_high_neighbors_counter/pointIdxRadiusSearch.size ()>=IG_neighbors_ratio_threshold) &&  // high IG
//                    //                                    (BDD_high_neighbors_counter/pointIdxRadiusSearch.size ()<BDD_neighbors_ratio_threshold) &&      // low BDD
//                    //                                    (MED_high_neighbors_counter/pointIdxRadiusSearch.size ()<MED_high_neighbors_ratio_threshold) && // not high MED
//                    //                                    (MED_low_neighbors_counter/pointIdxRadiusSearch.size ()<MED_low_neighbors_ratio_threshold))     // not low MED

//                    //                else if ((IG_high_neighbors_counter/pointIdxRadiusSearch.size ()<IG_neighbors_ratio_threshold) &&
//                    //                    (BDD_high_neighbors_counter/pointIdxRadiusSearch.size ()>=BDD_neighbors_ratio_threshold))
//                    else
//                    {
//                        // gray for the rest
//                        r=(uint8_t)128;
//                        g=(uint8_t)128;
//                        b=(uint8_t)128;
//                    }
//                    ROS_INFO("IG_high_neighbors_counter:%d, BDD_high_neighbors_counter:%d, MED_high_neighbors_counter:%d, MED_low_neighbors_counter:%d, neighbors:%d ",
//                             IG_high_neighbors_counter,BDD_high_neighbors_counter,MED_high_neighbors_counter,MED_low_neighbors_counter,pointIdxRadiusSearch.size ());
//                }
//                else
//                {
                    // gray if no neighbors
                    r=(uint8_t)128;
                    g=(uint8_t)128;
                    b=(uint8_t)128;
//                }

            }


            //            // Direct mapping to RGB
            //            uint8_t  r = (uint8_t)(1*255*(mean_elevation_dif[pnt_count]-(-5))/(10-(-5)));  // -5to10 for exp1,exp2, exp3
            //            if (r>255)
            //                r=255;
            //            if (r<0)
            //                r=0;
            //            //uint8_t r = (uint8_t)(1*255*(mean_elevation_dif[pnt_count]-elevation_min )/(elevation_max-elevation_min));
            //            uint8_t g = (uint8_t)(1*255*(bilat_density_dif[pnt_count]-density_min)/(density_max-density_min));
            //            //uint8_t g = 128;
            //            uint8_t  b = (uint8_t)(255*(intensity_grad[pnt_count]-0)/(3500-0));  // 3500 for exp1,exp2, exp3
            //            if (b>255)
            //                b=255;
            //            //uint8_t b = (uint8_t)(1*255*(intensity_grad[pnt_count]-intensity_grad_min)/(intensity_grad_max-intensity_grad_min));    // r->elevation, g->density, b->intensity/reflectivity
            //            //uint32_t rgb = ((uint32_t)(r) << 16 | (uint32_t)(g) << 8 | (uint32_t)(b));
            //            //uint32_t rgb = ((uint32_t)(255-r) << 16 | (uint32_t)(255-g) << 8 | (uint32_t)(255-b));

            uint32_t rgb = ((uint32_t)(r) << 16 | (uint32_t)(g) << 8 | (uint32_t)(b));
            pedi_point.rgb = *reinterpret_cast<float*>(&rgb);

            cloud_plane_pedig_rgb->points.push_back(pedi_point);

        }

        ROS_INFO("Added blue:%d", added_blue);


        /// Here, is the code for segmentation of the other obstacles around the vehicle

//                // Creating the KdTree object for the search method of the extraction
//                tree->setInputCloud (cloud_filtered2);

//                std::vector<pcl::PointIndices> cluster_indices;
//                pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
//                ec.setClusterTolerance (0.05); // 5cm
//                ec.setMinClusterSize (100);
//                ec.setMaxClusterSize (25000);
//                ec.setSearchMethod (tree);
//                ec.setInputCloud (cloud_filtered2);
//                ec.extract (cluster_indices);

//                extract.filter(*cloud_cluster);  //make it the same as cloud_filtered2 for now

//                int j = 0;

//                std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clusters;
//                for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
//                {
//                    cloud_cluster->points.clear();

//                    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
//                        cloud_cluster->points.push_back (cloud_filtered2->points[*pit]); //*
//                    cloud_cluster->width = cloud_cluster->points.size ();
//                    cloud_cluster->height = 1;
//                    cloud_cluster->is_dense = true;
//                    cloud_cluster->sensor_orientation_ = cloud_filtered2->sensor_orientation_;
//                    cloud_cluster->sensor_origin_ = cloud_filtered2->sensor_origin_;


//                    std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;

//                    std::stringstream ss;
//                    //      ss << "cloud_cluster_" << j;
//                    //      pub_cluster = pcl_nh.advertise<sensor_msgs::PointCloud2> (ss.str(), 1);
//                    //      pcl::toROSMsg(*cloud_cluster,cluster_out);
//                    //      pub_cluster.publish(cluster_out);

//                    clusters.push_back(cloud_cluster);

//                    if (j==0)
//                    {
//                        pcl::toROSMsg(*cloud_cluster,cluster_out);
//                        pub_cluster_0.publish(cluster_out);
//                    }
//                    if (j==1)
//                    {
//                        pcl::toROSMsg(*cloud_cluster,cluster_out);
//                        pub_cluster_1.publish(cluster_out);
//                    }
//                    if (j==2)
//                    {
//                        pcl::toROSMsg(*cloud_cluster,cluster_out);
//                        pub_cluster_2.publish(cluster_out);
//                    }
//                    if (j==3)
//                    {
//                        pcl::toROSMsg(*cloud_cluster,cluster_out);
//                        pub_cluster_3.publish(cluster_out);
//                    }
//                    if (j==4)
//                    {
//                        pcl::toROSMsg(*cloud_cluster,cluster_out);
//                        pub_cluster_4.publish(cluster_out);
//                    }
//                    if (j==5)
//                    {
//                        pcl::toROSMsg(*cloud_cluster,cluster_out);
//                        pub_cluster_5.publish(cluster_out);
//                    }
//                    if (j==6)
//                    {
//                        pcl::toROSMsg(*cloud_cluster,cluster_out);
//                        pub_cluster_6.publish(cluster_out);
//                    }
//                    if (j==7)
//                    {
//                        pcl::toROSMsg(*cloud_cluster,cluster_out);
//                        pub_cluster_7.publish(cluster_out);
//                    }
//                    if (j==8)
//                    {
//                        pcl::toROSMsg(*cloud_cluster,cluster_out);
//                        pub_cluster_8.publish(cluster_out);
//                    }




//                    //      ss << "cloud_cluster_" << j << ".pcd";
//                    //      writer.write<pcl::PointXYZI> (ss.str (), *cloud_cluster, false); //*
//                    j++;

//                }


        // Convert the pcl/PointCloud data to sensor_msgs/PointCloud2
        pcl::toROSMsg(*cloud_filtered2,output);
        //pcl::toROSMsg(*cloud_cylinder,output);
        pcl::toROSMsg(*cloud_plane,output_plane);
        pcl::toROSMsg(*cloud_plane_intensity_grad,output_plane_intensity_grad);
        pcl::toROSMsg(*cloud_plane_elevation,output_plane_elevation);
        pcl::toROSMsg(*cloud_plane_density,output_plane_density);
        pcl::toROSMsg(*cloud_plane_pedig_rgb,output_plane_pedig_rgb);



        /// Publish the data.
        pub.publish (output);
        pub_1.publish (output_plane);
        pub_plane_intensity_grad.publish(output_plane_intensity_grad);
        pub_plane_elevation.publish(output_plane_elevation);
        pub_plane_density.publish(output_plane_density);
        pub_plane_pedig_rgb.publish(output_plane_pedig_rgb);
        //road_image_pub.publish (*road_image);

    }

};

int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "pcl_process");
    ros::NodeHandle nh;

    PointCloudProcess pcl_process;

    while(ros::ok())
    {
        ros::spinOnce();
        //rate.sleep();
    }

    return 0;
}

