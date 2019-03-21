// Code for collecting all the objects detected by Computer Vision module and then providing them to the Localization module in
// the correct format and for collecting the particle set computed in each iteration of the 'particle_filter_localization.cpp' code
// and constructing the final message to be published
// dependencies: 'vision/line.h','vision/corner.h','vision/goal.h','vision/obstacle.h' header files for cv objects
//               'particle_filter_localization.h' header file for the particle filter localization including all classes and finctions
//
// Maintainer: Sotirios Stasinopoulos email: sotstas@gmail.com
// Last edited: 2014.06.24


#include <ros/ros.h>
#include <math.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <time.h>
#include <tf/transform_listener.h>

#include "vision/LinesLandmarks.h"
#include "vision/Goal.h"
#include "vision/Ball.h"
#include "vision/Obstacle.h"
#include "particle_filter_localization.h"
#include "localization/OutputData.h"
#include "localization/ParticleSet.h"
#include "localization/MeanPoseConfStamped.h"


#include <boost/foreach.hpp>




class LocalizationDataInputOutput
{
    ros::NodeHandle ldio_nh;

    ros::Subscriber sub_vision_lines;
    ros::Subscriber sub_vision_corners;
    ros::Subscriber sub_vision_goal;
    ros::Subscriber sub_vision_ball;
    ros::Subscriber sub_vision_obstacles;
    ros::Subscriber sub_localization_particle_set;
    ros::Subscriber sub_localization_mean_pose;

    // subscribe to topic for the compass heading from the IMU
    ros::Subscriber sub_compass;

public:
    ros::Publisher pub_localization_world_objects;
    ros::Publisher pub_localization_output_data;

    localization::WorldObjects msg_world_objects;
    localization::OutputData msg_output_data;

    tf::TransformListener data_input_tf;
    tf::Stamped<tf::Pose> keeper_right_field_pose;
    tf::Stamped<tf::Pose> keeper_left_field_pose;
    tf::Stamped<tf::Pose> obstacle_field_pose;
    std::vector<tf::Stamped<tf::Pose> > previous_obstacle_field_pose;
    tf::Stamped<tf::Pose> ball_field_pose;

    int robot_pose_confidence_threshold; // threshold above which we trust the new mean robot pose on field
                                         // to update the positions of other objects on the field

    int world_objects_max_number; //threshold for max items of world objects to be examined once by the particle filter
                                  // if more than that, the array gets cleared
    int obstacle_max_number;   //max number of obstacles that can be detected

    // Variables to be used for matching image pixels to field points
    int resized_image_width;
    int resized_image_height;
    float camera_height;
    float focal_length;



    LocalizationDataInputOutput()
    {

        sub_vision_lines = ldio_nh.subscribe("vision/lines_landmarks",10,&LocalizationDataInputOutput::visionLinesLandmarksCallback,this);
        sub_vision_goal = ldio_nh.subscribe("vision/goal",10,&LocalizationDataInputOutput::visionGoalCallback,this);
        sub_vision_ball = ldio_nh.subscribe("vision/ball",10,&LocalizationDataInputOutput::visionBallCallback,this);
        sub_vision_obstacles = ldio_nh.subscribe("vision/obstacle",10,&LocalizationDataInputOutput::visionObstacleCallback,this);
        sub_localization_particle_set = ldio_nh.subscribe("localization/particle_set",10,&LocalizationDataInputOutput::localizationParticleSetCallback,this);
        sub_localization_mean_pose = ldio_nh.subscribe("localization/mean_pose_conf_stamped",10,&LocalizationDataInputOutput::localizationMeanPoseCallback,this);
        pub_localization_world_objects = ldio_nh.advertise<localization::WorldObjects>("localization/world_objects_detected",10);
        pub_localization_output_data = ldio_nh.advertise<localization::OutputData>("localization/output_data",10);

        sub_compass = ldio_nh.subscribe("/imu/data", 10, &LocalizationDataInputOutput::compassCallback, this);

        //Initialize world_objects and output_data
        msg_output_data.bBallWasSeen = 0;
        ball_field_pose = tf::Stamped<tf::Pose>(
                    tf::Transform(tf::createIdentityQuaternion(), tf::Vector3(1.5,0.2,0)),ros::Time::now(), "/field");;
        msg_output_data.bObstacleWasSeen=0;
        obstacle_field_pose = tf::Stamped<tf::Pose>(
                    tf::Transform(tf::createIdentityQuaternion(), tf::Vector3(-0.5,-0.2,0)),ros::Time::now(), "/field");
        msg_output_data.bKeeperWasSeen = 0;
        keeper_right_field_pose = tf::Stamped<tf::Pose>(
                    tf::Transform(tf::createIdentityQuaternion(), tf::Vector3(-3.0,0.2,0)),ros::Time::now(), "/field");
        keeper_left_field_pose = tf::Stamped<tf::Pose>(
                    tf::Transform(tf::createIdentityQuaternion(), tf::Vector3(-3.0,-0.2,0)),ros::Time::now(), "/field");
        msg_world_objects.lines.clear();
        msg_world_objects.objects.clear();
        msg_world_objects.obstacles.clear();
        robot_pose_confidence_threshold = 0.4;
        world_objects_max_number = 25;
        obstacle_max_number = 5;

        for (int j=0;j<obstacle_max_number;j++)
            previous_obstacle_field_pose.push_back(obstacle_field_pose);


        if (!ldio_nh.getParam("/vision/image_rectifier_resizer/resized_width", resized_image_width))
        {
            ROS_INFO("Didn't find resized_width on parameter server");
            resized_image_width=480;
        }
        if (!ldio_nh.getParam("/vision/image_rectifier_resizer/resized_height", resized_image_height))
            resized_image_height=360;
        camera_height = 1.25;
        focal_length = 0.050; //TODO: calculate this...

    }

    ~LocalizationDataInputOutput()
    {

    }

// Function that takes an image point Vec2f I(x,y) and returns the position of that point within the "\robot_head" frame
// Although it returns a 3D point, everything is calculated on the head, meaning z=camera_height
    Vec3f imagePointToRobotHeadPoint(Vec2f image_point)
    {
        Vec3f robot_head_point;
        float robot_x,robot_y;

        // using f_x,f_y look-up table
        int f_LUT_x[480][360];
        int f_LUT_y[480][360];
        std::ifstream fin_x;
        std::ifstream fin_y;
        fin_x.open("/home/robotlab719/robocup_ws/src/localization/src/f_LUT_x.txt");
        fin_y.open("/home/robotlab719/robocup_ws/src/localization/src/f_LUT_y.txt");
        for(int i = 0; i < 180; i++)
             {
               for(int j = 0; j < 480; j++)
                {
                   f_LUT_x[j][i] = 0;
                   f_LUT_y[j][i] = 0;
                }
              }
        for(int i = 180; i < 360; i++)
             {
               for(int j = 0; j < 480; j++)
                {
                   fin_x >> f_LUT_x[j][i];
                   fin_y >> f_LUT_y[j][i];
                }
              }

        robot_y = f_LUT_y[int(image_point.x())][int(image_point.y())]*camera_height/(image_point.y()-resized_image_height/2+1);
        //ROS_ERROR("localization out: f_lut_y seen at [%lf]",robot_y);
        robot_x = (image_point.x()-resized_image_width/2+1)*sqrt(pow(camera_height,2)+pow(robot_y,2))/
                sqrt(pow(f_LUT_x[int(image_point.x())][int(image_point.y())],2)+pow(image_point.y()-resized_image_height/2+1,2));

        robot_head_point = Vec3f(robot_y,robot_x,camera_height); // pay attention that we need the robot_head_pont.x to be the
        // y-axis of the image and the robot_head_point.y to be the x-axis of the image

        return robot_head_point;
    }

    void visionLinesLandmarksCallback(const vision::LinesLandmarksConstPtr& lines_landmarks_msg)
    {
        BOOST_FOREACH(const vision::Line& temp_line, lines_landmarks_msg->lines)
        {
//            msg_world_objects.lines.push_back(temp_line);
        };

        BOOST_FOREACH(const vision::Landmark& temp_landmark, lines_landmarks_msg->landmarks)
        {

            localization::ObjectsDetected temp_obj;
            temp_obj.pose.x = imagePointToRobotHeadPoint(Vec2f(temp_landmark.pose.x,temp_landmark.pose.y)).x();
            temp_obj.pose.y = imagePointToRobotHeadPoint(Vec2f(temp_landmark.pose.x,temp_landmark.pose.y)).y();
            temp_obj.pose.theta = 0.0f;
            temp_obj.confidence = temp_landmark.confidence;
            /* Used in find_lines_landmarks
            type == 1 type_out='X';
            type == 2 type_out='T';
            type == 3 type_out='L';
            type == 4 type_out='N';
            type == 5 type_out='penalty';*/
            switch(temp_landmark.type)
            {
            case 1:
                temp_obj.type = field_model::WorldObject::Type_LineCrossX;
                msg_world_objects.objects.push_back(temp_obj);
                break;
            case 2:
                temp_obj.type = field_model::WorldObject::Type_LineCrossT;
                msg_world_objects.objects.push_back(temp_obj);
                break;
            case 3:
                temp_obj.type = field_model::WorldObject::Type_LineCrossL;
                msg_world_objects.objects.push_back(temp_obj);
                break;
            case 5:
                temp_obj.type = field_model::WorldObject::Type_PenMarker;
                msg_world_objects.objects.push_back(temp_obj);
                break;
            default:
                ROS_ERROR("localization out: Unidentified crossing/landmark, ignoring it.");
                break;
            }
        }
    }

// Callback function to add the goalposts and goal center observed, and goalkeeper if detected
    void visionGoalCallback(const vision::GoalConstPtr& goal_msg)
    {
        //ROS_ERROR("localization out: goalposts seen");
        localization::ObjectsDetected temp_obj;
        temp_obj.confidence = 1.0f;
        temp_obj.type = field_model::WorldObject::Type_GoalPost;
        temp_obj.pose.theta = 0.0f;
        switch(goal_msg->bPillarSeen)
        {
        case 1: //add the left post
            temp_obj.pose.x = imagePointToRobotHeadPoint(Vec2f(goal_msg->iLeftBottomInImageX,goal_msg->iLeftBottomInImageY)).x();
            temp_obj.pose.y = imagePointToRobotHeadPoint(Vec2f(goal_msg->iLeftBottomInImageX,goal_msg->iLeftBottomInImageY)).y();
            msg_world_objects.objects.push_back(temp_obj);
            ROS_ERROR("localization out: only left goalpost seen at [%lf,%lf]",temp_obj.pose.x,temp_obj.pose.y);
            break;
        case 2: //add the right post
            temp_obj.pose.x = imagePointToRobotHeadPoint(Vec2f(goal_msg->iRightBottomInImageX,goal_msg->iRightBottomInImageY)).x();
            temp_obj.pose.y = imagePointToRobotHeadPoint(Vec2f(goal_msg->iRightBottomInImageX,goal_msg->iRightBottomInImageY)).y();
            msg_world_objects.objects.push_back(temp_obj);
            ROS_ERROR("localization out: only right goalpost seen at [%lf,%lf]",temp_obj.pose.x,temp_obj.pose.y);
            break;
        case 3: //add both posts and the center
            temp_obj.pose.x = imagePointToRobotHeadPoint(Vec2f(goal_msg->iLeftBottomInImageX,goal_msg->iLeftBottomInImageY)).x();
            temp_obj.pose.y = imagePointToRobotHeadPoint(Vec2f(goal_msg->iLeftBottomInImageX,goal_msg->iLeftBottomInImageY)).y();
            msg_world_objects.objects.push_back(temp_obj);
            ROS_ERROR("localization out: left goalpost seen at [%lf,%lf]",temp_obj.pose.x,temp_obj.pose.y);

            temp_obj.pose.x = imagePointToRobotHeadPoint(Vec2f(goal_msg->iRightBottomInImageX,goal_msg->iRightBottomInImageY)).x();
            temp_obj.pose.y = imagePointToRobotHeadPoint(Vec2f(goal_msg->iRightBottomInImageX,goal_msg->iRightBottomInImageY)).y();
            msg_world_objects.objects.push_back(temp_obj);
            ROS_ERROR("localization out: right goalpost seen at [%lf,%lf]",temp_obj.pose.x,temp_obj.pose.y);

            temp_obj.type = field_model::WorldObject::Type_Goal;
            temp_obj.pose.x = (imagePointToRobotHeadPoint(Vec2f(goal_msg->iLeftBottomInImageX,goal_msg->iLeftBottomInImageY)).x() +
                    imagePointToRobotHeadPoint(Vec2f(goal_msg->iRightBottomInImageX,goal_msg->iRightBottomInImageY)).x())/2;
            temp_obj.pose.y = (imagePointToRobotHeadPoint(Vec2f(goal_msg->iLeftBottomInImageX,goal_msg->iLeftBottomInImageY)).y() +
                    imagePointToRobotHeadPoint(Vec2f(goal_msg->iRightBottomInImageX,goal_msg->iRightBottomInImageY)).y())/2;
            msg_world_objects.objects.push_back(temp_obj);
            break;
        case 4: //add only the left in case it is unknown
            temp_obj.pose.x = imagePointToRobotHeadPoint(Vec2f(goal_msg->iLeftBottomInImageX,goal_msg->iLeftBottomInImageY)).x();
            temp_obj.pose.y = imagePointToRobotHeadPoint(Vec2f(goal_msg->iLeftBottomInImageX,goal_msg->iLeftBottomInImageY)).y();
            msg_world_objects.objects.push_back(temp_obj);
            ROS_ERROR("localization out: unknown left goalpost seen at [%lf,%lf]",temp_obj.pose.x,temp_obj.pose.y);
            break;
        default:
            ROS_ERROR("localization out: no goalposts seen");
            break;
        }


        // Also write the keeper
        if (goal_msg->bKeeperSeen == 1)
        {
            msg_output_data.bKeeperWasSeen = 1;

            // Get pose at the head of the robot frame, '/robot_head' refers to where the robot stands on with the direction of the head
            tf::Stamped<tf::Pose> keeper_left_head_pose(
                        tf::Transform(tf::createIdentityQuaternion(),
                                      tf::Vector3(imagePointToRobotHeadPoint(
                                                      Vec2f(goal_msg->iLeftLegStartX,goal_msg->iLeftLegStartY)).x(),
                                                  imagePointToRobotHeadPoint(
                                                      Vec2f(goal_msg->iLeftLegStartX,goal_msg->iLeftLegStartY)).y(),0)),
                ros::Time::now(), "/robot_head");
            tf::Stamped<tf::Pose> keeper_right_head_pose(
                        tf::Transform(tf::createIdentityQuaternion(),
                                      tf::Vector3(imagePointToRobotHeadPoint(
                                                      Vec2f(goal_msg->iRightLegEndX,goal_msg->iRightLegEndY)).x(),
                                                  imagePointToRobotHeadPoint(
                                                      Vec2f(goal_msg->iRightLegEndX,goal_msg->iRightLegEndY)).y(),0)),
                ros::Time::now(), "/robot_head");

            try{
                // wait to listen the transform from "/robot_head" to "/field"
                data_input_tf.waitForTransform("/robot_head", "/field",
                                          ros::Time::now(), ros::Duration(0.5));
            }
            catch (tf::TransformException ex){
                ROS_ERROR("loc out: Couldn't find the '/robot_head' to '/field' transform. %s",ex.what());
            }

            // Transform the pose to the field frame '/field'
            try
            {
                data_input_tf.transformPose("/field", keeper_left_head_pose, keeper_left_field_pose);
            }
            catch(tf::TransformException& e)
            {
                fprintf(stderr, "Failed to compute goalkeeper left pose on field, using previous pose\n");
            }
            // Transform the pose to the field frame '/field'
            try
            {
                data_input_tf.transformPose("/field", keeper_right_head_pose, keeper_right_field_pose);
            }
            catch(tf::TransformException& e)
            {
                fprintf(stderr, "Failed to compute goalkeeper right pose on field, using previous pose\n");
            }

            // change the location of the goalkeeper on the field if the confidence of the robot pose on field is bigger than threshold
            if (msg_output_data.robotPoseConfidence > robot_pose_confidence_threshold)
            {
                msg_output_data.keeperLeftStartOnField.x = keeper_left_field_pose.getOrigin().x();
                msg_output_data.keeperLeftStartOnField.y = keeper_left_field_pose.getOrigin().y();
                msg_output_data.keeperRightEndOnField.x = keeper_right_field_pose.getOrigin().x();
                msg_output_data.keeperRightEndOnField.y = keeper_right_field_pose.getOrigin().y();
            }

        };

    }

// Callback function for new obstacles detected
    void visionObstacleCallback(const vision::ObstacleConstPtr& obstacle_msg)
    {
        if (obstacle_msg->bObstacleWasSeen)
        {
            // Clear all the vectors of the obstacles
            msg_output_data.obstacleAngle.clear();
            msg_output_data.obstacleDistance.clear();
            msg_output_data.obstacleLeftEndInImage.clear();
            msg_output_data.obstacleRightEndInImage.clear();
            msg_output_data.obstacleCenterOnField.clear();
            msg_output_data.obstacleRadiusOnField.clear();

//            // for all obstacles detected until the max number allowed
//            for (int i=0; (i< obstacle_msg->iObstacleNumber) && (i<obstacle_max_number);i++)
//            {
//                msg_output_data.bObstacleWasSeen=1;
//                msg_output_data.iObstacleNumber = obstacle_msg->iObstacleNumber;
//                geometry_msgs::Point point_tmp;
//                point_tmp.x = obstacle_msg->iLeftEdgeInImageX[i];
//                point_tmp.y = obstacle_msg->iLeftEdgeInImageY[i];

//                msg_output_data.obstacleLeftEndInImage.push_back(point_tmp);
//                point_tmp.x = obstacle_msg->iRightEdgeInImageX[i];
//                point_tmp.y = obstacle_msg->iRightEdgeInImageY[i];

//                msg_output_data.obstacleRightEndInImage.push_back(point_tmp);

//                Vec3f obstacle_center_from_robot_head = imagePointToRobotHeadPoint(
//                            Vec2f(obstacle_msg->iLeftEdgeInImageX[i]+
//                                  (obstacle_msg->iRightEdgeInImageX[i]-obstacle_msg->iLeftEdgeInImageX[i])/2,
//                                  obstacle_msg->iLeftEdgeInImageY[i]+
//                                  (obstacle_msg->iRightEdgeInImageY[i]-obstacle_msg->iLeftEdgeInImageY[i])/2));

//                Vec3f obstacle_left_from_robot_head = imagePointToRobotHeadPoint(
//                            Vec2f(obstacle_msg->iLeftEdgeInImageX[i],obstacle_msg->iLeftEdgeInImageY[i]));

//                msg_output_data.obstacleDistance.push_back(obstacle_center_from_robot_head.head<2>().norm());

//                msg_output_data.obstacleAngle.push_back(atan2(obstacle_center_from_robot_head.x(),obstacle_center_from_robot_head.y())); //in rad

//                obstacle_field_pose = previous_obstacle_field_pose[i];

//                // change the location of the obstacle on the field if the confidence of the robot pose on field is bigger than threshold
//                if (msg_output_data.robotPoseConfidence > robot_pose_confidence_threshold)
//                {
//                    // Get pose at the head of the robot frame, '/robot_head' refers to where the robot stands on with the direction it's looking at
//                    tf::Stamped<tf::Pose> obstacle_head_pose(tf::Transform(tf::createIdentityQuaternion(),
//                                              tf::Vector3(obstacle_center_from_robot_head.x(),obstacle_center_from_robot_head.y(),0)),
//                        ros::Time::now(), "/robot_head");

//                    try{
//                        // wait to listen the transform from "/robot_head" to "/field"
//                        data_input_tf.waitForTransform("/robot_head", "/field",
//                                                  ros::Time::now(), ros::Duration(0.5));
//                    }
//                    catch (tf::TransformException ex){
//                        ROS_ERROR("loc out: Couldn't find the '/robot_head' to '/field' transform. %s",ex.what());
//                    }

//                    // Transform the pose to the field frame '/field'
//                    try
//                    {
//                        data_input_tf.transformPose("/field", obstacle_head_pose, obstacle_field_pose);
//                    }
//                    catch(tf::TransformException& e)
//                    {
//                        fprintf(stderr, "Failed to compute obstacle pose on field, using previous pose\n");
//                    }
//                    previous_obstacle_field_pose[i] = obstacle_field_pose;
//                            //tf::Pose(obstacle_field_pose.getOrigin().x,
//                              //                                 obstacle_field_pose.getOrigin().y,0);
//                }

//                // Always push it in the vector if the obstacle was seen, but use the previous pose
//                point_tmp.x = obstacle_field_pose.getOrigin().x();
//                point_tmp.y = obstacle_field_pose.getOrigin().y();
//                msg_output_data.obstacleCenterOnField.push_back(point_tmp);

//                if (msg_output_data.obstacleCenterOnField.empty())
//                ROS_ERROR("loc out: obstacle is empty");

//                // calculate the radius based on the distance between the center and the left edge on the "\robot_head" frame
//                msg_output_data.obstacleRadiusOnField.push_back((obstacle_center_from_robot_head-obstacle_left_from_robot_head).norm());

//            }

        }

    }

    // Callback function for new ball detected
        void visionBallCallback(const vision::BallConstPtr& ball_msg)
        {
            if (ball_msg->bBallWasSeen)
            {
                //ROS_ERROR("localization: ball was seen");
                msg_output_data.bBallWasSeen=1;
                msg_output_data.ballCenterInImage.x=ball_msg->iCenterInImageX;
                msg_output_data.ballCenterInImage.y=ball_msg->iCenterInImageY;


                Vec3f ball_center_from_robot_head = imagePointToRobotHeadPoint(
                                                     Vec2f(ball_msg->iCenterInImageX,ball_msg->iCenterInImageY));

                //ROS_ERROR("localization: ball was seen in position [x=%f,y=%f] on robot frame",
                //          ball_center_from_robot_head.x(),ball_center_from_robot_head.y());

                msg_output_data.ballDistance = ball_center_from_robot_head.head<2>().norm();

                msg_output_data.ballAngle = atan2(ball_center_from_robot_head.x(),ball_center_from_robot_head.y()); //in rad

                // Get pose at the base head of the robot frame, '/base_link' refers to where the robot stands on with the direction it's looking at
                tf::Stamped<tf::Pose> ball_base_pose(tf::Transform(tf::createIdentityQuaternion(),
                                          tf::Vector3(ball_msg->ball_range*cos(ball_msg->ball_bearing),
                                                      ball_msg->ball_range*sin(ball_msg->ball_bearing),0)),
                    ros::Time::now(), "/base_link");

                try{
                    // wait to listen the transform from "/base_link" to "/field"
                    data_input_tf.waitForTransform("/base_link", "/field",
                                              ros::Time::now(), ros::Duration(0.5));
                }
                catch (tf::TransformException ex){
                    ROS_ERROR("loc out: Couldn't find the '/base_link' to '/field' transform. %s",ex.what());
                }
                // Transform the pose to the field frame '/field'
                try
                {
                    data_input_tf.transformPose("/field", ball_base_pose, ball_field_pose);
                    //fprintf(stderr,"localization: ball position transformed to field frame\n");
                }
                catch(tf::TransformException& e)
                {
                    fprintf(stderr, "Failed to compute ball pose on field, using previous pose\n");
                }

                // change the location of the ball on the field if the confidence of the robot pose on field is bigger than threshold
//                if (msg_output_data.robotPoseConfidence > robot_pose_confidence_threshold)
//                {
                    //ROS_ERROR("localization: ball position changed");
                    msg_output_data.ballCenterOnField.x = ball_field_pose.getOrigin().x();
                    msg_output_data.ballCenterOnField.y = ball_field_pose.getOrigin().y();
               // }

            }

        }


    void localizationParticleSetCallback(const localization::ParticleSetConstPtr& particle_set_msg)
    {
    //  ROS_INFO("", );
    }

    // Callback function to handle compass readings
    void compassCallback(const sensor_msgs::ImuConstPtr& compass)
    {
        // If we trust the orientation given by the compass
       //  msg_output_data.robotPose.theta = tf::getYaw(compass->orientation) -1.22; //add 70d for difference
        //ROS_ERROR("loc out: received new compass reading %f",tf::getYaw(compass->orientation));
    }

    // Callback function for receiving the current mean pose
    void localizationMeanPoseCallback(const localization::MeanPoseConfStampedConstPtr& mean_pose_msg)
    {
//        ROS_ERROR("out: Mean pose (%f,%f,%f) with confidence %f",mean_pose_msg->robotPose.x, mean_pose_msg->robotPose.y,
//                  mean_pose_msg->robotPose.theta, mean_pose_msg->robotPoseConfidence);

        msg_output_data.header = mean_pose_msg->header;

        tf::StampedTransform head_to_base;
        head_to_base.frame_id_ = "/robot_head";
        head_to_base.child_frame_id_ = "/robot_base";
        head_to_base.setIdentity();
        head_to_base.stamp_ = ros::Time::now();
        try{
            // wait to listen the transform from "/robot_head" to "/robot_base"
            data_input_tf.waitForTransform("/robot_head", "/robot_base",
                                      head_to_base.stamp_, ros::Duration(0.5));
            data_input_tf.lookupTransform("/robot_head", "/robot_base",
                                      head_to_base.stamp_, head_to_base);
        }
        catch (tf::TransformException ex){
            ROS_ERROR("loc out: Couldn't find the '/robot_head' to '/robot_base' transform. %s",ex.what());
        }

        // Transform the head pose to the robot base frame '/robot_base'.
        // The theta is always the output of the gyro sensor.
        // Use the pose of the robot body on field after reversing the rotation of the head
        msg_output_data.robotPose.x = mean_pose_msg->robotPose.x;
        msg_output_data.robotPose.y = mean_pose_msg->robotPose.y;
        // if we trust the orientation given by the localization
        msg_output_data.robotPose.theta = mean_pose_msg->robotPose.theta;

        ROS_ERROR("Using rotation %f with initial rotation %f\n",
                  tf::getYaw(head_to_base.getRotation()),msg_output_data.robotPose.theta);
        msg_output_data.robotPose.theta = msg_output_data.robotPose.theta
                + tf::getYaw(head_to_base.getRotation());

        msg_output_data.robotPoseConfidence = mean_pose_msg->robotPoseConfidence;

        // Publish detected world objects for use in the particle filter localization

        pub_localization_world_objects.publish(msg_world_objects);

        // Clear everything after publishing if the objects are more than the threshold.
        // Otherwise, reuse them.
        if (msg_world_objects.objects.size()> world_objects_max_number)
        {
            msg_world_objects.lines.clear();
            msg_world_objects.objects.clear();
            msg_world_objects.obstacles.clear();
        }
    }

};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "localization_data_input_output");
    LocalizationDataInputOutput ldio;

    ROS_INFO("started data_input_output");

    ros::Rate rate(5); // at least as fast as gait and localization update,
                             // TODO: check if there is need to make it faster to catch up with vision

    ros::NodeHandle nh;

    int i=0;
    while(ros::ok())
    {

        // Publish output data for use by behavior
        ldio.pub_localization_output_data.publish(ldio.msg_output_data);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
