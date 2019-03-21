// Code for collecting all the objects detected by Computer Vision module and then providing them to the Localization module in
// the correct format and for collecting the particle set computed in each iteration of the 'particle_filter_localization.cpp' code
// and constructing the final message to be published
// dependencies: 'vision/Landmarks.h', 'vision/corner.h','vision/goal.h','vision/opponents.h' header files for cv objects
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

#include "vision/Lines.h"
#include "vision/Landmarks.h"
#include "vision/Ball.h"
#include "vision/Goalpost.h"
#include "vision/Opponents.h"
#include "particle_filter_localization.h"
#include "localization/OutputData.h"
#include "localization/ParticleSet.h"
#include "localization/MeanPoseConfStamped.h"


#include <boost/foreach.hpp>



class LocalizationDataInputOutput
{
    ros::NodeHandle ldio_nh;

    public:

    ros::Subscriber sub_vision_lines;
    ros::Subscriber sub_vision_landmarks;
    ros::Subscriber sub_vision_goal;
    ros::Subscriber sub_vision_ball;
    ros::Subscriber sub_vision_opponents;
    ros::Subscriber sub_localization_particle_set;
    ros::Subscriber sub_localization_mean_pose;


public:
    ros::Publisher pub_localization_world_objects;
    ros::Publisher pub_localization_output_data;

    localization::WorldObjects msg_world_objects;
    localization::OutputData msg_output_data;

    tf::TransformListener data_input_tf;
    tf::Stamped<tf::Pose> opponent_field_pose;
//    tf::Stamped<tf::Pose> previous_opponent_field_pose[5];
//    tf::Stamped<tf::Pose> obstacle_field_pose;
//    tf::Stamped<tf::Pose> previous_obstacle_field_pose[5];
    tf::Stamped<tf::Pose> ball_field_pose;


    int robot_pose_confidence_threshold; // threshold above which we trust the new mean robot pose on field
    // to update the positions of other objects on the field

    int world_objects_max_number; //threshold for max items of world objects to be examined once by the particle filter
    // if more than that, the array gets cleared

    float camera_height;



    LocalizationDataInputOutput()
    {
        sub_vision_lines = ldio_nh.subscribe("vision/lines",10,&LocalizationDataInputOutput::visionLinesCallback,this);
        sub_vision_landmarks = ldio_nh.subscribe("vision/landmarks",10,&LocalizationDataInputOutput::visionLandmarksCallback,this);
        sub_vision_goal = ldio_nh.subscribe("vision/goalpost",10,&LocalizationDataInputOutput::visionGoalCallback,this);
        sub_vision_ball = ldio_nh.subscribe("vision/ball",10,&LocalizationDataInputOutput::visionBallCallback,this);
        sub_localization_particle_set = ldio_nh.subscribe("localization/particle_set",10,&LocalizationDataInputOutput::localizationParticleSetCallback,this);
        sub_localization_mean_pose = ldio_nh.subscribe("localization/mean_pose_conf_stamped",10,&LocalizationDataInputOutput::localizationMeanPoseCallback,this);
        pub_localization_world_objects = ldio_nh.advertise<localization::WorldObjects>("localization/world_objects_detected",10);
        pub_localization_output_data = ldio_nh.advertise<localization::OutputData>("localization/output_data",10);


        //Initialize world_objects and output_data
        msg_output_data.bBallWasSeen = 0;
        ball_field_pose = tf::Stamped<tf::Pose>(
                    tf::Transform(tf::createIdentityQuaternion(), tf::Vector3(1.5,0.2,0)),ros::Time::now(), "/field");;
//        msg_output_data.bObstacleWasSeen=0;
        msg_output_data.bOpponentWasSeen = 0;
        opponent_field_pose = tf::Stamped<tf::Pose>(
                    tf::Transform(tf::createIdentityQuaternion(), tf::Vector3(-0.5,-0.2,0)),ros::Time::now(), "/field");
        msg_world_objects.lines.clear();
        msg_world_objects.objects.clear();
        msg_world_objects.goalposts.clear();
//        msg_world_objects.obstacles.clear();
        robot_pose_confidence_threshold = 0.4;  //lin jie xin ren ?
        world_objects_max_number = 25;

//        previous_opponent_field_pose = opponent_field_pose;

        camera_height = 1.32;

    }

    ~LocalizationDataInputOutput()
    {

    }


    void visionLinesCallback(const vision::LinesConstPtr& lines_msg) // need to change
    {

        for (int i=0;i<lines_msg->lines_number;i++)
        {

            // OK for now, but we should use the coordinates of lines on the robot's system?
            localization::LinesDetected temp_line;
            temp_line.x1 = lines_msg->line_start_x[i];
            temp_line.y1 = lines_msg->line_start_y[i];
            temp_line.x2 = lines_msg->line_end_x[i];
            temp_line.y2 = lines_msg->line_end_y[i];
            msg_world_objects.lines.push_back(temp_line);
            ROS_ERROR("localization io: Line detected at x1,y1:%f,%f, x2,y2:%f,%f.",
                      temp_line.x1,temp_line.y1,temp_line.x2,temp_line.y2);
        }
    }


    void visionLandmarksCallback(const vision::LandmarksConstPtr& landmarks_msg) // need to change
    {

        for (int i=0;i<landmarks_msg->landmark_number;i++)
        {

            localization::ObjectsDetected temp_obj;
            temp_obj.pose.x = landmarks_msg->landmark_range[i] * cos(fabs(landmarks_msg->landmark_bearing[i]));
            temp_obj.pose.y = landmarks_msg->landmark_range[i] * sin(landmarks_msg->landmark_bearing[i]);
            temp_obj.pose.theta = 0.0f;
            temp_obj.confidence = landmarks_msg->landmark_confidence[i];
            /* Used in find_landmarks
            type == 1 type_out='X';
            type == 2 type_out='T';
            type == 3 type_out='L';
            type == 4 type_out='N';
            type == 5 type_out='penalty';*/
            switch(landmarks_msg->landmark_type[i])
            {
            case 1:
                temp_obj.type = field_model::WorldObject::Type_LineCrossX;
                msg_world_objects.objects.push_back(temp_obj);
                ROS_ERROR("localization io: X detected at x,y:%f,%f.",temp_obj.pose.x,temp_obj.pose.y);
                break;
            case 2:
                temp_obj.type = field_model::WorldObject::Type_LineCrossT;
                msg_world_objects.objects.push_back(temp_obj);
                ROS_ERROR("localization io: T detected at x,y:%f,%f.",temp_obj.pose.x,temp_obj.pose.y);
                break;
            case 3:
                temp_obj.type = field_model::WorldObject::Type_LineCrossL;
                msg_world_objects.objects.push_back(temp_obj);
                ROS_ERROR("localization io: L detected at x,y:%f,%f.",temp_obj.pose.x,temp_obj.pose.y);
                break;
            case 5:
                temp_obj.type = field_model::WorldObject::Type_PenMarker;
                msg_world_objects.objects.push_back(temp_obj);
                ROS_ERROR("localization io: Pen detected at x,y:%f,%f.",temp_obj.pose.x,temp_obj.pose.y);
                break;
            default:
                ROS_ERROR("localization io: Unidentified crossing/landmark, ignoring it.");
                break;
            }
        }

        // Publish detected world objects for use in the particle filter localization

        msg_world_objects.header.stamp = ros::Time::now();
        pub_localization_world_objects.publish(msg_world_objects);

        ROS_ERROR("loc_IO:Published %d objects",msg_world_objects.objects.size());

        // Clear everything after publishing if the objects are more than the threshold.                  IS THIS CORRECT????????
        // Otherwise, reuse them.
//        if (msg_world_objects.objects.size()> world_objects_max_number)
//        {
            msg_world_objects.lines.clear();
            msg_world_objects.objects.clear();
//            msg_world_objects.obstacles.clear();
//        }
    }


    // Callback function to add the goalposts and goal center observed, and goalkeeper if detected

    void visionGoalCallback(const vision::GoalpostConstPtr& goal_msg)
    {
        if (goal_msg->goalpost_detected)
        {
            ROS_ERROR("localization io: goalposts seen");
            localization::GoalpostsDetected temp_obj;
            temp_obj.confidence = 1.0f;
            temp_obj.type = field_model::WorldObject::Type_GoalPost;
            temp_obj.pose.theta = 0.0f;
//            switch(goal_msg->goalpost_number)
//            {
//            case 1: //add the left post
//                temp_obj.pose.x = goal_msg->goalpost_left_range * cos(fabs(goal_msg->goalpost_left_bearing));
//                temp_obj.pose.y = goal_msg->goalpost_left_range * sin(goal_msg->goalpost_left_bearing);
//                msg_world_objects.goalposts.push_back(temp_obj);
//                ROS_ERROR("localization io: only left goalpost seen at [%lf,%lf], range:%f, bearing:%f",temp_obj.pose.x,temp_obj.pose.y,goal_msg->goalpost_left_range,goal_msg->goalpost_left_bearing);
//                break;
//            case 2: //add both posts and the center
//                temp_obj.pose.x = goal_msg->goalpost_left_range * cos(fabs(goal_msg->goalpost_left_bearing));
//                temp_obj.pose.y = goal_msg->goalpost_left_range * sin(goal_msg->goalpost_left_bearing);
//                msg_world_objects.goalposts.push_back(temp_obj);
//                ROS_ERROR("localization io: left goalpost seen at [%lf,%lf], range:%f, bearing:%f",temp_obj.pose.x,temp_obj.pose.y,goal_msg->goalpost_left_range,goal_msg->goalpost_left_bearing);

//                temp_obj.pose.x = goal_msg->goalpost_right_range * cos(fabs(goal_msg->goalpost_right_bearing));
//                temp_obj.pose.y = goal_msg->goalpost_right_range * sin(goal_msg->goalpost_right_bearing);
//                msg_world_objects.goalposts.push_back(temp_obj);
//                ROS_ERROR("localization io: right goalpost seen at [%lf,%lf], range:%f, bearing:%f",temp_obj.pose.x,temp_obj.pose.y,goal_msg->goalpost_right_range,goal_msg->goalpost_right_bearing);

                // The goal center in the middle of the goalposts
                temp_obj.type = field_model::WorldObject::Type_Goal;
                temp_obj.pose.x = goal_msg->goalpost_left_range * cos(goal_msg->goalpost_left_bearing);
                temp_obj.pose.y = goal_msg->goalpost_left_range * sin(goal_msg->goalpost_left_bearing);

                msg_world_objects.goalposts.push_back(temp_obj);
//                break;
//            default:
//                //ROS_ERROR("localization io: wrong goalpost number");
//                break;
//            }
            msg_world_objects.header.stamp = ros::Time::now();
            pub_localization_world_objects.publish(msg_world_objects);
            msg_world_objects.goalposts.clear();
        }
        else
        {
            //ROS_ERROR("localization io: Goalpost msg, but no goalposts seen");
            ;
        }
    }


//    // Callback function for opponent and other obstacles detected

//    void visionOpponentCallback(const vision::ObstaclesConstPtr& obstacle_msg)
//    {

//        if (obstacle_msg->opponent_detected)
//        {
//            msg_output_data.bOpponentWasSeen = 1;

//            geometry_msgs::Point point_tmp;

//            point_tmp.x = 0;
//            point_tmp.y = 0;


//            // Data on image not given for now
//            msg_output_data.opponentLeftEndInImage = point_tmp;
//            msg_output_data.opponentRightEndInImage = point_tmp;

//            msg_output_data.opponentDistance = obstacle_msg->opponent_range;
//            msg_output_data.opponentAngle = obstacle_msg->opponent_bearing;

//            point_tmp.x = obstacle_msg->opponent_range * cos(fabs(obstacle_msg->opponent_bearing));
//            point_tmp.y = obstacle_msg->opponent_range * sin(obstacle_msg->opponent_bearing);

//            opponent_field_pose = previous_opponent_field_pose;

//            // change the location of the obstacle on the field if the confidence of the robot pose on field is bigger than threshold
//            if (msg_output_data.robotPoseConfidence > robot_pose_confidence_threshold)
//            {
//                // Get pose at the head of the robot frame, '/robot_head' refers to where the robot stands on with the direction it's looking at
//                tf::Stamped<tf::Pose> obstacle_head_pose(tf::Transform(tf::createIdentityQuaternion(),
//                                                                       tf::Vector3(point_tmp.x,point_tmp.y,0)),
//                                                         ros::Time::now(), "/robot_head");

//                try{
//                    // wait to listen the transform from "/robot_head" to "/field"
//                    data_input_tf.waitForTransform("/robot_head", "/field",
//                                                   ros::Time::now(), ros::Duration(0.5));
//                }
//                catch (tf::TransformException ex){
//                    ROS_ERROR("loc out: Couldn't find the '/robot_head' to '/field' transform. %s",ex.what());
//                }

//                // Transform the pose to the field frame '/field'
//                try
//                {
//                    data_input_tf.transformPose("/field", obstacle_head_pose, obstacle_field_pose);
//                }
//                catch(tf::TransformException& e)
//                {
//                    fprintf(stderr, "Failed to compute opponent pose on field, using previous pose\n");
//                }
//                previous_opponent_field_pose = opponent_field_pose;
//            }

//            // Always add the opponent if seen, but use the previous pose if the confidence isn't high
//            point_tmp.x = opponent_field_pose.getOrigin().x();
//            point_tmp.y = opponent_field_pose.getOrigin().y();
//            msg_output_data.opponentCenterOnField = point_tmp;


//            // Radius 0.5 for now
//            msg_output_data.opponentRadiusOnField = 0.5;
//        }
//        else
//        {
//            msg_output_data.bOpponentWasSeen = 0;
//        }

//    }

    // Callback function for new ball detected
    void visionBallCallback(const vision::BallConstPtr& ball_msg)
    {

        if (ball_msg->ball_detected)
        {
            ROS_ERROR("localization io: ball was seen");
            msg_output_data.bBallWasSeen=1;
            msg_output_data.ballCenterInImage.x=0;
            msg_output_data.ballCenterInImage.y=0;

            msg_output_data.ballDistance = ball_msg->ball_range;

            msg_output_data.ballAngle = ball_msg->ball_bearing; //in rad

            // Get pose at the head of the robot frame, '/robot_head' refers to where the robot stands on with the direction it's looking at
            tf::Stamped<tf::Pose> ball_head_pose(tf::Transform(tf::createIdentityQuaternion(),
                                                               tf::Vector3(msg_output_data.ballDistance*cos(msg_output_data.ballAngle),msg_output_data.ballDistance*sin(msg_output_data.ballAngle),0)),
                                                 ros::Time::now(), "/robot_head");

            try{
                // wait to listen the transform from "/robot_head" to "/field"
                data_input_tf.waitForTransform("/robot_head", "/field",
                                               ros::Time::now(), ros::Duration(0.5));
            }
            catch (tf::TransformException ex){
                //ROS_ERROR("localization io: Couldn't find the '/robot_head' to '/field' transform. %s",ex.what());
            }
            // Transform the pose to the field frame '/field'
            try
            {
                data_input_tf.transformPose("/field", ball_head_pose, ball_field_pose);
                //fprintf(stderr,"localization io: ball position transformed to field frame\n");
            }
            catch(tf::TransformException& e)
            {
                //fprintf(stderr, "localization io: Failed to compute ball pose on field, using previous pose\n");
            }

            // change the location of the ball on the field if the confidence of the robot pose on field is bigger than threshold
            if (msg_output_data.robotPoseConfidence > robot_pose_confidence_threshold)
            {
                //ROS_ERROR("localization io: ball position changed");
                //msg_output_data.ballCenterOnField.x = ball_field_pose.getOrigin().x();
                //msg_output_data.ballCenterOnField.y = ball_field_pose.getOrigin().y();

                msg_output_data.ballCenterOnField.x = msg_output_data.robotPose.x
                        + ball_msg->ball_range * cos(msg_output_data.robotPose.theta+ball_msg->ball_bearing);
                msg_output_data.ballCenterOnField.y = msg_output_data.robotPose.y
                        + ball_msg->ball_range * sin(msg_output_data.robotPose.theta+ball_msg->ball_bearing);

            }

        }
        else
        {
            msg_output_data.bBallWasSeen=0;
            ROS_ERROR("localization io: ball message, but ball wasn't seen");
        }

    }


    void localizationParticleSetCallback(const localization::ParticleSetConstPtr& particle_set_msg)
    {
        //  ROS_INFO("", );
    }


    // Callback function for receiving the current mean pose
    void localizationMeanPoseCallback(const localization::MeanPoseConfStampedConstPtr& mean_pose_msg)
    {
        //ROS_ERROR("out: Mean pose (%f,%f,%f) with confidence %f",mean_pose_msg->robotPose.x, mean_pose_msg->robotPose.y, mean_pose_msg->robotPose.theta, mean_pose_msg->robotPoseConfidence);

        msg_output_data.header = mean_pose_msg->header;

        tf::StampedTransform head_to_base;
        head_to_base.frame_id_ = "/robot_head";
        head_to_base.child_frame_id_ = "/base_link";
        head_to_base.setIdentity();
        head_to_base.stamp_ = ros::Time::now();
        try{
            // wait to listen the transform from "/robot_head" to "/base_link"
            data_input_tf.waitForTransform("/robot_head", "/base_link",
                                           head_to_base.stamp_, ros::Duration(0.5));
            data_input_tf.lookupTransform("/robot_head", "/base_link",
                                          head_to_base.stamp_, head_to_base);
        }
        catch (tf::TransformException ex){
            //ROS_ERROR("localization io: Couldn't find the '/robot_head' to '/base_link' transform. %s",ex.what());
        }

        // Transform the head pose to the robot base frame '/base_link'.
        // Use the pose of the robot body on field after reversing the rotation of the head and the dislocation of the waist
        msg_output_data.robotHeadPose.x = mean_pose_msg->robotPose.x;
        msg_output_data.robotHeadPose.y = mean_pose_msg->robotPose.y;
        // if we trust the orientation given by the localization
        msg_output_data.robotHeadPose.theta = mean_pose_msg->robotPose.theta + tf::getYaw(head_to_base.getRotation());



        ROS_INFO("Loc IO: Localization output: head x,y,theta: %f,%f,%f and dislocation x,y,theta: %f,%f,%f",
                  msg_output_data.robotHeadPose.x, msg_output_data.robotHeadPose.y, msg_output_data.robotHeadPose.theta,
                  head_to_base.getOrigin().x(),head_to_base.getOrigin().y(),tf::getYaw(head_to_base.getRotation()));

        //Change this because now all distances are to the front, leave only head yaw

        msg_output_data.robotPose.x = mean_pose_msg->robotPose.x;
        msg_output_data.robotPose.y = mean_pose_msg->robotPose.y;
        msg_output_data.robotPose.theta = mean_pose_msg->robotPose.theta;

        msg_output_data.robotPoseConfidence = mean_pose_msg->robotPoseConfidence;

        // Publish output data for use by behavior
        msg_output_data.header.stamp = ros::Time::now();
        pub_localization_output_data.publish(msg_output_data);

    }

};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "localization_data_input_output");
    LocalizationDataInputOutput ldio;

    //ROS_INFO("localization io: started data_input_output");

    ros::Rate rate(10); // at least as fast as gait and localization update,
    // TODO: check if there is need to make it faster to catch up with vision !

    ros::NodeHandle nh;

    int i=0;
    while(ros::ok())
    {

        ros::spinOnce();
        //rate.sleep(); // check if delay is needed
    }

    return 0;
}
