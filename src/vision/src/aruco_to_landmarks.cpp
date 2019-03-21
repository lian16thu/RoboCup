#include <assert.h>
#include <sys/time.h>
#include <unistd.h>
#include <signal.h>

#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/Marker.h>

#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv/cv.h>
#include <vector>

#include <sstream>
#include "vision/Markers.h"
#include "vision/Landmarks.h"
#include "vision/Goalpost.h"

#include "sensor_msgs/CameraInfo.h"


#include <list>
#include <string>

using namespace std;
using namespace cv;


class ArucoToLandmarks {

    ros::NodeHandle atl_nh;

public:
    ros::Subscriber markersSub;
    ros::Subscriber cameraInfoSub;
    ros::Publisher landmarksPub;
    ros::Publisher goalpostPub;

    // void markersCallback(const vision::Markers::ConstPtr &msg);

    vision::Landmarks msg_landmarks;
    vision::Goalpost msg_goalpost;


    ArucoToLandmarks()
    {
        markersSub = atl_nh.subscribe("vision/marker", 1,&ArucoToLandmarks::markersCallback, this);
        cameraInfoSub = atl_nh.subscribe("/zed/left/camera_info", 1,&ArucoToLandmarks::cameraInfoCallback, this);
        landmarksPub = atl_nh.advertise<vision::Landmarks>("vision/landmarks", 1);
        goalpostPub = atl_nh.advertise<vision::Goalpost>("vision/goalpost", 1);


        msg_landmarks.landmark_number = 0;
        msg_goalpost.goalpost_detected = false;
        msg_goalpost.goalpost_number = 0;

        ROS_INFO("Aruco to Landmarks initialized");
    }


    ~ArucoToLandmarks()
    {

    }

    void cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg)
    {

    }

    void markersCallback(const vision::Markers::ConstPtr& msg)
    {

        if (msg->markers_detected)
        {

            for (int i=0; i<msg->markers_number; i++) {


                switch(msg->marker_id[i])
                {
                // 1 is X, 2 is T, 3 is L, 4 is unknown, 5 is penaltypoint
                case 1: case 2:  // X
                    msg_landmarks.landmark_type[msg_landmarks.landmark_number] = 1;
                    msg_landmarks.landmark_range[msg_landmarks.landmark_number] = sqrt(pow(msg->marker_transvec_x[i],2)+pow(msg->marker_transvec_z[i],2));
                    msg_landmarks.landmark_bearing[msg_landmarks.landmark_number] = atan2(msg->marker_transvec_x[i],msg->marker_transvec_z[i]);
                    msg_landmarks.landmark_confidence[msg_landmarks.landmark_number] = 1;
                    ROS_INFO("Marker:%d detected at x:%f, z:%f, range:%f and bearing:%f, adding an X",
                             msg->marker_id[i],msg->marker_transvec_x[i],msg->marker_transvec_z[i],
                             msg_landmarks.landmark_range[msg_landmarks.landmark_number],msg_landmarks.landmark_bearing[msg_landmarks.landmark_number]*180/M_PI);
                    msg_landmarks.landmark_number++;
                    break;
                case 3: case 4:  // Penalty point
                    msg_landmarks.landmark_type[msg_landmarks.landmark_number] = 5;
                    msg_landmarks.landmark_range[msg_landmarks.landmark_number] = sqrt(pow(msg->marker_transvec_x[i],2)+pow(msg->marker_transvec_z[i],2));
                    msg_landmarks.landmark_bearing[msg_landmarks.landmark_number] = atan2(msg->marker_transvec_x[i],msg->marker_transvec_z[i]);
                    msg_landmarks.landmark_confidence[msg_landmarks.landmark_number] = 1;
                    ROS_INFO("Marker:%d detected at x:%f, z:%f, range:%f and bearing:%f, adding a penalty point",
                             msg->marker_id[i],msg->marker_transvec_x[i],msg->marker_transvec_z[i],
                             msg_landmarks.landmark_range[msg_landmarks.landmark_number],msg_landmarks.landmark_bearing[msg_landmarks.landmark_number]*180/M_PI);
                    msg_landmarks.landmark_number++;
                    break;
                case 5: case 6: case 7: case 8: case 9: case 10:  // T
                    msg_landmarks.landmark_type[msg_landmarks.landmark_number] = 2;
                    msg_landmarks.landmark_range[msg_landmarks.landmark_number] = sqrt(pow(msg->marker_transvec_x[i],2)+pow(msg->marker_transvec_z[i],2));
                    msg_landmarks.landmark_bearing[msg_landmarks.landmark_number] = atan2(msg->marker_transvec_x[i],msg->marker_transvec_z[i]);
                    msg_landmarks.landmark_confidence[msg_landmarks.landmark_number] = 1;
                    ROS_INFO("Marker:%d detected at x:%f, z:%f, range:%f and bearing:%f, adding a T",
                             msg->marker_id[i],msg->marker_transvec_x[i],msg->marker_transvec_z[i],
                             msg_landmarks.landmark_range[msg_landmarks.landmark_number],msg_landmarks.landmark_bearing[msg_landmarks.landmark_number]*180/M_PI);
                    msg_landmarks.landmark_number++;
                    break;
                case 11: case 12: case 13: case 14: case 15: case 16: case 17: case 18:  // L
                    msg_landmarks.landmark_type[msg_landmarks.landmark_number] = 3;
                    msg_landmarks.landmark_range[msg_landmarks.landmark_number] = sqrt(pow(msg->marker_transvec_x[i],2)+pow(msg->marker_transvec_z[i],2));
                    msg_landmarks.landmark_bearing[msg_landmarks.landmark_number] = atan2(msg->marker_transvec_x[i],msg->marker_transvec_z[i]);
                    msg_landmarks.landmark_confidence[msg_landmarks.landmark_number] = 1;
                    ROS_INFO("Marker:%d detected at x:%f, z:%f, range:%f and bearing:%f, adding an L",
                             msg->marker_id[i],msg->marker_transvec_x[i],msg->marker_transvec_z[i],
                             msg_landmarks.landmark_range[msg_landmarks.landmark_number],msg_landmarks.landmark_bearing[msg_landmarks.landmark_number]*180/M_PI);
                    msg_landmarks.landmark_number++;
                    break;
                case 19: case 22:  // Left Goal post
                    msg_goalpost.goalpost_detected = true;
                    msg_goalpost.goalpost_number++;
                    msg_goalpost.goalpost_left_range = sqrt(pow(msg->marker_transvec_x[i],2)+pow(msg->marker_transvec_z[i],2));
                    msg_goalpost.goalpost_left_bearing = atan2(msg->marker_transvec_x[i],msg->marker_transvec_z[i]);

                    ROS_INFO("Marker:%d detected at x:%f, z:%f, range:%f and bearing:%f, adding left goalpost",
                             msg->marker_id[i],msg->marker_transvec_x[i],msg->marker_transvec_z[i],
                             msg_goalpost.goalpost_left_range,msg_goalpost.goalpost_left_bearing*180/M_PI);
                    break;
                case 20: case 21:  case 23: // Right Goal post  //23 was added instead of 21
                    msg_goalpost.goalpost_detected = true;
                    msg_goalpost.goalpost_number++;
                    msg_goalpost.goalpost_right_range = sqrt(pow(msg->marker_transvec_x[i],2)+pow(msg->marker_transvec_z[i],2));
                    msg_goalpost.goalpost_right_bearing = atan2(msg->marker_transvec_x[i],msg->marker_transvec_z[i]);
                    // If only right detected, add it to the position of the left as well, since only left is read in case of 1
                    if (msg_goalpost.goalpost_number < 2)
                    {
                        msg_goalpost.goalpost_left_range = sqrt(pow(msg->marker_transvec_x[i],2)+pow(msg->marker_transvec_z[i],2));
                        msg_goalpost.goalpost_left_bearing = atan2(msg->marker_transvec_x[i],msg->marker_transvec_z[i]);
                    }
                    ROS_INFO("Marker:%d detected at x:%f, z:%f, range:%f and bearing:%f, adding right goalpost",
                             msg->marker_id[i],msg->marker_transvec_x[i],msg->marker_transvec_z[i],
                             msg_goalpost.goalpost_right_range,msg_goalpost.goalpost_right_bearing*180/M_PI);
                    break;
                default:
                    ROS_ERROR("localization io: Unidentified marker, ignoring it.");
                    break;
                }


            }
        }

    }

};


int main(int argc, char ** argv) {
    ros::init(argc, argv, "aruco_to_landmarks");
    ArucoToLandmarks atl;

    ros::NodeHandle nh;

    ros::Rate r(20);
    while (ros::ok()) {

        atl.msg_landmarks.header.stamp = ros::Time::now();
        atl.msg_goalpost.header.stamp = ros::Time::now();

        if (atl.msg_landmarks.landmark_number>0)
            atl.landmarksPub.publish(atl.msg_landmarks);

        atl.msg_landmarks.landmark_number = 0;

        if (atl.msg_goalpost.goalpost_detected)
            atl.goalpostPub.publish(atl.msg_goalpost);

        atl.msg_goalpost.goalpost_detected = false;
        atl.msg_goalpost.goalpost_number = 0;

        ros::spinOnce();
        r.sleep();
    }

    return 0;
}
