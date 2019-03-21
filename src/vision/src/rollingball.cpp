#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/opencv.hpp>


#include <cv.hpp>
#include "cv_bridge/cv_bridge.h"
#include <image_transport/image_transport.h>

#include <sensor_msgs/image_encodings.h>

//#include "stereo_process/Ball.h"
#include "boost/array.hpp"
#include <head_motion/head_pose.h>
#include <decision/SerialReceived.h>

#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/BoundingBox.h>

#include <vision/Ball.h>

#include <numeric>
#include <fstream>

#define MIN_CLUSTER_SIZE 15
#define CAMERA_HEIGHT 1.27

//camera parameters
double fx = 677.178;
double PI = 3.1415926;

using namespace cv;
using namespace std;


bool first_frame;
vector<Point2f> center_vec;
//vector<double> time_vec;
//vector<double> time_interval;
//added by luohy
double time_intervalsum = 0;
double time_interval2sum = 0;
Point2f time_intervalpsum = Point2f(0.0,0.0);
Point2f psum = Point2f(0.0,0.0);
double lambda = 0.2;
//end

bool moving_flag;
double moving_indicator = 10.0;
bool head_down;
double t2kick = 18.0;

class BallDetector
{
   ros::NodeHandle nh;

   image_transport::ImageTransport it;

   ros::Subscriber sub_detection_boxes;
   ros::Publisher ball_msg_pub;

   ros::Subscriber sub_found_object;
   ros::Subscriber head_pose,serial_listener;

   //msg
   vision::Ball ball_msg;
   darknet_ros_msgs::BoundingBoxes BBox_array;
   darknet_ros_msgs::BoundingBox BBox;
   cv::Point2i BBox_center;
   bool found_ball;

public:

   double head_pitch, head_yaw;
   double pc_timestamp;

   sensor_msgs::CameraInfo camera_info;

   BallDetector():it(nh)
   {
       sub_detection_boxes = nh.subscribe("/darknet_ros/bounding_boxes", 1, &BallDetector::detectionCb, this);
       //sub_image_info = pcl_nh.subscribe("/zed/rgb/camera_info", 1, &PointCloudProcess::imageCb, this); // check this topic's name

       head_pose = nh.subscribe ("/decision/head_command", 1, &BallDetector::headCb,this);

       ball_msg_pub = nh.advertise<vision::Ball>("/vision/ball", 1);

       ROS_INFO("Initiated for rolling ball");

       head_pitch = 0.0;//60.0/180*M_PI;
       head_yaw = 0.0;//-20.0/180*M_PI;

       pc_timestamp = ros::Time::now().toSec();

       //camera_info
       camera_info.height = 720;
       camera_info.width = 1280;
       camera_info.distortion_model = "plumb_bob";
       vector<double> a;
       a.push_back(0.0);a.push_back(0.0);a.push_back(0.0);a.push_back(0.0);a.push_back(0.0);
       camera_info.D = a;

       boost::array<double, 9ul> b;
       b.at(0) = 677.0040893554688; b.at(1) = 0.0; b.at(2) = 675.9085693359375;
       b.at(3) = 0.0; b.at(4) = 677.0040893554688; b.at(5) = 373.6982421875;
       b.at(6) = 0.0; b.at(7) = 0.0; b.at(8) = 1.0;
       camera_info.K = b;

       b.at(0) = 1.0; b.at(1) = 0.0; b.at(2) = 0.0;
       b.at(3) = 0.0; b.at(4) = 1.0; b.at(5) = 0.0;
       b.at(6) = 0.0; b.at(7) = 0.0; b.at(8) = 1.0;
       camera_info.R = b;

       boost::array<double, 12ul> c;
       c.at(0) = 677.0040893554688; c.at(1) = 0.0; c.at(2) = 675.9085693359375; c.at(3) = 0.0;
       c.at(4) = 0.0; c.at(5) = 677.0040893554688; c.at(6) = 373.6982421875; c.at(7) = 0.0;
       c.at(8) = 0.0; c.at(9) = 0.0; c.at(10) = 1.0; c.at(11) = 0.0;
       camera_info.P = c;

       camera_info.binning_x = 0;
       camera_info.binning_y = 0;
       camera_info.roi.x_offset = 0;
       camera_info.roi.y_offset = 0;
       camera_info.roi.height= 0;
       camera_info.roi.width= 0;
       camera_info.roi.do_rectify= false;

       //Initialize ball msg
       ball_msg.header.stamp = ros::Time::now();
       ball_msg.ball_detected = false;
       ball_msg.ball_range = 0.0;
       ball_msg.ball_bearing = 0.0;
       ball_msg.kick_time= 100.0;

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

       double time_n;

       pc_timestamp = ros::Time::now().toSec();

       //ROS_INFO("Time stamp is %f", pc_timestamp);

       //time_predict.push_back(pc_timestamp);

       for(int i=0; i< msg->bounding_boxes.size(); i++)
       {
           const darknet_ros_msgs::BoundingBox &BBox = msg->bounding_boxes[i];

           //found ball, then calculate the bearing and range
           if(BBox.Class == "Ball")
           {
               found_ball = true;

               BBox_center.x = (BBox.xmax - BBox.xmin) / 2 + BBox.xmin;
               BBox_center.y = (BBox.ymax - BBox.ymin) / 2 + BBox.ymin;


               cout << BBox.Class <<" detected." << " Center position is "<< BBox_center.x << "," << BBox_center.y << endl;

               cv::Mat coord_2d(3, 1, CV_64FC1);
               cv::Mat coord_3d(3, 1, CV_64FC1);
               cv::Point2f ball_position;
               cv::Mat homo(3, 3, CV_64FC1);
               cv::Point2f moving_position;

               double ball_radius = 0.1f;

               if(head_pitch < 35.0/180*M_PI)
               {
                   //only detection and publish
                   ROS_WARN("Looking upper-----");

                   //publish the message
                   ball_msg.ball_detected = true;

                   ROS_ERROR("Corrected ball found");

                   ball_msg.header.stamp = msg->header.stamp;
                   ball_msg.kick_time= 100.0;
                   ball_msg_pub.publish(ball_msg);

               }
               else if((head_pitch > 35.0/180*M_PI) && head_yaw > 0.0/180*M_PI)
               {
                   //looking left
                   ROS_WARN("Looking left corner");

                   if(head_down==false)
                   {
                       //first time head_down
                       head_down = true;
                       ros::Duration(2).sleep();
                   }
                   else
                   {
                       //in kick position
                       coord_2d.at<double>(0) = BBox_center.x;
                       coord_2d.at<double>(1) = BBox_center.y;
                       coord_2d.at<double>(2) = 1;

                       //need to revise according to 45,-8 degree with left camera
                       homo.at<double>(0,0) = 0.440246;
                       homo.at<double>(1,0) = -0.0965702;
                       homo.at<double>(2,0) = 3.8173e-05;
                       homo.at<double>(0,1) = 0.254487;
                       homo.at<double>(1,1) = 0.662561;
                       homo.at<double>(2,1) = 0.00260807;
                       homo.at<double>(0,2) = -296.193;
                       homo.at<double>(1,2) = -117.396;
                       homo.at<double>(2,2) = 1;

                       coord_3d.at<double>(0) = (homo.at<double>(0,0) * coord_2d.at<double>(0) + homo.at<double>(0,1) * coord_2d.at<double>(1) + homo.at<double>(0,2) * coord_2d.at<double>(2));
                       coord_3d.at<double>(1) = (homo.at<double>(1,0) * coord_2d.at<double>(0) + homo.at<double>(1,1) * coord_2d.at<double>(1) + homo.at<double>(1,2) * coord_2d.at<double>(2));
                       coord_3d.at<double>(2) = (homo.at<double>(2,0) * coord_2d.at<double>(0) + homo.at<double>(2,1) * coord_2d.at<double>(1) + homo.at<double>(2,2) * coord_2d.at<double>(2));

                       coord_3d.at<double>(0) = coord_3d.at<double>(0) / coord_3d.at<double>(2);
                       coord_3d.at<double>(1) = coord_3d.at<double>(1) / coord_3d.at<double>(2);

                       //x coordinate w.r.t robot home position
                       ball_position.x = -(coord_3d.at<double>(1) - 125);
                       //y coordinate w.r.t robot home position
                       ball_position.y = -(coord_3d.at<double>(0) - 77);

                       cout << "ball center position w.r.t robot home coord system is :" << ball_position.x << "," << ball_position.y <<endl;

                       double ball_bearing;
                       double ball_range;

                       ball_bearing = atan(ball_position.y / ball_position.x);
                       ball_range = sqrt(ball_position.x * ball_position.x + ball_position.y * ball_position.y) / 100  - ball_radius;

    //                   ball_range = sqrt(ball_position.x * ball_position.x + ball_position.y * ball_position.y) / 100;

                       ball_msg.header.stamp = ros::Time::now();
                       ball_msg.ball_detected = true;
                       ball_msg.ball_bearing = ball_bearing;
                       ball_msg.ball_range = ball_range;

                       ROS_INFO("Ball detected, ready for kick position is %f, %f", ball_msg.ball_bearing*180/PI, ball_msg.ball_range);

    //                   ball_msg_pub.publish(ball_msg);

                       if(first_frame)
                       {

                           ROS_WARN("it is first frame");
                           first_frame = false;
                           center_vec.push_back(ball_position);

                            //minimal frame for position prediction is 2

                       }
                       else
                       {
                           double moving_range;

                           moving_position.x = ball_position.x-center_vec.back().x;
                           moving_position.y = ball_position.y-center_vec.back().y;

                           moving_range = sqrt( moving_position.x * moving_position.x + moving_position.y * moving_position.y );

                           ROS_ERROR("moving range is %f", moving_range);

                           center_vec.push_back(ball_position);

                           if(moving_range > moving_indicator)
                           {
                                moving_flag = true;

                                if(moving_flag)
                                {
                                    moving_flag = false;

                                    ROS_ERROR("ATTENTION! Ball is moving! Preparing for kicking!");

                                    ball_msg.header.stamp = ros::Time::now();
                                    ball_msg.ball_detected = true;
                                    ball_msg.kick_time = t2kick;
                                    ball_msg_pub.publish(ball_msg);

    //                                time_vec.push_back(pc_timestamp);

    //                                time_n = pc_timestamp - time_vec.at(0);

    //                                ROS_ERROR("ATTENTION! Ball is moving, current velocity is %f, %f", moving_position.x/100/time_n, moving_position.y/100/time_n);

    //                                time_interval.push_back(time_n);//ç¸å¯¹äºfirst frame

    //                                time_intervalsum += time_n;
    //                                time_interval2sum += time_n * time_n;
    //                                time_intervalpsum.x += time_n * ball_position.x;
    //                                time_intervalpsum.y += time_n * ball_position.y;

    //                                psum.x += ball_position.x;
    //                                psum.y += ball_position.y;
    //                                //end

    //                                prediction(ball_position);


                                }


                            }

                       }

                   }




               }
               else if((head_pitch > 35.0/180*M_PI) && head_yaw < 0.0/180*M_PI)
               {

                    //looking right
                   ROS_WARN("Looking right corner");

                   if(head_down==false)
                   {
                       //first time head_down
                       head_down = true;
                       ros::Duration(2).sleep();
                   }
                   else
                   {

                       //in kick position
                       coord_2d.at<double>(0) = BBox_center.x;
                       coord_2d.at<double>(1) = BBox_center.y;
                       coord_2d.at<double>(2) = 1;

                       //need to revise according to 45,-8 degree with left camera
                       homo.at<double>(0,0) = 0.460338;
                       homo.at<double>(1,0) = 0.122519;
                       homo.at<double>(2,0) = -3.03184e-06;
                       homo.at<double>(0,1) = -0.0110615;
                       homo.at<double>(1,1) = 0.683556;
                       homo.at<double>(2,1) = 0.00276201;
                       homo.at<double>(0,2) = -193.25;
                       homo.at<double>(1,2) = -293.831;
                       homo.at<double>(2,2) = 1;

                       coord_3d.at<double>(0) = (homo.at<double>(0,0) * coord_2d.at<double>(0) + homo.at<double>(0,1) * coord_2d.at<double>(1) + homo.at<double>(0,2) * coord_2d.at<double>(2));
                       coord_3d.at<double>(1) = (homo.at<double>(1,0) * coord_2d.at<double>(0) + homo.at<double>(1,1) * coord_2d.at<double>(1) + homo.at<double>(1,2) * coord_2d.at<double>(2));
                       coord_3d.at<double>(2) = (homo.at<double>(2,0) * coord_2d.at<double>(0) + homo.at<double>(2,1) * coord_2d.at<double>(1) + homo.at<double>(2,2) * coord_2d.at<double>(2));

                       coord_3d.at<double>(0) = coord_3d.at<double>(0) / coord_3d.at<double>(2);
                       coord_3d.at<double>(1) = coord_3d.at<double>(1) / coord_3d.at<double>(2);

                       //x coordinate w.r.t robot home position
                       ball_position.x = -(coord_3d.at<double>(1) - 125);
                       //y coordinate w.r.t robot home position
                       ball_position.y = -(coord_3d.at<double>(0) - 30);

                       cout << "ball center position w.r.t robot home coord system is :" << ball_position.x << "," << ball_position.y <<endl;

                       double ball_bearing;
                       double ball_range;

                       ball_bearing = atan(ball_position.y / ball_position.x);
                       ball_range = sqrt(ball_position.x * ball_position.x + ball_position.y * ball_position.y) / 100  - ball_radius;

    //                   ball_range = sqrt(ball_position.x * ball_position.x + ball_position.y * ball_position.y) / 100;

                       ball_msg.header.stamp = ros::Time::now();
                       ball_msg.ball_detected = true;
                       ball_msg.ball_bearing = ball_bearing;
                       ball_msg.ball_range = ball_range;

                       ROS_INFO("Ball detected, ready for kick position is %f, %f", ball_msg.ball_bearing*180/PI, ball_msg.ball_range);

    //                   ball_msg_pub.publish(ball_msg);

                       if(first_frame)
                       {

                           ROS_WARN("it is first frame");
                           first_frame = false;
                           center_vec.push_back(ball_position);

                            //minimal frame for position prediction is 2

                       }
                       else
                       {
                           double moving_range;

                           moving_position.x = ball_position.x-center_vec.back().x;
                           moving_position.y = ball_position.y-center_vec.back().y;

                           moving_range = sqrt( moving_position.x * moving_position.x + moving_position.y * moving_position.y );

                           center_vec.push_back(ball_position);

                           if(moving_range > moving_indicator)
                           {
                                moving_flag = true;

                                if(moving_flag)
                                {
                                    moving_flag = false;

                                    ROS_ERROR("ATTENTION! Ball is moving! Preparing for kicking!");

                                    ball_msg.header.stamp = ros::Time::now();
                                    ball_msg.ball_detected = true;
                                    ball_msg.kick_time = t2kick;
                                    ball_msg_pub.publish(ball_msg);

    //                                time_vec.push_back(pc_timestamp);

    //                                time_n = pc_timestamp - time_vec.at(0);

    //                                ROS_ERROR("ATTENTION! Ball is moving, current velocity is %f, %f", moving_position.x/100/time_n, moving_position.y/100/time_n);

    //                                time_interval.push_back(time_n);//ç¸å¯¹äºfirst frame

    //                                time_intervalsum += time_n;
    //                                time_interval2sum += time_n * time_n;
    //                                time_intervalpsum.x += time_n * ball_position.x;
    //                                time_intervalpsum.y += time_n * ball_position.y;

    //                                psum.x += ball_position.x;
    //                                psum.y += ball_position.y;
    //                                //end

    //                                prediction(ball_position);


                                }


                            }

                       }

                   }

               }

           }


       }

   }

//   void prediction(Point2f center)
//   {
//       float predict_range = 0;
//       predict_range = sqrt(center.x * center.x + center.y * center.y) / 100 - 0.1f;
//       //float kick_thresh = 2.0;
//       //added by luohy
//       Point2f p0;
//       p0.x = ((lambda+time_interval2sum)*psum.x-time_intervalsum*time_intervalpsum.x)/(time_vec.size()*(lambda+time_interval2sum)-time_intervalsum*time_intervalsum);
//       p0.y = ((lambda+time_interval2sum)*psum.y-time_intervalsum*time_intervalpsum.y)/(time_vec.size()*(lambda+time_interval2sum)-time_intervalsum*time_intervalsum);
//       Point2f v;
//       v.x = (time_vec.size()*time_intervalpsum.x-time_intervalsum*psum.x)/(time_vec.size()*(lambda+time_interval2sum)-time_intervalsum*time_intervalsum);
//       v.y = (time_vec.size()*time_intervalpsum.y-time_intervalsum*psum.y)/(time_vec.size()*(lambda+time_interval2sum)-time_intervalsum*time_intervalsum);

//       ROS_INFO("Current velocity is %f, %f", v.x/100, v.y/100);

//       double t2kick = 0.5*(v.x*time_vec.at(0)-p0.x)/v.x+0.5*(v.y*time_vec.at(0)-p0.y)/v.y-time_vec.at(time_vec.size()-1);
//       //end
////       if(predict_range < kick_thresh)
////       {

//           ROS_ERROR("Ball ready for kicking time is %f second later", t2kick);

////       }
//        if(t2kick>=0)
//        {
//            ball_msg.kick_time = t2kick;
//            ball_msg_pub.publish(ball_msg);
//        }
//        else cout<<"t2kick<0!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<endl;

//   }


};

int main (int argc, char** argv)
{
   // Initialize ROS
   ros::init (argc, argv, "rolling_ball_detect");

   BallDetector ic;

   first_frame = true;
   moving_flag = false;
   center_vec.clear();
//   time_vec.clear();
//   time_interval.clear();
   head_down = false;

   ROS_INFO("Rolling ball detection program starting...");


   while (ros::ok())
   {
       ros::spinOnce();

   }

   return 0;

}


