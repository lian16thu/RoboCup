#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/opencv.hpp>

//#include "vision/ball.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
//#include "vision/ObjectOnImage.h"
#include "vision/Ball.h"
//#include "vision/DepthRequest.h"
#include <cstdlib>
#include <math.h>

//#include "message_filters/subscriber.h"
//#include "message_filters/synchronizer.h"
//#include "message_filters/sync_policies/approximate_time.h"
//#include "message_filters/cache.h"
#include "sensor_msgs/Image.h"
//#include "decision/UDPReceived.h"

//#include "opencv2/xobjdetect.hpp"

#include "vision_define.h"
#include <opencv/cv.h>
#include <vector>

using namespace std;
//using namespace message_filters;
using namespace cv;
//using namespace xobjdetect;

int pix_threshold = 0;
double PI = 3.1415926;
double length_thresh = 0.9;
double area_thresh = 0.8;

int frame_buffer_size = 60;

double fx = 527.569432;
double fy = 525.817154;

//adjust parameters:
const int MAX_NUM_OBJECTS=50;
const int min_radius = 50;
const int max_radius = 180;
int MIN_OBJECT_AREA = min_radius * min_radius;
int MAX_OBJECT_AREA = max_radius * max_radius;

//define field range in HSV color space
int hmin = 0;
int hmax = 30;
int smin = 180;
int smax = 255;
int vmin = 0;
int vmax = 255;
//const Scalar SCALAR_FIELD_LOW = Scalar(hmin, smin, vmin);
//const Scalar SCALAR_FIELD_HIGH = Scalar(hmax, smax, vmax);
//const Scalar SCALAR_FIELD_LOW_ROI = Scalar(44, 103, 0);
//const Scalar SCALAR_FIELD_HIGH_ROI = Scalar(72, 255, 255);
double ball_size_threshold_low = 40;
double ball_size_threshold_high = 100;

double ball_xradius;
double ball_yradius;

double PCL_center_x;
double PCL_center_y;

Point2d PCL_ball_loc;
int ROI_size = 120;

float head_pitch = 1.43;//0.43;
float head_yaw = 0.0;

double start_time;
double end_time;

struct AreaCmp {
AreaCmp(const vector<float>& _areas) : areas(&_areas) {}
bool operator()(int a, int b) const { return (*areas)[a] > (*areas)[b]; }
const vector<float>* areas;
};

//typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, vision::ObjectOnImage> MySyncPolicy;
//message_filters::Cache<sensor_msgs::Image> image_cache;
sensor_msgs::Image image_buffer;

//typedef message_filters::sync_policies::ApproximateTime<vision::ObjectOnImage, vision::ObjectOnImage> MySyncPolicy;

//static bool abs_compare(int a, int b)
//{
//    return (abs(a) < abs(b));
//}

Mat homo(3, 3, CV_64FC1);

class BallDetector
{

  ros::NodeHandle nh;
  image_transport::ImageTransport it;
  image_transport::Subscriber image_sub_1;
    image_transport::Subscriber image_sub_2;
  //message_filters::Subscriber<sensor_msgs::Image> *image_sub_sync;
  ros::Subscriber head_rotation_sub;

  image_transport::Publisher image_pub1;
  image_transport::Publisher image_pub2;

  image_transport::Publisher image_test1;
  image_transport::Publisher image_test2;
  image_transport::Publisher image_test3;
  image_transport::Publisher image_test4;

  ros::Publisher ball_msg_pub;
  ros::Publisher ball_kick_pub;

  ros::Subscriber ball_img_loc_sub;


  Mat image_ball;
  Mat image_raw;
  Mat image_raw_last_frame;
  Mat image_ball_detect;
  Mat image_hsv;
  Mat image_hue;
  Mat image_hue_hist;
  Mat image_gray;
  Mat image_cvt;
  Mat SE3x3 = getStructuringElement(MORPH_ELLIPSE, Size(3,3));
  Mat SE5x5 = getStructuringElement(MORPH_ELLIPSE, Size(5,5));

  vector<double> ball_u, ball_v;

  ros::Time PCL_time;
  vector<double> frame_time;
  vector<double> ball_x, ball_y;
  int ball_size;

  ros::ServiceClient depth_request_client;

  vision::Ball ball_msg;

  Rect ball_loc_ROI;
  Rect ball_loc_2D_ROI;
  Rect ball_loc_PCL_ROI;

//  vector<float> radius_variance_detect;
//  vector<float> ball_range_vector;

  Mat ball_image_ROI;
  Mat image_hue_PCL;

  struct candidate_ball
  {
     Point2i center;
     float radius;
     bool true_ball;
     //float range;
     float bearing;
     int ball_center_x;
     int ball_center_y;

  };

  candidate_ball detected_ball;

  vector<candidate_ball> candidate_ball_vec;


public:
   BallDetector():it(nh)
   {
     // Subscrive to input video feed and publish output video feed

     image_sub_2 = it.subscribe("/zed/left/image_rect_color", 1, &BallDetector::Detect2D_Callback_camera2, this);
//     image_sub_1 = it.subscribe("/camera1/rgb/image_rect_color", 1, &BallDetector::Detect2D_Callback_camera1, this);
//     image_sub = it.subscribe("/image_raw", 1, &BallDetector::Detect2D_Callback, this);
     ball_msg_pub = nh.advertise<vision::Ball>("/vision/ball", 10);

     //Initialization
     image_raw_last_frame = Mat::zeros(Height, Width, CV_8UC3);
     image_raw = Mat::zeros(Height, Width, CV_8UC3);

     image_gray = Mat::zeros(Height, Width, CV_8UC1);


     ROS_INFO("class initialized..");

    //Initialize ball msg
//     ball_msg.header.stamp = ros::Time::now();
//     ball_msg.ball_detected = false;
//     ball_msg.ball_range = 0.0;
//     ball_msg.ball_bearing = 0.0;
//     ball_msg.ball_x = 0;
//     ball_msg.ball_y = 0;

    }


   void Detect2D_Callback_camera1(const sensor_msgs::ImageConstPtr& Image_msg);
   void Detect2D_Callback_camera2(const sensor_msgs::ImageConstPtr& Image_msg);
   candidate_ball candidate_ball_check(Mat image, Point2i center, float radius);


};

void BallDetector::Detect2D_Callback_camera1(const sensor_msgs::ImageConstPtr& Image_msg)
{

}

/*************************************************************************************************************************/
/*  Synchronize 2D frame time with computed candidate location from Point Cloud time, then complete the object detection */
/*  If detected, publish the ball message to ROS.                                                                        */
/*************************************************************************************************************************/
void BallDetector::Detect2D_Callback_camera2(const sensor_msgs::ImageConstPtr& Image_msg)
{
        //using cv_bridge to transport the image from ROS to openCV cvMat, find closet time in frame time.
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
           cv_ptr = cv_bridge::toCvCopy(Image_msg, sensor_msgs::image_encodings::BGR8);

        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;

        }

        Mat &image_raw = cv_ptr->image;

        image_raw.copyTo(image_raw_last_frame);

        startWindowThread();
        namedWindow("BallTracking2D:raw");
        imshow("BallTracking2D:raw", image_raw_last_frame);
        waitKey(5);
        imwrite("image_raw_last_frame.jpg", image_raw_last_frame);
        waitKey(5);

        //--------------------------------Start Section I  find field---------------------------------//
        GaussianBlur(image_raw_last_frame, image_raw_last_frame, Size(5,5), 3, 3);
        cvtColor(image_raw_last_frame, image_hsv, COLOR_BGR2HSV);
        inRange(image_hsv, Scalar(hmin, smin, vmin), Scalar(hmax, smax, vmax), image_hue);

        //ball image
        imshow("BallTracking2D:image_orange", image_hue);
        waitKey(3);

 //       adaptiveThreshold(image_hue, image_hue, 255, CV_ADAPTIVE_THRESH_GAUSSIAN_C, CV_THRESH_BINARY, 9, 0);


        morphologyEx(image_hue, image_hue, MORPH_CLOSE, SE5x5);
        morphologyEx(image_hue, image_hue, MORPH_CLOSE, SE3x3);

        morphologyEx(image_hue, image_hue, MORPH_OPEN, SE3x3);
        erode(image_hue, image_hue, SE5x5);
        erode(image_hue, image_hue, SE5x5);

        //ball image
//        imshow("BallTracking2D:image_adaptive_filtered", image_hue);
//        waitKey(3);

//-------------------------
//        vector< Vec4i > hierarchy;
//        vector< vector< Point> > contours;
//       Point2f center;
//        float radius;

//        findContours(image_hue, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

//        //Label the results
//        vector<int> sortIdx(contours.size());
//        vector<float> areas(contours.size());

//        for( int n = 0; n < (int)contours.size(); n++ )
//        {
//            sortIdx[n] = n;
//            areas[n] = contourArea(contours[n], false);
//        }

//        // sort contours so that the largest contours go first
//        std::sort(sortIdx.begin(), sortIdx.end(), AreaCmp(areas));

//         // if the ball is not occuluded
//        for( int n = 0; n < (int)sortIdx.size(); n++ )
//        {
//             int idx = sortIdx[n];

//             minEnclosingCircle(contours[n], center, radius);

////                float area_circle;
////                area_circle = contourArea(contours[n], false);
////                cout << "areas is " << area_circle << endl;

////                circle(image_raw_last_frame, Point(center.x, center.y), radius, CV_RGB(255,0,0), 2);
////                                    imshow("BallTracking2D:ball_candidate_test", image_raw_last_frame);
////                                    waitKey(3);

////               cout << "radius is " << radius << endl;
////                cout << "center is (" << center.x  <<", "<< center.y << ")" << endl;
////                 cout << "areas is " << areas[idx] << endl;
//             Point2i circle_center;
//             circle_center = Point2i(center);
////                cout << "circle_center is (" << circle_center.x  <<", "<< circle_center.y << ")" << endl;

//             //if the ball is not occuluded, obtain the center position
//             if( (radius <= ball_size_threshold_high) && (radius >= ball_size_threshold_low) )// && (area_circle > PI*radius*radius/4) )
//             {
//                        circle(image_raw_last_frame, Point(circle_center.x, circle_center.y), radius, CV_RGB(0,0,255), 2);
////                                    imshow("BallTracking2D:detected as candidate", image_raw_last_frame);
////                                    waitKey(3);

////                       ROS_ERROR("areas is %f", areas[idx]);
//                    //ROS_INFO("Ball detected by 2D processing, the center is = %f, %f, radius is %f. area is %f", center.x + ball_loc_ROI.x, center.y + ball_loc_ROI.y, radius, areas[idx]);

//                    //candidate_ball_vec.push_back(Point(center.x, center.y, radius);


//                 //detected_ball  = candidate_ball_check(image_raw_last_frame, Point2i(circle_center.x, circle_center.y), radius);

//             }


//        }

////        if(detected_ball.true_ball==true)
////        {
////            //ball detected
////            circle(image_raw_last_frame, Point(detected_ball.center.x, detected_ball.center.y), detected_ball.radius, CV_RGB(0,0,255), 2);
////            imshow("BallTracking2D:ball_detected", image_raw_last_frame);
////            waitKey(3);
//           //       ROS_INFO("Ball detected, center is %d, %d", detected_ball.center.y, detected_ball.center.x);

////               sensor_msgs::ImagePtr ball_image_out2 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image_raw_last_frame).toImageMsg();
////               image_pub2.publish(ball_image_out2);

////            ball_msg.ball_range = detected_ball.range;
////            ball_msg.ball_bearing = detected_ball.bearing;
//            //ball_range_vector.push_back(detected_ball.range);

//            //compute the bearing and then publish the message to ROS
////            ball_msg.header.stamp = Detect2D_time;
////            ball_msg.ball_detected = true;
////            ball_msg.ball_center_x = detected_ball.center.x;
////            ball_msg.ball_center_y = detected_ball.center.y;
////            ball_msg.ball_radius = detected_ball.radius;

//            //publish the ball information to ROS
////            ball_msg_pub.publish(ball_msg);

////            ROS_ERROR("Ball detected, center is %d, %d", detected_ball.center.x, detected_ball.center.y);


////        }
////        else
////        {
////            //publish the ball information to ROS
//////            ball_msg.header.stamp = Detect2D_time;
//////            ball_msg.ball_detected = false;
//////            ball_msg.ball_bearing = 0.0;
//////            ball_msg.ball_range = 0.0;
////            //ball_msg_pub.publish(ball_msg);
////            ROS_INFO("No ball detected");

////        }

//-------------------------
        Mat labels, stats, centroids;
        float radius_width, radius_height;
        int i, nccomps = connectedComponentsWithStats(image_hue, labels, stats, centroids, 8);
                    //cout << "Total Connected Components Detected: " << nccomps << endl;

                       // imwrite("goalpost.jpg", image_binary);

        Mat coord_2d(3, 1, CV_64FC1);
        Mat coord_3d(3, 1, CV_64FC1);

        double ball_radius = 10.0f;

        homo.at<double>(0,0) = 0.183286;
        homo.at<double>(1,0) = -0.00105063;
        homo.at<double>(2,0) = -6.0161e-05;
        homo.at<double>(0,1) = 0.0329943;
        homo.at<double>(1,1) = -0.164617;
        homo.at<double>(2,1) = 0.00124523;
        homo.at<double>(0,2) = -31.6952;
        homo.at<double>(1,2) = 84.0783;
        homo.at<double>(2,2) = 1;

        bool found = false;
        Point2f ball_position;

        for( i = 0; i <= nccomps; i++ )
        {

            if( (stats.at<int>(i, CC_STAT_AREA) != 0) && (stats.at<int>(i, CC_STAT_AREA) != 307200) && (stats.at<int>(i, CC_STAT_HEIGHT) > 40) )
            {

//                            cout << "CC_STAT_WIDTH is "<<stats.at<int>(i, CC_STAT_WIDTH) <<endl;
//                            cout << "CC_STAT_HEIGHT is "<<stats.at<int>(i, CC_STAT_HEIGHT) <<endl;
//                cout << "CC_STAT_AREA is "<<stats.at<int>(i, CC_STAT_AREA) <<endl;

//circle(image_raw_last_frame, Point2f(centroids.at<double>(i,0), centroids.at<double>(i,1)), cvRound(radius), Scalar(255,0,0), 2, LINE_AA);
                if( (stats.at<int>(i, CC_STAT_AREA) > MIN_OBJECT_AREA) && (stats.at<int>(i, CC_STAT_AREA) < MAX_OBJECT_AREA))
                {
                    //cout << "i is " << i << endl;
                    cout << "center.x is " << centroids.at<double>(i,0) <<endl;
                    cout << "center.y is " << centroids.at<double>(i,1) <<endl;

                //putText(image_raw_last_frame,"Tracking Object",Point(0,50),2,1,Scalar(0,255,0),2);

                //draw object location on screen

                    cout << "CC_STAT_WIDTH is "<<stats.at<int>(i, CC_STAT_WIDTH) <<endl;
                    cout << "CC_STAT_HEIGHT is "<<stats.at<int>(i, CC_STAT_HEIGHT) <<endl;

                    cout << "CC_STAT_LEFT is "<<stats.at<int>(i, CC_STAT_LEFT) <<endl;
                    cout << "CC_STAT_TOP is "<<stats.at<int>(i, CC_STAT_TOP) <<endl;

                    cout << "CC_STAT_AREA is "<<stats.at<int>(i, CC_STAT_AREA) <<endl;

                    radius_width = stats.at<int>(i, CC_STAT_WIDTH)/2;
                    radius_height = stats.at<int>(i, CC_STAT_HEIGHT)/2;

                    cout << "radius is "<< MAX(radius_width,radius_height) <<endl;

                    if(stats.at<int>(i, CC_STAT_LEFT)==0)
                    {
                        circle(image_raw_last_frame, Point2f(centroids.at<double>(i,0), centroids.at<double>(i,1)), cvRound(radius_height), Scalar(0,0,255), 2, LINE_AA);
                        circle(image_raw_last_frame, Point2f(centroids.at<double>(i,0), centroids.at<double>(i,1)), 2, Scalar(0,255,0), 2, LINE_AA);
                    }
                    else if(stats.at<int>(i, CC_STAT_TOP)==0)
                    {
                        circle(image_raw_last_frame, Point2f(centroids.at<double>(i,0), centroids.at<double>(i,1)), cvRound(radius_width), Scalar(0,0,255), 2, LINE_AA);
                        circle(image_raw_last_frame, Point2f(centroids.at<double>(i,0), centroids.at<double>(i,1)), 2, Scalar(0,255,0), 2, LINE_AA);

                    }
                    else if(stats.at<int>(i, CC_STAT_TOP) > 311)
                    {

                        circle(image_raw_last_frame, Point2f(centroids.at<double>(i,0), stats.at<int>(i, CC_STAT_TOP) + 85), 2, Scalar(0,255,0), 2, LINE_AA);
                        ROS_ERROR("Ball occuluded, y is %d ", stats.at<int>(i, CC_STAT_TOP) + 85);

                    }
                    else
                    {
                        circle(image_raw_last_frame, Point2f(centroids.at<double>(i,0), centroids.at<double>(i,1)), cvRound(MAX(radius_width,radius_height)), Scalar(0,0,255), 2, LINE_AA);
                        circle(image_raw_last_frame, Point2f(centroids.at<double>(i,0), centroids.at<double>(i,1)), 2, Scalar(0,255,0), 2, LINE_AA);

                    }


                    //ROS_ERROR("Ball detected, center is %f, %f", centroids.at<double>(i,0), centroids.at<double>(i,1));

                    ball_position = Point2f(centroids.at<double>(i,0), centroids.at<double>(i,1));

                    found = true;

                }

            }

        }
 //-------------------------
        imshow("BallTracking2D:Detected", image_raw_last_frame);
        waitKey(5);

//        cout << "homography matrix is"<<endl;

//        for(int j = 0; j < homo.rows; j++)
//        {
//            for(int i = 0; i < homo.cols; i++)
//            {
//               cout<<"home("<< i<<", "<< j<<") is "<<homo.at<double>(i,j) <<endl;
//            }
//        }

//        if(found)
//        {
//            coord_2d.at<double>(0) = ball_position.x;
//            coord_2d.at<double>(1) = ball_position.y;
//            coord_2d.at<double>(2) = 1;

//            ROS_ERROR("Ball detected, center is %f, %f", ball_position.x, ball_position.y);


//            coord_3d.at<double>(0) = (homo.at<double>(0,0) * coord_2d.at<double>(0) + homo.at<double>(0,1) * coord_2d.at<double>(1) + homo.at<double>(0,2) * coord_2d.at<double>(2));
//            coord_3d.at<double>(1) = (homo.at<double>(1,0) * coord_2d.at<double>(0) + homo.at<double>(1,1) * coord_2d.at<double>(1) + homo.at<double>(1,2) * coord_2d.at<double>(2));
//            coord_3d.at<double>(2) = (homo.at<double>(2,0) * coord_2d.at<double>(0) + homo.at<double>(2,1) * coord_2d.at<double>(1) + homo.at<double>(2,2) * coord_2d.at<double>(2));

//            coord_3d.at<double>(0) = coord_3d.at<double>(0) / coord_3d.at<double>(2);
//            coord_3d.at<double>(1) = coord_3d.at<double>(1) / coord_3d.at<double>(2);

//           cout << "3D location of ball is :" << coord_3d.at<double>(0)-27 << "," << coord_3d.at<double>(1)- ball_radius <<endl;

//           ball_msg.header.stamp = ros::Time::now();
//           ball_msg.ball_detected = true;

//           ball_msg.ball_x = (coord_3d.at<double>(0)-27)/100;
//           ball_msg.ball_y = (coord_3d.at<double>(1)- ball_radius)/100;
//           ball_msg_pub.publish(ball_msg);

//        }
//        else
//        {

//        }


}

BallDetector::candidate_ball BallDetector::candidate_ball_check(Mat image, Point2i center, float radius)
{

   ROS_INFO("candidate ball center is %d,%d, radius is %f", center.x, center.y, radius);

    Mat candidate_image_ROI;
    Rect candidate_loc_ROI;
    Mat candidate_gray;
    Mat candidate_hsv;
    Mat candidate_hue;
    candidate_ball temp_candidate_ball;

    candidate_loc_ROI.x = center.x - ceil(radius);
    candidate_loc_ROI.y = center.y - ceil(radius);
    candidate_loc_ROI.width = ceil(radius*2);
    candidate_loc_ROI.height = ceil(radius*2);

    if(candidate_loc_ROI.x < 0)
    {
        candidate_loc_ROI.x = 0;
    }
    if(candidate_loc_ROI.y < 0)
    {
        candidate_loc_ROI.y = 0;
    }

    if(candidate_loc_ROI.y + candidate_loc_ROI.width >= 480)
    {
        candidate_loc_ROI.y = 479 - candidate_loc_ROI.width;
    }

    if(candidate_loc_ROI.x + candidate_loc_ROI.height >= 640)
    {
        candidate_loc_ROI.x = 639 - candidate_loc_ROI.height;
    }

//    cout << "candidate_loc_ROI.x is " <<candidate_loc_ROI.x<<" candidate_loc_ROI.y is "<<candidate_loc_ROI.y<<endl;
//    cout << "candidate_loc_ROI.width is " <<candidate_loc_ROI.width<<" candidate_loc_ROI.height is "<<candidate_loc_ROI.height<<endl;

    candidate_image_ROI = image(candidate_loc_ROI);

//    sensor_msgs::ImagePtr candidate_loc_ROI_test = cv_bridge::CvImage(std_msgs::Header(), "bgr8", candidate_image_ROI).toImageMsg();
//    image_test2.publish(candidate_loc_ROI_test);

    imshow("BallTracking2D:candidate_image_ROI", candidate_image_ROI);
    waitKey(5);

    cvtColor(candidate_image_ROI, candidate_hsv, COLOR_BGR2HSV);
    inRange(candidate_hsv, Scalar(hmin, smin, vmin), Scalar(hmax, smax, vmax), candidate_hue);

//    cvtColor(candidate_image_ROI, candidate_gray, COLOR_BGR2GRAY);

    int num_orange = 0;
    int non_orange = 0;

    double orange_ratio = 0;

    double candidate_orange_ratio = 0.5;
    double orange_threhold = 20;

    for(int i = 0; i < candidate_hue.cols; i++)
    {
        for(int j = 0; j < candidate_hue.rows; j++)
        {
             if(candidate_hue.at<unsigned char>(i,j) < orange_threhold)
             {

                 num_orange++;

             }

        }


     }

//    imshow("BallTracking3D:candidate_gray", candidate_gray);
//    waitKey(10);

    orange_ratio = double (num_orange)/(candidate_hue.cols*candidate_hue.rows);


    if((orange_ratio > candidate_orange_ratio) )
    {

//        float position_x = (-0.2142*center.y + 112.016)/100;
//        float position_y = (-0.216*center.x + 65.793)/100;

//        depth = sqrt(position_x*position_x+position_y*position_y);

//        //calculate the bearing in radians
//        if((center.x)> 320)
//        {
//            //bearing direction is positive
//            temp_candidate_ball.bearing = atan((center.x - 320)*7.4/fx);

//        }
//        else
//        {

//            //bearing direction is negative
//            temp_candidate_ball.bearing = -atan((320 - center.x)*7.4/fx);

//        }

        temp_candidate_ball.true_ball = true;
        temp_candidate_ball.center = center;
        temp_candidate_ball.radius = radius;
        //temp_candidate_ball.range = depth;
        temp_candidate_ball.ball_center_x = center.x;
        temp_candidate_ball.ball_center_y = center.y;

        ROS_INFO("Ball detected, center is %d,%d, radius is %f, candidate ball orange_ratio is %f", center.x, center.y, radius, orange_ratio);


    }
    else
    {
        temp_candidate_ball.true_ball = false;
//        temp_candidate_ball.center = center;
//        temp_candidate_ball.radius = radius;
//        temp_candidate_ball.ball_center_x = center.x;
//        temp_candidate_ball.ball_center_y = center.y;

       ROS_ERROR("not ball, candidate ball orange_ratio is %f", orange_ratio);

    }

    return temp_candidate_ball;

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "balldetect");

    BallDetector ic;

    ROS_INFO("Ball detection program starting...");
    namedWindow("TrackBar", 10);
//    createTrackbar("gray_Threshold", "TrackBar", &gray_Threshold, 255);

    createTrackbar("hmin", "TrackBar", &hmin, 179);
    createTrackbar("hmax", "TrackBar", &hmax, 179);
    createTrackbar("smin", "TrackBar", &smin, 255);
    createTrackbar("smax", "TrackBar", &smax, 255);
    createTrackbar("vmin", "TrackBar", &vmin, 255);
    createTrackbar("vmax", "TrackBar", &vmax, 255);

    while (ros::ok())
    {
        //ROS_INFO("Spinned once");
        ros::spinOnce();

    }

    return 0;

}
