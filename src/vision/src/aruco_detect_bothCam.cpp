#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/opencv.hpp>

////#include "vision/ball.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
////#include "vision/ObjectOnImage.h"
////#include "vision/Ball.h"
////#include "vision/DepthRequest.h"
#include <cstdlib>
#include <math.h>

//#include "message_filters/subscriber.h"
//#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/cache.h"
#include "sensor_msgs/Image.h"
//#include "decision/UDPReceived.h"

#include "opencv2/aruco.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/core.hpp"
#include "opencv2/imgcodecs.hpp"

#include "vision_define.h"
#include <opencv/cv.h>
#include <vector>

#include <sstream>
#include "vision/Markers.h"

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include "fiducial_msgs/FiducialTransformArray.h"
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

using namespace std;
using namespace cv;

double fx = 5284.88423;
int errorCorrection_test = 90;
int maxErroneousBits_test = 90;
int OtsuDev_test = 5;
int AdapThreWinSizeMax_test = 53;
int AdapThreWinSizeMin_test = 3;
int AdapThreWinSizeStep_test = 4;
int polygonalApproxAccuracyRate_test = 4;
int perspectiveRemoveIgnoredMarginPerCell_test = 13;
int perspectiveRemovePixelPerCell_test = 8;

double PI = 3.1415926;

#define IMG_WIDTH 640
#define IMG_HEIGHT 480

//double to string helper function
string doubleToString(double number){

    //this function has a number input and string output
    std::stringstream ss;
    ss << number;
    return ss.str();
}

class BallDetector
{

  ros::NodeHandle nh;
  image_transport::ImageTransport it;
  image_transport::Subscriber image_sub_Cam1;
  image_transport::Subscriber image_sub_Cam2;

  image_transport::Publisher image_pub1;

  Mat image_raw_Cam1;
  Mat image_raw_last_frame_Cam1;
  Mat image_raw_Cam2;
  Mat image_raw_last_frame_Cam2;

//  ros::Time PCL_time;
//  vector<double> frame_time;

public:
  ros::Publisher marker_msg_pub;

  vision::Markers marker_msg_Cam1;
  vision::Markers marker_msg_Cam2;

  //SLAM
  ros::Publisher * pose_pub;

  struct marker
  {
     int id;
     double rotvec_x;
     double rotvec_y;
     double rotvec_z;
     double transvec_x;
     double transvec_y;
     double transvec_z;

  };

  marker detected_marker_Cam1;
  marker detected_marker_Cam2;

  vector<marker> detected_marker_vec_Cam1;
  vector<marker> detected_marker_vec_Cam2;

  string reference_frame_Cam1 = "camera1_link";
  string reference_frame_Cam2 = "camera2_link";

public:
   BallDetector():it(nh)
   {
     // Subscrive to input video feed and publish output video feed
//    image_sub = it.subscribe("/image_raw", 1, &BallDetector::Detect2D_Callback, this);
    image_sub_Cam2 = it.subscribe("/camera2/rgb/image_rect_color", 1, &BallDetector::Detect2D_Callback_Cam2, this);
    image_sub_Cam1 = it.subscribe("/camera1/rgb/image_rect_color", 1, &BallDetector::Detect2D_Callback_Cam1, this);

    marker_msg_pub = nh.advertise<vision::Markers>("/vision/marker", 10);

    pose_pub = new ros::Publisher(nh.advertise<fiducial_msgs::FiducialTransformArray>("/fiducial_transforms_camera", 1));


     //Initialization
     image_raw_last_frame_Cam1 = Mat::zeros(Height, Width, CV_8UC3);
     image_raw_Cam1 = Mat::zeros(Height, Width, CV_8UC3);
     image_raw_last_frame_Cam2 = Mat::zeros(Height, Width, CV_8UC3);
     image_raw_Cam2 = Mat::zeros(Height, Width, CV_8UC3);

     ROS_INFO("class initialized..");

    //Initialize marker msg
     marker_msg_Cam1.header.stamp = ros::Time::now();
     marker_msg_Cam1.header.frame_id = reference_frame_Cam1;
     marker_msg_Cam1.markers_detected = false;

     marker_msg_Cam2.header.stamp = ros::Time::now();
     marker_msg_Cam2.header.frame_id = reference_frame_Cam2;
     marker_msg_Cam2.markers_detected = false;

    }

   void Detect2D_Callback_Cam1(const sensor_msgs::ImageConstPtr& Image_msg);
   void Detect2D_Callback_Cam2(const sensor_msgs::ImageConstPtr& Image_msg);

   Mat rot2euler(const Mat & rotationMatrix);
   void estimatePoseSingleMarkers(const vector<vector<Point2f > >&corners,
                                  float markerLength,
                                  const cv::Mat &cameraMatrix,
                                  const cv::Mat &distCoeffs,
                                  vector<Vec3d>& rvecs, vector<Vec3d>& tvecs,
                                  vector<double>& reprojectionError);

};

static void getSingleMarkerObjectPoints(float markerLength, vector<Point3f>& objPoints) {

    CV_Assert(markerLength > 0);

    // set coordinate system in the middle of the marker, with Z pointing out
    objPoints.push_back(Vec3f(-markerLength / 2.f, markerLength / 2.f, 0));
    objPoints.push_back(Vec3f( markerLength / 2.f, markerLength / 2.f, 0));
    objPoints.push_back(Vec3f( markerLength / 2.f,-markerLength / 2.f, 0));
    objPoints.push_back(Vec3f(-markerLength / 2.f,-markerLength / 2.f, 0));
}

// Euclidean distance between two points
static double dist(const cv::Point2f &p1, const cv::Point2f &p2)
{
    double x1 = p1.x;
    double y1 = p1.y;
    double x2 = p2.x;
    double y2 = p2.y;

    double dx = x1 - x2;
    double dy = y1 - y2;

    return sqrt(dx*dx + dy*dy);
}

// estimate reprojection error
static double getReprojectionError(const vector<Point3f> &objectPoints,
                            const vector<Point2f> &imagePoints,
                            const Mat &cameraMatrix, const Mat  &distCoeffs,
                            const Vec3d &rvec, const Vec3d &tvec) {

    vector<Point2f> projectedPoints;

    cv::projectPoints(objectPoints, rvec, tvec, cameraMatrix,
                      distCoeffs, projectedPoints);

    // calculate RMS image error
    double totalError = 0.0;
    for (unsigned int i=0; i<objectPoints.size(); i++) {
        double error = dist(imagePoints[i], projectedPoints[i]);
        totalError += error*error;
    }
    double rerror = totalError/objectPoints.size();
    return rerror;
}

void BallDetector::estimatePoseSingleMarkers(const vector<vector<Point2f > >&corners,
                               float markerLength,
                               const cv::Mat &cameraMatrix,
                               const cv::Mat &distCoeffs,
                               vector<Vec3d>& rvecs, vector<Vec3d>& tvecs,
                               vector<double>& reprojectionError) {

    CV_Assert(markerLength > 0);

    vector<Point3f> markerObjPoints;
    getSingleMarkerObjectPoints(markerLength, markerObjPoints);
    int nMarkers = (int)corners.size();
    rvecs.reserve(nMarkers);
    tvecs.reserve(nMarkers);
    reprojectionError.reserve(nMarkers);

    // for each marker, calculate its pose
    for (int i = 0; i < nMarkers; i++) {

       cv::solvePnP(markerObjPoints, corners[i], cameraMatrix, distCoeffs,
                    rvecs[i], tvecs[i]);

       reprojectionError[i] =
          getReprojectionError(markerObjPoints, corners[i],
                               cameraMatrix, distCoeffs,
                               rvecs[i], tvecs[i]);
    }
}

// Compute area in image of a fiducial, using Heron's formula
// to find the area of two triangles
static double calcFiducialArea(const std::vector<cv::Point2f> &pts)
{
    const Point2f &p0 = pts.at(0);
    const Point2f &p1 = pts.at(1);
    const Point2f &p2 = pts.at(2);
    const Point2f &p3 = pts.at(3);

    double a1 = dist(p0, p1);
    double b1 = dist(p0, p3);
    double c1 = dist(p1, p3);

    double a2 = dist(p1, p2);
    double b2 = dist(p2, p3);
    double c2 = c1;

    double s1 = (a1 + b1 + c1) / 2.0;
    double s2 = (a2 + b2 + c2) / 2.0;

    a1 = sqrt(s1*(s1-a1)*(s1-b1)*(s1-c1));
    a2 = sqrt(s2*(s2-a2)*(s2-b2)*(s2-c2));
    return a1+a2;
}

Mat BallDetector::rot2euler(const Mat & rotationMatrix)
{
    Mat euler(3, 1, CV_64F);

    double m00 = rotationMatrix.at<double>(0,0);
    double m02 = rotationMatrix.at<double>(0,2);
    double m10 = rotationMatrix.at<double>(1,0);
    double m11 = rotationMatrix.at<double>(1,1);
    double m12 = rotationMatrix.at<double>(1,2);
    double m20 = rotationMatrix.at<double>(2,0);
    double m22 = rotationMatrix.at<double>(2,2);

    double x, y, z;
    // Assuming the angles are in radians.
      if (m10 > 0.998) { // singularity at north pole
        x = 0;
        y = CV_PI/2;
        z = atan2(m02,m22);
      }
      else if (m10 < -0.998) { // singularity at south pole
        x = 0;
        y = -CV_PI/2;
        z = atan2(m02,m22);
      }
      else
      {
        x = atan2(-m12,m11);
        y = asin(m10);
        z = atan2(-m20,m00);
      }

      euler.at<double>(0) = x;
      euler.at<double>(1) = y;
      euler.at<double>(2) = z;

    return euler;

}

/*************************************************************************************************************************/
/*  Synchronize 2D frame time with computed candidate location from Point Cloud time, then complete the object detection */
/*  If detected, publish the ball message to ROS.                                                                        */
/*************************************************************************************************************************/
void BallDetector::Detect2D_Callback_Cam1(const sensor_msgs::ImageConstPtr& Image_msg)
{

         ros::Time  Detect2D_time = Image_msg->header.stamp;
                  ROS_INFO("receive Cam 1");
 //         ROS_INFO("frame timestamp is %f", Detect2D_time.toSec());

        //using cv_bridge to transport the image from ROS to openCV cvMat, find closet time in frame time.
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
           cv_ptr = cv_bridge::toCvCopy(Image_msg, sensor_msgs::image_encodings::BGR8);
    //       start_time= ros::Time::now().toSec();
    //       ROS_INFO("start time is %f", start_time);

        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;

        }

        Mat &image_raw_Cam1 = cv_ptr->image;
        int width = image_raw_Cam1.cols;
        int height = image_raw_Cam1.rows;

        image_raw_Cam1.copyTo(image_raw_last_frame_Cam1);

        startWindowThread();
//        namedWindow("BallTracking2D:raw", WINDOW_AUTOSIZE);
//        imshow("BallTracking2D:raw", image_raw_last_frame);
//        waitKey(5);

        Mat image_rejected_Cam1;
        image_raw_Cam1.copyTo(image_rejected_Cam1);

        //--------------------------------Start Section I  find marker---------------------------------//
        vector< int > markerIds;
        vector< vector< Point2f > > markerCorners, rejectedCandidates;

        aruco::DetectorParameters parameters;

        Ptr<cv::aruco::Dictionary> markerDictionary = aruco::getPredefinedDictionary(aruco::DICT_4X4_50);
                //Ptr<cv::aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(aruco::DICT_6X6_250);
        Ptr<aruco::DetectorParameters> detectorParams = aruco::DetectorParameters::create();

        int borderBits = 2;
        detectorParams->markerBorderBits = borderBits;
        //detectorParams->doCornerRefinement = false;

        detectorParams->adaptiveThreshWinSizeMin = AdapThreWinSizeMin_test;
        detectorParams->adaptiveThreshWinSizeMax = AdapThreWinSizeMax_test;
        detectorParams->adaptiveThreshWinSizeStep = AdapThreWinSizeStep_test;
        detectorParams -> minOtsuStdDev = double(OtsuDev_test);
        detectorParams -> minMarkerDistanceRate = 0.03;
        detectorParams -> maxMarkerPerimeterRate = 3.0;

        detectorParams -> perspectiveRemoveIgnoredMarginPerCell = double(perspectiveRemoveIgnoredMarginPerCell_test/100);
        detectorParams -> perspectiveRemovePixelPerCell = perspectiveRemovePixelPerCell_test;
        detectorParams -> maxErroneousBitsInBorderRate = double(maxErroneousBits_test/100);
        detectorParams -> errorCorrectionRate = double(errorCorrection_test/100);

        //pamameters for pose estimation
        Mat camMatrix, distCoeffs;
        float cam_data [9] = {515.5079203874326, 0, 317.3374703863852, 0, 515.9706549395567, 234.3778675160045, 0, 0, 1};
        camMatrix = Mat(3, 3, CV_32F, cam_data);
        float dis_data [5] = {-0.00604187361096875, -0.02656712142303156, -0.003630905715142372, 0.001490305578152859, 0};
        distCoeffs = Mat(1, 5, CV_32F, dis_data);

//        cout << camMatrix << endl;
//        cout << distCoeffs << endl;

        vector< Vec3d > rvecs, tvecs;

        //parameter
        float markerDimension = 0.1;

        //int dictionaryId = 1; //parser.get<int>("d");
        bool showRejected = true;//parser.has("r");
        bool estimatePose = true;//parser.has("c");
        float markerLength = 0.1;//parser.get<float>("l");     

        // detect markers and estimate pose
        aruco::detectMarkers(image_raw_last_frame_Cam1, markerDictionary, markerCorners, markerIds, detectorParams, rejectedCandidates);

        vector<double> reprojectionError_Cam1;

        estimatePoseSingleMarkers(markerCorners, markerDimension, camMatrix, distCoeffs, rvecs, tvecs, reprojectionError_Cam1);


        //        if(estimatePose && markerIds.size() > 0)
//            aruco::estimatePoseSingleMarkers(markerCorners, markerLength, camMatrix, distCoeffs, rvecs,
//                                             tvecs);



//        ROS_INFO("marker size is %d", markerIds.size());

        fiducial_msgs::FiducialTransformArray fta_Cam1;
        fta_Cam1.header.stamp = Image_msg->header.stamp;
        fta_Cam1.header.frame_id = reference_frame_Cam1;
        fta_Cam1.image_seq = Image_msg->header.seq;

        int markercounter = 0;

        detected_marker_vec_Cam1.clear();
        marker_msg_Cam1.marker_id.clear();
        marker_msg_Cam1.marker_rotvec_x.clear();
        marker_msg_Cam1.marker_rotvec_y.clear();
        marker_msg_Cam1.marker_rotvec_z.clear();
        marker_msg_Cam1.marker_rotvec_x.clear();
        marker_msg_Cam1.marker_rotvec_y.clear();
        marker_msg_Cam1.marker_rotvec_z.clear();

        // draw results
        if(markerIds.size() > 0)
        {
            aruco::drawDetectedMarkers(image_raw_last_frame_Cam1, markerCorners, markerIds);

            for(int i = 0 ; i < markerIds.size(); i++)
            {
                aruco::drawAxis(image_raw_last_frame_Cam1, camMatrix, distCoeffs, rvecs[i], tvecs[i], markerDimension * 0.8f);

                double angle = norm(rvecs[i]);
                Vec3d axis = rvecs[i] / angle;
                ROS_WARN("Cam1: angle %f axis %f %f %f", angle, axis[0], axis[1], axis[2]);

                fiducial_msgs::FiducialTransform ft_Cam1;

                ft_Cam1.fiducial_id = markerIds[i];

                ft_Cam1.transform.translation.x = tvecs[i][0];
                ft_Cam1.transform.translation.y = tvecs[i][1];
                ft_Cam1.transform.translation.z = tvecs[i][2];

                tf2::Quaternion q;
                q.setRotation(tf2::Vector3(axis[0], axis[1], axis[2]), angle);

                ft_Cam1.transform.rotation.w = q.w();
                ft_Cam1.transform.rotation.x = q.x();
                ft_Cam1.transform.rotation.y = q.y();
                ft_Cam1.transform.rotation.z = q.z();

                ft_Cam1.fiducial_area = calcFiducialArea(markerCorners[i]);
                ft_Cam1.image_error = reprojectionError_Cam1[i];

                // Convert image_error (in pixels) to object_error (in meters)
                ft_Cam1.object_error =
                    (reprojectionError_Cam1[i] / dist(markerCorners[i][0], markerCorners[i][2])) *
                    (norm(tvecs[i]) / markerDimension);

                fta_Cam1.transforms.push_back(ft_Cam1);



                //display the rotation and translation matrix results
                putText(image_raw_last_frame_Cam1, "marker ID: " + doubleToString(markerIds[i]) , Point(markerCorners.at(i).at(1).x,(markerCorners.at(i).at(1).y -30)),FONT_HERSHEY_SIMPLEX, 0.8, Scalar(255,0,0), 2, LINE_8);

                putText(image_raw_last_frame_Cam1, "r(x): " + doubleToString(rvecs[i][0]* 180 / PI) , Point(markerCorners.at(i).at(1).x,markerCorners.at(i).at(1).y),FONT_HERSHEY_SIMPLEX, 0.8, Scalar(255,0,0), 2, LINE_8);
                putText(image_raw_last_frame_Cam1, "r(y): " + doubleToString(rvecs[i][1]* 180 / PI) , Point(markerCorners.at(i).at(1).x,(markerCorners.at(i).at(1).y + 30)),FONT_HERSHEY_SIMPLEX, 0.8, Scalar(255,0,0), 2, LINE_8);
                putText(image_raw_last_frame_Cam1, "r(z): " + doubleToString(rvecs[i][2]* 180 / PI) , Point(markerCorners.at(i).at(1).x,(markerCorners.at(i).at(1).y + 60)),FONT_HERSHEY_SIMPLEX, 0.8, Scalar(255,0,0), 2, LINE_8);

                putText(image_raw_last_frame_Cam1, "t(x): " + doubleToString(tvecs[i][0]) , Point(markerCorners.at(i).at(1).x,(markerCorners.at(i).at(1).y + 90)),FONT_HERSHEY_SIMPLEX, 0.8, Scalar(255,0,0), 2, LINE_8);
                putText(image_raw_last_frame_Cam1, "t(y): " + doubleToString(tvecs[i][1]) , Point(markerCorners.at(i).at(1).x,(markerCorners.at(i).at(1).y + 120)),FONT_HERSHEY_SIMPLEX, 0.8, Scalar(255,0,0), 2, LINE_8);
                putText(image_raw_last_frame_Cam1, "t(z): " + doubleToString(tvecs[i][2]) , Point(markerCorners.at(i).at(1).x,(markerCorners.at(i).at(1).y + 150)),FONT_HERSHEY_SIMPLEX, 0.8, Scalar(255,0,0), 2, LINE_8);

                markercounter++;

                detected_marker_Cam1.id = markerIds[i];
                detected_marker_Cam1.rotvec_x = rvecs[i][0];
                detected_marker_Cam1.rotvec_y = rvecs[i][1];
                detected_marker_Cam1.rotvec_z = rvecs[i][2];
                detected_marker_Cam1.transvec_x = tvecs[i][0];
                detected_marker_Cam1.transvec_y = tvecs[i][1];
                detected_marker_Cam1.transvec_z = tvecs[i][2];

                detected_marker_vec_Cam1.push_back(detected_marker_Cam1);

            }

            pose_pub->publish(fta_Cam1);

        }

        //publish the detected marker
        marker_msg_Cam1.header.stamp = ros::Time::now();
        marker_msg_Cam1.header.frame_id = reference_frame_Cam1;

        if(detected_marker_vec_Cam1.size() != 0)
        {

            marker_msg_Cam1.markers_detected = true;
            //marker_msg.markers_number = markercounter;

             for(int i = 0; i < detected_marker_vec_Cam1.size(); i++)
             {

                 marker_msg_Cam1.marker_id.push_back(detected_marker_vec_Cam1.at(i).id);
                 marker_msg_Cam1.marker_rotvec_x.push_back(detected_marker_vec_Cam1.at(i).rotvec_x);
                 marker_msg_Cam1.marker_rotvec_y.push_back(detected_marker_vec_Cam1.at(i).rotvec_y);
                 marker_msg_Cam1.marker_rotvec_z.push_back(detected_marker_vec_Cam1.at(i).rotvec_z);

                 marker_msg_Cam1.marker_transvec_x.push_back(detected_marker_vec_Cam1.at(i).transvec_x);
                 marker_msg_Cam1.marker_transvec_y.push_back(detected_marker_vec_Cam1.at(i).transvec_y);
                 marker_msg_Cam1.marker_transvec_z.push_back(detected_marker_vec_Cam1.at(i).transvec_z);

                cout<< "detected marker is ID: #"<< detected_marker_vec_Cam1.at(i).id <<endl;

                cout <<"r(x) is: " << detected_marker_vec_Cam1.at(i).rotvec_x * 180 / PI <<endl;
                cout <<"r(y) is: " << detected_marker_vec_Cam1.at(i).rotvec_y * 180 / PI <<endl;
                cout <<"r(z) is: " << detected_marker_vec_Cam1.at(i).rotvec_z * 180 / PI <<endl;

                cout<< "t(x) is " << detected_marker_vec_Cam1.at(i).transvec_x <<endl;
                cout<< "t(y) is " << detected_marker_vec_Cam1.at(i).transvec_y <<endl;
                cout<< "t(z) is " << detected_marker_vec_Cam1.at(i).transvec_z <<endl;

                ROS_ERROR("publish marker information to ROS: Cam1");
                 //marker_msg_pub.publish(marker_msg);
                 ROS_INFO("r(x) is %f, r(y) is %f, r(z) is %f", marker_msg_Cam1.marker_rotvec_x.at(i), marker_msg_Cam1.marker_rotvec_y.at(i), marker_msg_Cam1.marker_rotvec_z.at(i));
                 ROS_INFO("t(x) is %f, t(y) is %f, t(z) is %f", marker_msg_Cam1.marker_id.at(i), marker_msg_Cam1.marker_transvec_x.at(i), marker_msg_Cam1.marker_transvec_y.at(i), marker_msg_Cam1.marker_transvec_z.at(i));

             }

        }
        else
        {

             marker_msg_Cam1.markers_detected = false;
             //marker_msg.markers_number = 0;

        }

        //display the detected result
        imshow("result_Cam1", image_raw_last_frame_Cam1);
        waitKey(5);


        //display the rejected markers
        aruco::drawDetectedMarkers(image_rejected_Cam1, rejectedCandidates, noArray(), Scalar(100, 0, 255));
        imshow("rejected_Cam1", image_rejected_Cam1);
        waitKey(5);


}

static bool readCameraParameters(string filename, Mat &camMatrix, Mat &distCoeffs)
{
    FileStorage fs(filename, FileStorage::READ);
    if(!fs.isOpened())
    {
        return false;

    }
    else
    {
        fs["camMatrix"] >> camMatrix;
        //fs["distortion_coefficients"] >> distCoeffs;

        fs.release();
        return true;

    }


}

/*************************************************************************************************************************/
/*  Synchronize 2D frame time with computed candidate location from Point Cloud time, then complete the object detection */
/*  If detected, publish the ball message to ROS.                                                                        */
/*************************************************************************************************************************/
void BallDetector::Detect2D_Callback_Cam2(const sensor_msgs::ImageConstPtr& Image_msg)
{

    ros::Time  Detect2D_time = Image_msg->header.stamp;
         ROS_INFO("receive Cam 2");

   //using cv_bridge to transport the image from ROS to openCV cvMat, find closet time in frame time.
   cv_bridge::CvImagePtr cv_ptr;
   try
   {
      cv_ptr = cv_bridge::toCvCopy(Image_msg, sensor_msgs::image_encodings::BGR8);
//       start_time= ros::Time::now().toSec();
//       ROS_INFO("start time is %f", start_time);

   }
   catch (cv_bridge::Exception& e)
   {
       ROS_ERROR("cv_bridge exception: %s", e.what());
       return;

   }

   Mat &image_raw_Cam2 = cv_ptr->image;
   int width = image_raw_Cam2.cols;
   int height = image_raw_Cam2.rows;

   image_raw_Cam2.copyTo(image_raw_last_frame_Cam2);

   startWindowThread();
//        namedWindow("BallTracking2D:raw", WINDOW_AUTOSIZE);
//        imshow("BallTracking2D:raw", image_raw_last_frame_Cam2);
//        waitKey(5);

   Mat image_rejected_Cam2;
   image_raw_Cam2.copyTo(image_rejected_Cam2);

   //--------------------------------Start Section I  find marker---------------------------------//
   vector< int > markerIds;
   vector< vector< Point2f > > markerCorners, rejectedCandidates;

   aruco::DetectorParameters parameters;

   Ptr<cv::aruco::Dictionary> markerDictionary = aruco::getPredefinedDictionary(aruco::DICT_4X4_50);
           //Ptr<cv::aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(aruco::DICT_6X6_250);
   Ptr<aruco::DetectorParameters> detectorParams = aruco::DetectorParameters::create();

   int borderBits = 2;
   detectorParams->markerBorderBits = borderBits;
   //detectorParams->doCornerRefinement = false;

   //pamameters for pose estimation
   Mat camMatrix, distCoeffs;
   float cam_data [9] = {527.5694325104213, 0, 308.8603442411372, 0, 525.8171546326166, 256.4807142083769, 0, 0, 1};
   camMatrix = Mat(3, 3, CV_32F, cam_data);
   float dis_data [5] = {0.004529075343302832, -0.04052857948455536, 0.002827484841900654, -0.002446165900522254, 0};
   distCoeffs = Mat(1, 5, CV_32F, dis_data);

   vector< Vec3d > rvecs, tvecs;

   //parameter
   float markerDimension = 0.1;
   bool showRejected = true;//parser.has("r");
   bool estimatePose = true;//parser.has("c");

   // detect markers and estimate pose
   aruco::detectMarkers(image_raw_last_frame_Cam2, markerDictionary, markerCorners, markerIds, detectorParams, rejectedCandidates);

   vector<double> reprojectionError_Cam2;
   estimatePoseSingleMarkers(markerCorners, markerDimension, camMatrix, distCoeffs, rvecs, tvecs, reprojectionError_Cam2);

//            aruco::estimatePoseSingleMarkers(markerCorners, markerLength, camMatrix, distCoeffs, rvecs,
//                                             tvecs);

   fiducial_msgs::FiducialTransformArray fta_Cam2;
   fta_Cam2.header.stamp = Image_msg->header.stamp;
   fta_Cam2.header.frame_id = reference_frame_Cam2;
   fta_Cam2.image_seq = Image_msg->header.seq;

   int markercounter = 0;

   detected_marker_vec_Cam2.clear();
   marker_msg_Cam2.marker_id.clear();
   marker_msg_Cam2.marker_rotvec_x.clear();
   marker_msg_Cam2.marker_rotvec_y.clear();
   marker_msg_Cam2.marker_rotvec_z.clear();
   marker_msg_Cam2.marker_rotvec_x.clear();
   marker_msg_Cam2.marker_rotvec_y.clear();
   marker_msg_Cam2.marker_rotvec_z.clear();

   // draw results
   if(markerIds.size() > 0)
   {
       aruco::drawDetectedMarkers(image_raw_last_frame_Cam2, markerCorners, markerIds);

       for(int i = 0 ; i < markerIds.size(); i++)
       {
           aruco::drawAxis(image_raw_last_frame_Cam2, camMatrix, distCoeffs, rvecs[i], tvecs[i], markerDimension * 0.8f);

           double angle = norm(rvecs[i]);
           Vec3d axis = rvecs[i] / angle;
           ROS_WARN("Cam2: angle %f axis %f %f %f", angle, axis[0], axis[1], axis[2]);

           fiducial_msgs::FiducialTransform ft_Cam2;
           ft_Cam2.fiducial_id = markerIds[i];

           ft_Cam2.transform.translation.x = tvecs[i][0];
           ft_Cam2.transform.translation.y = tvecs[i][1];
           ft_Cam2.transform.translation.z = tvecs[i][2];

           tf2::Quaternion q;
           q.setRotation(tf2::Vector3(axis[0], axis[1], axis[2]), angle);

           ft_Cam2.transform.rotation.w = q.w();
           ft_Cam2.transform.rotation.x = q.x();
           ft_Cam2.transform.rotation.y = q.y();
           ft_Cam2.transform.rotation.z = q.z();

           ft_Cam2.fiducial_area = calcFiducialArea(markerCorners[i]);
           ft_Cam2.image_error = reprojectionError_Cam2[i];

           // Convert image_error (in pixels) to object_error (in meters)
           ft_Cam2.object_error =
               (reprojectionError_Cam2[i] / dist(markerCorners[i][0], markerCorners[i][2])) *
               (norm(tvecs[i]) / markerDimension);

           fta_Cam2.transforms.push_back(ft_Cam2);

           //display the rotation and translation matrix results
           putText(image_raw_last_frame_Cam2, "marker ID: " + doubleToString(markerIds[i]) , Point(markerCorners.at(i).at(1).x,(markerCorners.at(i).at(1).y -30)),FONT_HERSHEY_SIMPLEX, 0.8, Scalar(255,0,0), 2, LINE_8);

           putText(image_raw_last_frame_Cam2, "r(x): " + doubleToString(rvecs[i][0]* 180 / PI) , Point(markerCorners.at(i).at(1).x,markerCorners.at(i).at(1).y),FONT_HERSHEY_SIMPLEX, 0.8, Scalar(255,0,0), 2, LINE_8);
           putText(image_raw_last_frame_Cam2, "r(y): " + doubleToString(rvecs[i][1]* 180 / PI) , Point(markerCorners.at(i).at(1).x,(markerCorners.at(i).at(1).y + 30)),FONT_HERSHEY_SIMPLEX, 0.8, Scalar(255,0,0), 2, LINE_8);
           putText(image_raw_last_frame_Cam2, "r(z): " + doubleToString(rvecs[i][2]* 180 / PI) , Point(markerCorners.at(i).at(1).x,(markerCorners.at(i).at(1).y + 60)),FONT_HERSHEY_SIMPLEX, 0.8, Scalar(255,0,0), 2, LINE_8);

           putText(image_raw_last_frame_Cam2, "t(x): " + doubleToString(tvecs[i][0]) , Point(markerCorners.at(i).at(1).x,(markerCorners.at(i).at(1).y + 90)),FONT_HERSHEY_SIMPLEX, 0.8, Scalar(255,0,0), 2, LINE_8);
           putText(image_raw_last_frame_Cam2, "t(y): " + doubleToString(tvecs[i][1]) , Point(markerCorners.at(i).at(1).x,(markerCorners.at(i).at(1).y + 120)),FONT_HERSHEY_SIMPLEX, 0.8, Scalar(255,0,0), 2, LINE_8);
           putText(image_raw_last_frame_Cam2, "t(z): " + doubleToString(tvecs[i][2]) , Point(markerCorners.at(i).at(1).x,(markerCorners.at(i).at(1).y + 150)),FONT_HERSHEY_SIMPLEX, 0.8, Scalar(255,0,0), 2, LINE_8);

           markercounter++;

           detected_marker_Cam2.id = markerIds[i];
           detected_marker_Cam2.rotvec_x = rvecs[i][0];
           detected_marker_Cam2.rotvec_y = rvecs[i][1];
           detected_marker_Cam2.rotvec_z = rvecs[i][2];
           detected_marker_Cam2.transvec_x = tvecs[i][0];
           detected_marker_Cam2.transvec_y = tvecs[i][1];
           detected_marker_Cam2.transvec_z = tvecs[i][2];

           detected_marker_vec_Cam2.push_back(detected_marker_Cam2);

       }

       pose_pub->publish(fta_Cam2);

   }

   //publish the detected marker
   marker_msg_Cam2.header.stamp = ros::Time::now();
   marker_msg_Cam2.header.frame_id = reference_frame_Cam2;

   if(detected_marker_vec_Cam2.size() != 0)
   {

       marker_msg_Cam2.markers_detected = true;
       //marker_msg.markers_number = markercounter;

        for(int i = 0; i < detected_marker_vec_Cam2.size(); i++)
        {

            marker_msg_Cam2.marker_id.push_back(detected_marker_vec_Cam2.at(i).id);
            marker_msg_Cam2.marker_rotvec_x.push_back(detected_marker_vec_Cam2.at(i).rotvec_x);
            marker_msg_Cam2.marker_rotvec_y.push_back(detected_marker_vec_Cam2.at(i).rotvec_y);
            marker_msg_Cam2.marker_rotvec_z.push_back(detected_marker_vec_Cam2.at(i).rotvec_z);

            marker_msg_Cam2.marker_transvec_x.push_back(detected_marker_vec_Cam2.at(i).transvec_x);
            marker_msg_Cam2.marker_transvec_y.push_back(detected_marker_vec_Cam2.at(i).transvec_y);
            marker_msg_Cam2.marker_transvec_z.push_back(detected_marker_vec_Cam2.at(i).transvec_z);

           cout<< "detected marker is ID: #"<< detected_marker_vec_Cam2.at(i).id <<endl;

           cout <<"r(x) is: " << detected_marker_vec_Cam2.at(i).rotvec_x * 180 / PI <<endl;
           cout <<"r(y) is: " << detected_marker_vec_Cam2.at(i).rotvec_y * 180 / PI <<endl;
           cout <<"r(z) is: " << detected_marker_vec_Cam2.at(i).rotvec_z * 180 / PI <<endl;

           cout<< "t(x) is " << detected_marker_vec_Cam2.at(i).transvec_x <<endl;
           cout<< "t(y) is " << detected_marker_vec_Cam2.at(i).transvec_y <<endl;
           cout<< "t(z) is " << detected_marker_vec_Cam2.at(i).transvec_z <<endl;

           ROS_ERROR("publish marker information to ROS: Cam2");
            //marker_msg_pub.publish(marker_msg);
            ROS_INFO("r(x) is %f, r(y) is %f, r(z) is %f", marker_msg_Cam2.marker_rotvec_x.at(i), marker_msg_Cam2.marker_rotvec_y.at(i), marker_msg_Cam2.marker_rotvec_z.at(i));
            ROS_INFO("t(x) is %f, t(y) is %f, t(z) is %f", marker_msg_Cam2.marker_id.at(i), marker_msg_Cam2.marker_transvec_x.at(i), marker_msg_Cam2.marker_transvec_y.at(i), marker_msg_Cam2.marker_transvec_z.at(i));

        }

   }
   else
   {

        marker_msg_Cam2.markers_detected = false;
        //marker_msg.markers_number = 0;

   }

   //detection completed, then erase the candidate image
//        for(int n = 0; n < detected_marker_vec.size(); n++)
//        {
//            detected_marker_vec.erase(detected_marker_vec.begin()+n);

//        }

   //display the detected result
   imshow("result_Cam2", image_raw_last_frame_Cam2);
   waitKey(5);


   //display the rejected markers
//   aruco::drawDetectedMarkers(image_rejected, rejectedCandidates, noArray(), Scalar(100, 0, 255));
//   imshow("rejected", image_rejected);
//   waitKey(5);


}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "marker_detect");

    BallDetector ic;

    ros::Rate rate(10);

//    Mat camMatrix, distCoeffs;
//    string filename = "camera_TEST.yaml";


//    Mat mat = Mat::eye(Size(3,3), CV_64F);

//    FileStorage fs("camera_TEST.yaml", FileStorage::READ);
//    fs <<"camMatrix" << mat;
//    fs.release();

//    if(!fs.isOpened())
//    {
//        ROS_ERROR("Invalid camera file");

//        return false;

//    }
//    bool readOK = readCameraParameters(filename, camMatrix, distCoeffs);
//    if(!readOK)
//    {
//        ROS_ERROR("Invalid camera file");

//    }
//    else
//    {
//        cout<<"ok"<<endl;

//        cout << "camMatrix is: "<<endl;
//        for(int r = 0; r < camMatrix.rows; r++)
//        {
//            for(int c = 0; c < camMatrix.cols; c++)
//            {

//                cout << camMatrix.at<double>(r,c) << endl;


//            }
//        }

//        cout << "distCoeffs is: "<<endl;
//        for(int r = 0; r < distCoeffs.rows; r++)
//        {
//            for(int c = 0; c < distCoeffs.cols; c++)
//            {

//                //cout << distCoeffs.at(r,c) << endl;


//            }
//        }

//    }


    ROS_INFO("Marker detection program starting...");

    namedWindow("TrackBar", 10);

    createTrackbar("errorCorrection_test", "TrackBar", &errorCorrection_test, 100);
    createTrackbar("maxErroneousBits_test", "TrackBar", &maxErroneousBits_test, 100);
    createTrackbar("minOtsuStdDev_test", "TrackBar", &OtsuDev_test, 10);
    createTrackbar("AdapThreWinSizeMax_test", "TrackBar", &AdapThreWinSizeMax_test, 100);
    createTrackbar("AdapThreWinSizeMin_test", "TrackBar", &AdapThreWinSizeMin_test, 10);
    createTrackbar("AdapThreWinSizeStep_test", "TrackBar", &AdapThreWinSizeStep_test, 10);

    createTrackbar("perspectiveRemoveIgnoredMarginPerCell_test", "TrackBar", &perspectiveRemoveIgnoredMarginPerCell_test, 100);
    createTrackbar("perspectiveRemovePixelPerCell_test", "TrackBar", &AdapThreWinSizeStep_test, 20);


    while (ros::ok())
    {
        //ROS_INFO("Spinned once");
        ros::spinOnce();

        ic.marker_msg_pub.publish(ic.marker_msg_Cam1);

        ic.marker_msg_Cam1.marker_id.clear();
        ic.marker_msg_Cam1.marker_rotvec_x.clear();
        ic.marker_msg_Cam1.marker_rotvec_y.clear();
        ic.marker_msg_Cam1.marker_rotvec_z.clear();
        ic.marker_msg_Cam1.marker_transvec_x.clear();
        ic.marker_msg_Cam1.marker_transvec_y.clear();
        ic.marker_msg_Cam1.marker_transvec_z.clear();

        ic.marker_msg_pub.publish(ic.marker_msg_Cam2);

        ic.marker_msg_Cam2.marker_id.clear();
        ic.marker_msg_Cam2.marker_rotvec_x.clear();
        ic.marker_msg_Cam2.marker_rotvec_y.clear();
        ic.marker_msg_Cam2.marker_rotvec_z.clear();
        ic.marker_msg_Cam2.marker_transvec_x.clear();
        ic.marker_msg_Cam2.marker_transvec_y.clear();
        ic.marker_msg_Cam2.marker_transvec_z.clear();
        rate.sleep();

    }

    return 0;

}
