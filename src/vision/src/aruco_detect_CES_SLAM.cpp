#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include <cstdlib>
#include <math.h>

#include "sensor_msgs/Image.h"

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

double PI = 3.1415926;

int wrong_axis_pic_no;
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
  image_transport::Subscriber image_sub;

  image_transport::Publisher image_pub1;

  Mat image_raw;
  Mat image_raw_last_frame;

  ros::Time PCL_time;
  vector<double> frame_time; 

public:
  ros::Publisher marker_msg_pub;

  //SLAM
  ros::Publisher * pose_pub;

  vision::Markers marker_msg;

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

  marker detected_marker;

  vector<marker> detected_marker_vec;

  string reference_frame = "camera2_link";



public:
   BallDetector():it(nh)
   {
     // Subscrive to input video feed and publish output video feed

       image_sub = it.subscribe("/zed/left/image_rect_color", 1, &BallDetector::Detect2D_Callback, this);
      // image_sub = it.subscribe("/zed/right/image_rect_color", 1, &BallDetector::Detect2D_Callback, this);


   image_pub1 =it.advertise("/vision/aruco_result",10);

    marker_msg_pub = nh.advertise<vision::Markers>("/vision/marker", 10);

  pose_pub = new ros::Publisher(nh.advertise<fiducial_msgs::FiducialTransformArray>("/fiducial_transforms", 1));
//    pose_pub = new ros::Publisher(nh.advertise<fiducial_msgs::FiducialTransformArray>("/fiducial_transforms_camera", 1));

  
     //Initialization
     image_raw_last_frame = Mat::zeros(Height, Width, CV_8UC3);
     image_raw = Mat::zeros(Height, Width, CV_8UC3);

     ROS_INFO("class initialized..");

    //Initialize marker msg
     marker_msg.header.stamp = ros::Time::now();
     marker_msg.header.frame_id = reference_frame;
     marker_msg.markers_detected = false;

       wrong_axis_pic_no = 0;

    }

   void Detect2D_Callback(const sensor_msgs::ImageConstPtr& Image_msg);
   Mat rot2euler(const Mat & rotationMatrix);
   void estimatePoseSingleMarkers(const vector<vector<Point2f > >&corners,
                                  float markerLength,
                                  const cv::Mat &cameraMatrix,
                                  const cv::Mat &distCoeffs,
                                  vector<Vec3d>& rvecs, vector<Vec3d>& tvecs,
                                  vector<double>& reprojectionError);


};

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

/*************************************************************************************************************************/
/*  Synchronize 2D frame time with computed candidate location from Point Cloud time, then complete the object detection */
/*  If detected, publish the ball message to ROS.                                                                        */
/*************************************************************************************************************************/
void BallDetector::Detect2D_Callback(const sensor_msgs::ImageConstPtr& Image_msg)
{

         ros::Time  Detect2D_time = Image_msg->header.stamp;
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

        Mat &image_raw = cv_ptr->image;
        int width = image_raw.cols;
        int height = image_raw.rows;

        image_raw.copyTo(image_raw_last_frame);

        startWindowThread();
//        namedWindow("BallTracking2D:raw", WINDOW_AUTOSIZE);
//        imshow("BallTracking2D:raw", image_raw_last_frame);
//        waitKey(5);

        Mat image_rejected;
        image_raw.copyTo(image_rejected);

        //--------------------------------Start Section I  find marker---------------------------------//
        vector< int > markerIds;
        vector< vector< Point2f > > markerCorners, rejectedCandidates;

        aruco::DetectorParameters parameters;

        Ptr<cv::aruco::Dictionary> markerDictionary = aruco::getPredefinedDictionary(aruco::DICT_5X5_50);
                //Ptr<cv::aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(aruco::DICT_6X6_250);
        Ptr<aruco::DetectorParameters> detectorParams = aruco::DetectorParameters::create();

        int borderBits = 2;
        detectorParams->markerBorderBits = borderBits;
        //detectorParams->doCornerRefinement = false;
        detectorParams->adaptiveThreshWinSizeMin = 5;
        detectorParams->adaptiveThreshWinSizeMax = 35;
        detectorParams->adaptiveThreshWinSizeStep = 15;

        detectorParams -> maxErroneousBitsInBorderRate = double(maxErroneousBits_test/100);
        detectorParams -> errorCorrectionRate = double(errorCorrection_test/100);

        //pamameters for pose estimation
        Mat camMatrix, distCoeffs;
        float cam_data [9] = {700.178, 0.000000, 663.24, 0.000000, 700.178, 373.074, 0.000000, 0.000000, 1.000000};
        camMatrix = Mat(3, 3, CV_32F, cam_data);
        float dis_data [5] = {-0.173344, 0.0274179, 0.000000, 0.000000, 0.000000};
        distCoeffs = Mat(1, 5, CV_32F, dis_data);

//        Mat camMatrix, distCoeffs;
//        float cam_data [9] = {527.5694325104213, 0, 308.8603442411372, 0, 525.8171546326166, 256.4807142083769, 0, 0, 1};
//        camMatrix = Mat(3, 3, CV_32F, cam_data);
//        float dis_data [5] = {0.004529075343302832, -0.04052857948455536, 0.002827484841900654, -0.002446165900522254, 0};
//        distCoeffs = Mat(1, 5, CV_32F, dis_data);

        vector< Vec3d > rvecs, tvecs;

        //parameter
        float markerDimension = 0.2;
        bool showRejected = false;//parser.has("r");
        bool estimatePose = true;//parser.has("c");

        // detect markers and estimate pose
        aruco::detectMarkers(image_raw_last_frame, markerDictionary, markerCorners, markerIds, detectorParams, rejectedCandidates);

        vector <double>reprojectionError;
        estimatePoseSingleMarkers(markerCorners, markerDimension, camMatrix, distCoeffs, rvecs, tvecs, reprojectionError);

//            aruco::estimatePoseSingleMarkers(markerCorners, markerLength, camMatrix, distCoeffs, rvecs,
//                                             tvecs);

        fiducial_msgs::FiducialTransformArray fta;
        fta.header.stamp = Image_msg->header.stamp;
        fta.header.frame_id = reference_frame;
        fta.image_seq = Image_msg->header.seq;

        int markercounter = 0;

        detected_marker_vec.clear();
        marker_msg.marker_id.clear();
        marker_msg.marker_rotvec_x.clear();
        marker_msg.marker_rotvec_y.clear();
        marker_msg.marker_rotvec_z.clear();
        marker_msg.marker_transvec_x.clear();
        marker_msg.marker_transvec_y.clear();
        marker_msg.marker_transvec_z.clear();

        //-------------------------------------solve for flipping problem-------------------------------------
        Mat rotationMatrix = Mat::eye(4, 4, CV_64F);

        Mat RotMatrix_3x3(3, 3, CV_64F);

        // draw results
        if(markerIds.size() > 0)
        {
            aruco::drawDetectedMarkers(image_raw_last_frame, markerCorners, markerIds);

            for(int i = 0 ; i < markerIds.size(); i++)
            {
                aruco::drawAxis(image_raw_last_frame, camMatrix, distCoeffs, rvecs[i], tvecs[i], markerDimension * 0.8f);

                //-------------------------------------solve for flipping problem-------------------------------------
                Rodrigues(rvecs[i], RotMatrix_3x3);

                for(int i = 0; i < 4;i++)
                {
                    rotationMatrix.at<double>(i,0) = RotMatrix_3x3.at<double>(i,0);
                    rotationMatrix.at<double>(i,1) = RotMatrix_3x3.at<double>(i,1);
                    rotationMatrix.at<double>(i,2) = RotMatrix_3x3.at<double>(i,2);

                }

                rotationMatrix.at<double>(0,3) = tvecs[i][0];
                rotationMatrix.at<double>(1,3) = tvecs[i][1];
                rotationMatrix.at<double>(2,3) = tvecs[i][2];

                rotationMatrix.at<double>(3,0) = 0;
                rotationMatrix.at<double>(3,1) = 0;
                rotationMatrix.at<double>(3,2) = 0;

                rotationMatrix.at<double>(3,3) = 1;

                Mat euler_angle;
                euler_angle = rot2euler(rotationMatrix);

                //-------------------------------------solve for flipping problem-------------------------------------
                fiducial_msgs::FiducialTransform ft;
                tf2::Quaternion q;

//                if(euler_angle.at<double>(0) * 180 / PI > 0)
//                {
                    double angle = norm(rvecs[i]);
                    Vec3d axis = rvecs[i] / angle;
                    ROS_INFO("angle %f axis %f %f %f",
                             angle, axis[0], axis[1], axis[2]);

                    //fiducial_msgs::FiducialTransform ft;
                    ft.fiducial_id = markerIds[i];

                    ft.transform.translation.x = tvecs[i][0];
                    ft.transform.translation.y = tvecs[i][1];
                    ft.transform.translation.z = tvecs[i][2];

                    //tf2::Quaternion q;
                    q.setRotation(tf2::Vector3(axis[0], axis[1], axis[2]), angle);

                    ft.transform.rotation.w = q.w();
                    ft.transform.rotation.x = q.x();
                    ft.transform.rotation.y = q.y();
                    ft.transform.rotation.z = q.z();

                    ft.fiducial_area = calcFiducialArea(markerCorners[i]);
                    ft.image_error = reprojectionError[i];

                    // Convert image_error (in pixels) to object_error (in meters)
                    ft.object_error =
                        (reprojectionError[i] / dist(markerCorners[i][0], markerCorners[i][2])) *
                        (norm(tvecs[i]) / markerDimension);

                    fta.transforms.push_back(ft);

                    //display the rotation and translation matrix results
                    putText(image_raw_last_frame, "marker ID: " + doubleToString(markerIds[i]) , Point(markerCorners.at(i).at(1).x,(markerCorners.at(i).at(1).y -30)),FONT_HERSHEY_SIMPLEX, 0.6, Scalar(255,0,0), 2, LINE_8);

                    putText(image_raw_last_frame, "r(x): " + doubleToString(euler_angle.at<double>(0) * 180 / PI) , Point(markerCorners.at(i).at(1).x,markerCorners.at(i).at(1).y),FONT_HERSHEY_SIMPLEX, 0.6, Scalar(255,0,0), 2, LINE_8);
                    putText(image_raw_last_frame, "r(y): " + doubleToString(euler_angle.at<double>(1) * 180 / PI) , Point(markerCorners.at(i).at(1).x,(markerCorners.at(i).at(1).y + 30)),FONT_HERSHEY_SIMPLEX, 0.6, Scalar(255,0,0), 2, LINE_8);
                    putText(image_raw_last_frame, "r(z): " + doubleToString(euler_angle.at<double>(2) * 180 / PI) , Point(markerCorners.at(i).at(1).x,(markerCorners.at(i).at(1).y + 60)),FONT_HERSHEY_SIMPLEX, 0.6, Scalar(255,0,0), 2, LINE_8);

                    putText(image_raw_last_frame, "t(x): " + doubleToString(tvecs[i][0]) , Point(markerCorners.at(i).at(1).x,(markerCorners.at(i).at(1).y + 90)),FONT_HERSHEY_SIMPLEX, 0.6, Scalar(255,0,0), 2, LINE_8);
                    putText(image_raw_last_frame, "t(y): " + doubleToString(tvecs[i][1]) , Point(markerCorners.at(i).at(1).x,(markerCorners.at(i).at(1).y + 120)),FONT_HERSHEY_SIMPLEX, 0.6, Scalar(255,0,0), 2, LINE_8);
                    putText(image_raw_last_frame, "t(z): " + doubleToString(tvecs[i][2]) , Point(markerCorners.at(i).at(1).x,(markerCorners.at(i).at(1).y + 150)),FONT_HERSHEY_SIMPLEX, 0.6, Scalar(255,0,0), 2, LINE_8);

                    markercounter++;

                    detected_marker.id = markerIds[i];
                    detected_marker.rotvec_x = rvecs[i][0];
                    detected_marker.rotvec_y = rvecs[i][1];
                    detected_marker.rotvec_z = rvecs[i][2];
                    detected_marker.transvec_x = tvecs[i][0];
                    detected_marker.transvec_y = tvecs[i][1];
                    detected_marker.transvec_z = tvecs[i][2];

                    detected_marker_vec.push_back(detected_marker);

//                }
//                else
//                {
//                    //flipping problem
//                    ostringstream convert;
//                    string imageName = "wrong_axis_";
//                    convert << imageName << wrong_axis_pic_no << ".jpg";
//                    imwrite(convert.str(), image_raw_last_frame);
//                    waitKey(3);

//                    wrong_axis_pic_no++;

//                }

            }

            pose_pub->publish(fta);

        }

        //publish the detected marker
        marker_msg.header.stamp = ros::Time::now();
        marker_msg.header.frame_id = reference_frame;

        if(detected_marker_vec.size() != 0)
        {

            marker_msg.markers_detected = true;
            //marker_msg.markers_number = markercounter;

             for(int i = 0; i < detected_marker_vec.size(); i++)
             {

                 marker_msg.marker_id.push_back(detected_marker_vec.at(i).id);
                 marker_msg.marker_rotvec_x.push_back(detected_marker_vec.at(i).rotvec_x);
                 marker_msg.marker_rotvec_y.push_back(detected_marker_vec.at(i).rotvec_y);
                 marker_msg.marker_rotvec_z.push_back(detected_marker_vec.at(i).rotvec_z);

                 marker_msg.marker_transvec_x.push_back(detected_marker_vec.at(i).transvec_x);
                 marker_msg.marker_transvec_y.push_back(detected_marker_vec.at(i).transvec_y);
                 marker_msg.marker_transvec_z.push_back(detected_marker_vec.at(i).transvec_z);

                cout<< "detected marker is ID: #"<< detected_marker_vec.at(i).id <<endl;

                cout <<"Euler at r(x) is: " << detected_marker_vec.at(i).rotvec_x * 180 / PI <<endl;
                cout <<"Euler at r(y) is: " << detected_marker_vec.at(i).rotvec_y * 180 / PI <<endl;
                cout <<"Euler at r(z) is: " << detected_marker_vec.at(i).rotvec_z * 180 / PI <<endl;

                cout<< "t(x) is " << detected_marker_vec.at(i).transvec_x <<endl;
                cout<< "t(y) is " << detected_marker_vec.at(i).transvec_y <<endl;
                cout<< "t(z) is " << detected_marker_vec.at(i).transvec_z <<endl;

                ROS_ERROR("publish marker information to ROS");
                 //marker_msg_pub.publish(marker_msg);
                 ROS_INFO("r(x) is %f, r(y) is %f, r(z) is %f", marker_msg.marker_rotvec_x.at(i), marker_msg.marker_rotvec_y.at(i), marker_msg.marker_rotvec_z.at(i));
                 ROS_INFO("t(x) is %f, t(y) is %f, t(z) is %f", marker_msg.marker_id.at(i), marker_msg.marker_transvec_x.at(i), marker_msg.marker_transvec_y.at(i), marker_msg.marker_transvec_z.at(i));

             }

             marker_msg.markers_number = detected_marker_vec.size();

        }
        else
        {

             marker_msg.markers_detected = false;
             //marker_msg.markers_number = 0;

        }

        //detection completed, then erase the candidate image
//        for(int n = 0; n < detected_marker_vec.size(); n++)
//        {
//            detected_marker_vec.erase(detected_marker_vec.begin()+n);

//        }

        //display the detected result
        imshow("result", image_raw_last_frame);
        waitKey(5);

        sensor_msgs::ImagePtr aruco_result_image = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image_raw_last_frame).toImageMsg();

        image_pub1.publish(aruco_result_image);

        //display the rejected markers
//        aruco::drawDetectedMarkers(image_rejected, rejectedCandidates, noArray(), Scalar(100, 0, 255));
//        imshow("rejected", image_rejected);
//        waitKey(5);


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

    while (ros::ok())
    {
        //ROS_INFO("Spinned once");
        ros::spinOnce();

        ic.marker_msg_pub.publish(ic.marker_msg);
        ic.marker_msg.markers_detected = false;
        ic.marker_msg.marker_id.clear();
        ic.marker_msg.marker_rotvec_x.clear();
        ic.marker_msg.marker_rotvec_y.clear();
        ic.marker_msg.marker_rotvec_z.clear();
        ic.marker_msg.marker_transvec_x.clear();
                ic.marker_msg.marker_transvec_y.clear();
                        ic.marker_msg.marker_transvec_z.clear();

        rate.sleep();

    }

    return 0;

}
