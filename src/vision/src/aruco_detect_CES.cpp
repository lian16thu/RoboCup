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

using namespace std;
using namespace cv;

double fx = 5284.88423;
int errorCorrection_test = 50;
int maxErroneousBits_test = 50;
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
  image_transport::Subscriber image_sub;

  image_transport::Publisher image_pub1;

  Mat image_raw;
  Mat image_raw_last_frame;

  ros::Time PCL_time;
  vector<double> frame_time;

public:
    ros::Publisher marker_msg_pub;

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
//    image_sub = it.subscribe("/image_raw", 1, &BallDetector::Detect2D_Callback, this);
    image_sub = it.subscribe("/camera2/rgb/image_rect_color", 1, &BallDetector::Detect2D_Callback, this);
//    image_sub = it.subscribe("/camera1/rgb/image_rect_color", 1, &BallDetector::Detect2D_Callback, this);

    marker_msg_pub = nh.advertise<vision::Markers>("/vision/marker", 10);

     //image_pub1 = it.advertise("/vision/detected_ball_3d", 1);

     //Initialization
     image_raw_last_frame = Mat::zeros(Height, Width, CV_8UC3);
     image_raw = Mat::zeros(Height, Width, CV_8UC3);

     ROS_INFO("class initialized..");

    //Initialize marker msg
     marker_msg.header.stamp = ros::Time::now();
     marker_msg.header.frame_id = reference_frame;
     marker_msg.markers_detected = false;
     //marker_msg.markers_number = 0; //all together, if more than two seen, placed from closer two to further
     //marker_msg.marker_id
     //marker_msg.marker_rotvec_x
     //marker_msg.marker_rotvec_y
     //marker_msg.marker_rotvec_z
     //marker_msg.marker_transvec_x
     //marker_msg.marker_transvec_y
     //marker_msg.marker_transvec_z

    }

   void Detect2D_Callback(const sensor_msgs::ImageConstPtr& Image_msg);
//   bool isRotationMatrix(Mat &R);
//   Vec3f rotationMatrixToEulerAngles(Mat &R);
   Mat rot2euler(const Mat & rotationMatrix);

};


//bool BallDetector::isRotationMatrix(Mat &R)
//{
//    // Checks if a matrix is a valid rotation matrix.
//    Mat Rt;
//    transpose(R, Rt);
//    Mat shouldBeIdentity = Rt * R;
//                        ROS_INFO("111");
//    Mat I = Mat::eye(3,3, shouldBeIdentity.type());
//                        ROS_INFO("333");

//    return  norm(I, shouldBeIdentity) < 1e-6;

//}


//Vec3f BallDetector::rotationMatrixToEulerAngles(Mat &R)
//{
//    // Calculates rotation matrix to euler angles
//    // The result is the same as MATLAB except the order
//    // of the euler angles ( x and z are swapped ).

//    assert(isRotationMatrix(R));

//    float sy = sqrt(R.at<double>(0,0) * R.at<double>(0,0) +  R.at<double>(1,0) * R.at<double>(1,0) );

//    bool singular = sy < 1e-6; // If

//    float x, y, z;
//    if (!singular)
//    {
//        x = atan2(R.at<double>(2,1) , R.at<double>(2,2));
//        y = atan2(-R.at<double>(2,0), sy);
//        z = atan2(R.at<double>(1,0), R.at<double>(0,0));
//    }
//    else
//    {
//        x = atan2(-R.at<double>(1,2), R.at<double>(1,1));
//        y = atan2(-R.at<double>(2,0), sy);
//        z = 0;
//    }
//    return Vec3f(x, y, z);



//}

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
        float cam_data [9] = {602.3554811285165, 0, 311.092750518436, 0, 599.8582889723741, 240.6942975137085, 0, 0, 1};
        camMatrix = Mat(3, 3, CV_32F, cam_data);
        float dis_data [5] = {0.1823309548724955, -0.2942736179629842, -0.004501611524250133, 0.005335803265015549, 0};
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
        aruco::detectMarkers(image_raw_last_frame, markerDictionary, markerCorners, markerIds, detectorParams, rejectedCandidates);

        aruco::estimatePoseSingleMarkers(markerCorners, markerDimension, camMatrix, distCoeffs, rvecs, tvecs);


        //        if(estimatePose && markerIds.size() > 0)
//            aruco::estimatePoseSingleMarkers(markerCorners, markerLength, camMatrix, distCoeffs, rvecs,
//                                             tvecs);



//        ROS_INFO("marker size is %d", markerIds.size());

        int markercounter = 0;

        detected_marker_vec.clear();
        marker_msg.marker_id.clear();
        marker_msg.marker_rotvec_x.clear();
        marker_msg.marker_rotvec_y.clear();
        marker_msg.marker_rotvec_z.clear();
        marker_msg.marker_rotvec_x.clear();
        marker_msg.marker_rotvec_y.clear();
        marker_msg.marker_rotvec_z.clear();

        Vec3d axis;
        double angle;

        Mat rotationMatrix = Mat::eye(4, 4, CV_64F);

        Mat RotMatrix_3x3(3, 3, CV_64F);

        // draw results
        if(markerIds.size() > 0)
        {
            aruco::drawDetectedMarkers(image_raw_last_frame, markerCorners, markerIds);

            //aruco::estimatePoseSingleMarkers(markerCorners, 0.05, camMatrix, distCoeffs, rvecs, tvecs);

            for(int i = 0 ; i < markerIds.size(); i++)
            {
//                cout<<"top_left corner(x) is "<<markerCorners.at(i).at(0).x<<endl;
//                cout<<"top_left corner(y) is "<<markerCorners.at(i).at(0).y<<endl;

                //aruco::drawAxis(image_raw_last_frame, camMatrix, distCoeffs, rvecs[i], tvecs[i], markerLength * 5);

                aruco::drawAxis(image_raw_last_frame, camMatrix, distCoeffs, rvecs[i], tvecs[i], markerDimension * 0.8f);

//                cout<< "detected marker is "<< markerIds[i]<<endl;
//                cout<< "r(x) is " << rvecs.at(i)[0] * 180 / PI <<endl;
//                cout<< "r(y) is " <<rvecs.at(i)[1] * 180 / PI <<endl;
//                cout<< "r(z) is " <<rvecs.at(i)[2] * 180 / PI <<endl;

//                cout<< "t(x) is " << tvecs.at(i)[0] <<endl;
//                cout<< "t(y) is " << tvecs.at(i)[1] <<endl;
//                cout<< "t(z) is " << tvecs.at(i)[2] <<endl;

//                angle = norm(rvecs.at(i));
//                ROS_INFO("angle is %f", angle);
//                axis[0] = rvecs.at(i)[0] * 180 / PI / angle;
//                ROS_INFO("222");
//                axis[1] = rvecs.at(i)[1] * 180 / PI / angle;
//                axis[2] = rvecs.at(i)[2] * 180 / PI / angle;
//                ROS_INFO("angle %f, axis %f %f %f", angle, axis[0], axis[1], axis[2]);

                Rodrigues(rvecs.at(i), RotMatrix_3x3);

                for(int i = 0; i < 4;i++)
                {
                    rotationMatrix.at<double>(i,0) = RotMatrix_3x3.at<double>(i,0);
                    rotationMatrix.at<double>(i,1) = RotMatrix_3x3.at<double>(i,1);
                    rotationMatrix.at<double>(i,2) = RotMatrix_3x3.at<double>(i,2);

                }

                rotationMatrix.at<double>(0,3) = tvecs.at(i)[0];
                rotationMatrix.at<double>(1,3) = tvecs.at(i)[1];
                rotationMatrix.at<double>(2,3) = tvecs.at(i)[2];

                rotationMatrix.at<double>(3,0) = 0;
                rotationMatrix.at<double>(3,1) = 0;
                rotationMatrix.at<double>(3,2) = 0;

                rotationMatrix.at<double>(3,3) = 1;


//                for(int i = 0; i < 4;i++)
//                {
//                    for(int j = 0; j < 4; j++)
//                    {
////                        cout <<"RotationMatrix(i"<< i<< ", j" <<j<<") is: " << rotationMatrix.at<double>(i,j) <<endl;

//                    }

//                }

                Mat euler_angle;
                euler_angle = rot2euler(rotationMatrix);
//                cout <<"Euler_angles at x-axis is: " << euler_angle.at<double>(0) * 180 / PI <<endl;
//                cout <<"Euler_angles at y-axis is: " << euler_angle.at<double>(1) * 180 / PI <<endl;
//                cout <<"Euler_angles at z-axis is: " << euler_angle.at<double>(2) * 180 / PI <<endl;

//                if(isRotationMatrix(rotationMatrix) == true)
//                {
//                    Vec3f Euler_angles;
//                    ROS_INFO("222");
//                    Euler_angles = rotationMatrixToEulerAngles(rotationMatrix);
//                    cout <<"Euler_angles at x-axis is: " << Euler_angles[0] <<endl;
//                    cout <<"Euler_angles at y-axis is: " << Euler_angles[1] <<endl;
//                    cout <<"Euler_angles at z-axis is: " << Euler_angles[2] <<endl;

//                }
//                else
//                {
//                    ROS_ERROR("The obtained rotation matrix is not valid!");
//                }

                //display the rotation and translation matrix results
                putText(image_raw_last_frame, "marker ID: " + doubleToString(markerIds[i]) , Point(markerCorners.at(i).at(1).x,(markerCorners.at(i).at(1).y -30)),FONT_HERSHEY_SIMPLEX, 0.8, Scalar(255,0,0), 2, LINE_8);

//                putText(image_raw_last_frame, "r(x): " + doubleToString(rvecs.at(i)[0]* 180 / PI) , Point(markerCorners.at(i).at(1).x,markerCorners.at(i).at(1).y),FONT_HERSHEY_SIMPLEX, 0.8, Scalar(255,0,0), 2, LINE_8);
//                putText(image_raw_last_frame, "r(y): " + doubleToString(rvecs.at(i)[1]* 180 / PI) , Point(markerCorners.at(i).at(1).x,(markerCorners.at(i).at(1).y + 30)),FONT_HERSHEY_SIMPLEX, 0.8, Scalar(255,0,0), 2, LINE_8);
//                putText(image_raw_last_frame, "r(z): " + doubleToString(rvecs.at(i)[2]* 180 / PI) , Point(markerCorners.at(i).at(1).x,(markerCorners.at(i).at(1).y + 60)),FONT_HERSHEY_SIMPLEX, 0.8, Scalar(255,0,0), 2, LINE_8);

                putText(image_raw_last_frame, "r(x): " + doubleToString(euler_angle.at<double>(0) * 180 / PI) , Point(markerCorners.at(i).at(1).x - 100,markerCorners.at(i).at(1).y),FONT_HERSHEY_SIMPLEX, 0.8, Scalar(255,0,0), 2, LINE_8);
                putText(image_raw_last_frame, "r(y): " + doubleToString(euler_angle.at<double>(1) * 180 / PI) , Point(markerCorners.at(i).at(1).x - 100,(markerCorners.at(i).at(1).y + 30)),FONT_HERSHEY_SIMPLEX, 0.8, Scalar(255,0,0), 2, LINE_8);
                putText(image_raw_last_frame, "r(z): " + doubleToString(euler_angle.at<double>(2) * 180 / PI) , Point(markerCorners.at(i).at(1).x - 100,(markerCorners.at(i).at(1).y + 60)),FONT_HERSHEY_SIMPLEX, 0.8, Scalar(255,0,0), 2, LINE_8);


                putText(image_raw_last_frame, "t(x): " + doubleToString(tvecs.at(i)[0]) , Point(markerCorners.at(i).at(1).x - 100,(markerCorners.at(i).at(1).y + 90)),FONT_HERSHEY_SIMPLEX, 0.8, Scalar(255,0,0), 2, LINE_8);
                putText(image_raw_last_frame, "t(y): " + doubleToString(tvecs.at(i)[1]) , Point(markerCorners.at(i).at(1).x - 100,(markerCorners.at(i).at(1).y + 120)),FONT_HERSHEY_SIMPLEX, 0.8, Scalar(255,0,0), 2, LINE_8);
                putText(image_raw_last_frame, "t(z): " + doubleToString(tvecs.at(i)[2]) , Point(markerCorners.at(i).at(1).x - 100,(markerCorners.at(i).at(1).y + 150)),FONT_HERSHEY_SIMPLEX, 0.8, Scalar(255,0,0), 2, LINE_8);

                markercounter++;

                detected_marker.id = markerIds[i];
                detected_marker.rotvec_x =euler_angle.at<double>(0);
                detected_marker.rotvec_y =euler_angle.at<double>(1);
                detected_marker.rotvec_z =euler_angle.at<double>(2);
                detected_marker.transvec_x = tvecs.at(i)[0];
                detected_marker.transvec_y = tvecs.at(i)[1];
                detected_marker.transvec_z = tvecs.at(i)[2];

                detected_marker_vec.push_back(detected_marker);

            }

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

                cout <<"Euler_angle at x-axis is: " << detected_marker_vec.at(i).rotvec_x * 180 / PI <<endl;
                cout <<"Euler_angle at y-axis is: " << detected_marker_vec.at(i).rotvec_y * 180 / PI <<endl;
                cout <<"Euler_angle at z-axis is: " << detected_marker_vec.at(i).rotvec_z * 180 / PI <<endl;

                cout<< "t(x) is " << detected_marker_vec.at(i).transvec_x <<endl;
                cout<< "t(y) is " << detected_marker_vec.at(i).transvec_y <<endl;
                cout<< "t(z) is " << detected_marker_vec.at(i).transvec_z <<endl;

                ROS_ERROR("publish marker information to ROS");
                 //marker_msg_pub.publish(marker_msg);
                 ROS_INFO("r(x) is %f, r(y) is %f, r(z) is %f", marker_msg.marker_rotvec_x.at(i), marker_msg.marker_rotvec_y.at(i), marker_msg.marker_rotvec_z.at(i));
                 ROS_INFO("t(x) is %f, t(y) is %f, t(z) is %f", marker_msg.marker_id.at(i), marker_msg.marker_transvec_x.at(i), marker_msg.marker_transvec_y.at(i), marker_msg.marker_transvec_z.at(i));

             }

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


        //display the rejected markers
        aruco::drawDetectedMarkers(image_rejected, rejectedCandidates, noArray(), Scalar(100, 0, 255));
        imshow("rejected", image_rejected);
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

int main(int argc, char** argv)
{
    ros::init(argc, argv, "marker_detect1");

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

        ic.marker_msg_pub.publish(ic.marker_msg);

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
