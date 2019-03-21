#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

#include "vision/Landmarks.h"
#include "std_msgs/String.h"
#include "vision/ObjectOnImage.h"
#include "vision/DepthRequest.h"

#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/cache.h"
#include "sensor_msgs/Image.h"

#include "vision_define.h"
#include <opencv/cv.h>

#include <unistd.h>


double PI = 3.1415926;

//检测罚球点
#define areaLow_Thresh 30
#define areaHeight_Thresh 2000
#define tempWhite 255

////预处理
int low_h=40;
int low_s=57;
int low_v=0;
int high_h=61;
int high_s=255;
int high_v=255;

int Hough_Thresh=163; //在霍夫空间内的阈值，大于该值时判定为直线
int minLineLength=78;
int maxGapPointToLine=26;//直线归类
int whiteNumThresh = 8;
int prolongThresh=12;
int thinningIter=6;

int delta = 40;
int BlockSize = 2;

int zzzz=0;
int switch_screenshot = 0;

#define maxDisOfPenaltyToBoundary 50 //如果该矩形距离边界太近，则不考虑
#define Percent_highThresh 0.6//如果白色的面积覆盖率足够大则采用
#define ratioEdge 4 //矩形的长宽比

int flag_f;
int frame_buffer_size = 30;
double white_ratio_threshold = 0.4;
double fx = 5284.88423;

////python判断
int judge_cnn;

////收图
bool accept_pic=true;

////判断CNN是否返回
bool cnn_callback=false;

////是否call srv
bool call_srv=false;

using namespace std;
using namespace message_filters;
using namespace cv;

typedef struct k_and_b{
    double k;
    double b;

    bool operator< (const k_and_b a)const
    {
        if (k != a.k) return (k < a.k);
        else return (b < a.b);
    }
}k_AND_b;

typedef struct myPoint{//to use vector
    int x;
    int y;
    bool state;//used in judge whether the point is the real crossPoint.
}MyPoint;


typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, vision::ObjectOnImage> MySyncPolicy;
message_filters::Cache<sensor_msgs::Image> image_cache;


class FieldDetector
{

    ros::Time msg_time;
    ros::NodeHandle nh;
    image_transport::ImageTransport it;
    image_transport::Subscriber image_sub;
    ros::Publisher landmark_msg_pub;
    ros::ServiceClient depth_request_client;

    Mat image_raw;
    Mat image_raw_last_frame;
    Mat image_field;
    Mat image_hsv;
    Mat image_hue;
    Mat image_hue_hist;
    Mat image_field_last_frame;
    Mat image_no_penalty;

    ////预处理
    Mat image_field_gray;
    Mat image_field_bin;

    vector<Point2i> Penalty_point_vec;
    vector<Point2i> T_point_vec;
    vector<Point2i> L_point_vec;
    vector<Point2i> I_point_vec;
    vector<Point2i> unknown_point_vec;

    vision::Landmarks landmark_msg;
    vision::DepthRequest srv;

    float temp_range;
    float temp_bearing;

    Mat SE3x3;

    ////C++发图
    image_transport::Publisher imageroi_pub;

    ////C++收string
    ros::Subscriber judge_landmark_sub;

public:
    FieldDetector():it(nh)
    {
        // Subscrive to input video feed and publish output video feed
        image_cache.setCacheSize(frame_buffer_size);
        image_sub = it.subscribe("/zed/left/image_rect_color", 1, &FieldDetector::field_2D_Callback, this);

        depth_request_client = nh.serviceClient<vision::DepthRequest>("depth_request");

        landmark_msg_pub = nh.advertise<vision::Landmarks>("/vision/landmarks", 20);

        //Initialization
        image_raw_last_frame = Mat::zeros(Height, Width, CV_8UC3);
        image_raw = Mat::zeros(Height, Width, CV_8UC3);
        image_field = Mat::zeros(Height, Width, CV_8UC3);

        ////C++发图
        imageroi_pub=it.advertise("/landmark/imageroi_topic",1);

        ////C++收string
        judge_landmark_sub = nh.subscribe("judge_landmark", 10, &FieldDetector::judgeCallback,this);

        sensor_msgs::Image field_msg;
    }

    void field_2D_Callback(const sensor_msgs::ImageConstPtr& Image_msg);

    //校正坐标
    int coord(int x,int y);
    //计算矩形r中颜色为targetColor的像素所占的百分比
    double countPercent(Mat src,Rect r,int targetColor);
    //声明细化函数
    void cvThin(Mat& src, Mat& dst, int iterations);
    //求出点(x,y)到直线(k，b)的距离，坐标系的要求见函数定义处的注释
    double Distance(double k,double b,int x,int y);
    //计算两条直线的交点
    bool calculateCrossPoint(int * x, int * y, double k1, double b1, double k2, double b2, Mat image, int *flag);
    //在图像中画出斜率为k，截距为b的直线，注意，仅在此，以左上角为原点，宽度方向为x轴，高度方向为y轴
    void drawLine(double k,double b, Mat image);
    float range_correction(float range);
    float bearing_correction(Point2f center, float range, float bearing);
    void judgeCallback(const std_msgs::String::ConstPtr& stringmsg);
};

void  FieldDetector::judgeCallback(const std_msgs::String::ConstPtr& stringmsg)
{
    ROS_INFO("get the judge!");
    cnn_callback=true;
    judge_cnn=atoi(stringmsg->data.c_str());
}

//emit用
float FieldDetector::range_correction(float range)
{
    float range_corrected;

    if(range < 1.0)
    {
        //head angle is 65 degree
        range_corrected = range*0.934;
    }
    else
    {
        //head angle is 25 degree
        range_corrected = range*0.926;
    }

    cout<<"original range is "<<range<<endl;
    cout<<"corrected range is "<<range_corrected<<endl;

    return range_corrected;
}

//emit用
float FieldDetector::bearing_correction(Point2f center, float range, float bearing)
{
    float bearing_corrected;
    float point_x;

    bearing_corrected=bearing;
    return bearing_corrected;

}

void FieldDetector::field_2D_Callback(const sensor_msgs::ImageConstPtr& Image_msg)
{
    //usleep(1000);
    if(accept_pic) {
        //sleep(1000);
        accept_pic = false;
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(Image_msg);
        }
        catch (cv_bridge::Exception &e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        msg_time = Image_msg->header.stamp;

        Penalty_point_vec.clear();
        T_point_vec.clear();
        L_point_vec.clear();
        I_point_vec.clear();
        unknown_point_vec.clear();

        Mat &image_field = cv_ptr->image;
        int width = image_raw.cols;
        int height = image_raw.rows;

        image_field_last_frame = Mat::zeros(height, width, CV_8UC3);

//----------------------------------------Start Section I  预处理--------------------------------------------------//
        //得到一帧图像
        image_field.copyTo(image_field_last_frame);

        //中值滤波
        medianBlur(image_field_last_frame, image_field_last_frame, 3);

        cvtColor(image_field_last_frame, image_field_gray, COLOR_BGR2GRAY);
        adaptiveThreshold(image_field_gray, image_field_bin, 255, CV_ADAPTIVE_THRESH_MEAN_C, CV_THRESH_BINARY_INV,
                          BlockSize * 2 + 3, delta - 50);

        //RGB-->HSV
        cvtColor(image_field_last_frame, image_hsv, COLOR_BGR2HSV);
        //阈值范围
        inRange(image_hsv, Scalar(low_h, low_s, low_v), Scalar(high_h, high_s, high_v), image_hue);

        //草地边界
        int boundary[width];

        for (int j = 0; j < width; j++) {
            int i = 0;

            for (i = 0; i < height - 4; i++) {
                if (image_hue.at<unsigned char>(i, j) == 255) {
                    i++;
                    if (image_hue.at<unsigned char>(i, j) == 255) {
                        i++;
                        if (image_hue.at<unsigned char>(i, j) == 255) {
                            i++;
                            if (image_hue.at<unsigned char>(i, j) == 255) {
                                i++;
                                if (image_hue.at<unsigned char>(i, j) == 255) {
                                    boundary[j] = i;
                                    break;
                                }
                            }
                        }
                    }

                }
            }

            if (i >= height - 4)
                boundary[j] = height - 4;//如果没有找到边界点，则边界点的值为图片的高度。
        }
        //将场地内的线为白色，其余的颜色均置为黑（0）
        Mat image_line1, image_line2, image_line;
        image_line1 = Mat::zeros(height, width, CV_8UC1);
        image_line2 = Mat::zeros(height, width, CV_8UC1);
        image_line = Mat::zeros(height, width, CV_8UC1);

//        bitwise_not(image_hue, image_line1);
//        bitwise_not(image_field_bin, image_line2);
        for (int j = 0; j < width; j++) {
            for (int i = boundary[j] + 1; i < height; i++) {

                if (image_hue.at<unsigned char>(i, j) == 0)
                    image_line1.at<unsigned char>(i, j) = 255;

                if (image_field_bin.at<unsigned char>(i, j) == 0)
                    image_line2.at<unsigned char>(i, j) = 255;
            }
        }

        image_line = image_line1 | image_line2;

        Mat clean_pic = image_line.clone();
        imshow("clean_pic", clean_pic);
        waitKey(3);

        //----------------------------------------Section II 检测罚球点 ----------------------------------------------------//
        vector<vector<Point> > contours;
        Mat temp_pic = image_line.clone();
        findContours(temp_pic, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);//查找轮廓，对应连通域
        vector<double> AreaOfConnectedDomain; //记录所有连通域的面积
        for (size_t i = 0; i < contours.size(); i++) {
            double temp_area = contourArea(contours[i]);// 计算所有连通域的面积
            AreaOfConnectedDomain.push_back(temp_area);
        }

        //----------------------------------------Section III  thinning算法，霍夫变换----------------------------------------//
        vector<Vec4i> mylines;
        Mat pThin = image_line.clone();
        cvThin(image_line, pThin, thinningIter);
        HoughLinesP(image_line, mylines, 1, CV_PI / 180, Hough_Thresh, minLineLength, 10);
        //imshow("FieldTracking2D_1:thinning", pThin);
        //waitKey(3);

        //----------------------------------------Section  V  对由霍夫变换得到的直线进行分类，找出归类后的有效直线-----------------//
        //----------[可调参数]判断点与直线距离Distance的结果，此处为 15---------------------//
        //-------根据两点式求解k，b: k = (y2-y1)/(x2-x1); b = y1 - (y2-y1)/(x2-x1)*x1;-----//
        //-------y=kx+b --> kx-y+b=0 --> A=k,B=-1,C=b;  d=fab(Ax0+By0+C)/pow(A^2+B^2)-----//
        vector<k_AND_b> k_b;//记录场地中直线的k、b信息
        k_AND_b temp_k_b;
        for (int i = 0; i < mylines.size(); i++) {
            Vec4i l = mylines[i];//x1 = l[0],y1 = l[1],x2 = l[2];y2 = l[3]//以左上角为原点，宽度方向为x轴，高度方向为y轴!!!
            temp_k_b.k = (l[3] - l[1]) * 1.0 / (l[2] - l[0]);
            temp_k_b.b = l[1] - (l[3] - l[1]) * 1.0 / (l[2] - l[0]) * 1.0 * l[0];
            if (k_b.size() == 0) {
                k_b.push_back(temp_k_b);
            } else {
                int j = 0;
                for (j = 0; j < k_b.size(); j++) {
                    //如果两个点距离当前直线的距离均小于25，则归并,否则新建曲线
                    if ((fabs(Distance(k_b[j].k, k_b[j].b, l[0], l[1])) < maxGapPointToLine) &&
                        (fabs(Distance(k_b[j].k, k_b[j].b, l[2], l[3])) < maxGapPointToLine)) {
                        //ROS_INFO("  similar!j = %d  count_kb = %d",j,k_b.size());
                        k_b[j].k = (k_b[j].k + temp_k_b.k) / 2;
                        k_b[j].b = (k_b[j].b + temp_k_b.b) / 2;
                        break;
                    }
                }
                //如果没有找到同类直线；
                if (j == k_b.size()) {
                    k_b.push_back(temp_k_b);
                }
            }
        }

        //-----------------------------------------罚球点检测第二部分--------------------------------------------------------//
        bool flagPenaltyToLine = true;
        int PenaltyNo = -1;
        int tempHeight = 520;
        for (size_t i = 0; i < contours.size(); i++) {
            flagPenaltyToLine = true;
            if ((AreaOfConnectedDomain[i] > areaLow_Thresh) && (AreaOfConnectedDomain[i] < areaHeight_Thresh)) {
                Rect r = boundingRect(contours[i]);
                if (fabs(boundary[r.x] - r.y) < maxDisOfPenaltyToBoundary) {//如果该矩形距离边界太近，则不考虑
                    continue;
                }
                if ((r.width * 1.0 / r.height > ratioEdge) || (r.height * 1.0 / r.width > ratioEdge)) {//如果矩形的长宽比太大，则不考虑
                    continue;
                }
                for (int i = 0; i < k_b.size(); i++) {//判断罚球点与场线的距离
                    if (fabs(Distance(k_b[i].k, k_b[i].b, (r.x + r.width / 2), (r.y + r.height / 2))) < 20) {
                        flagPenaltyToLine = false;
                        break;
                    }
                }
                if (flagPenaltyToLine == false)
                    continue;
                if (countPercent(image_line, r, tempWhite) > Percent_highThresh) {//如果白色的覆盖率
                    //画出所有可能的罚球点
                    //rectangle(image_line, r, Scalar(100), 5);
                    if (r.y < tempHeight) {//找到最高的罚球点
                        tempHeight = r.y;
                        PenaltyNo = i;
                    }
                }
            }
        }

        image_no_penalty = image_line.clone();
        if (PenaltyNo != -1) {
            Rect r = boundingRect(contours[PenaltyNo]);
            rectangle(image_line, r, Scalar(100), 5);
            rectangle(image_no_penalty, r, Scalar(0), -1);
            Point2i Penalty_point = Point2i(r.y + r.height / 2, r.x + r.width / 2);
            Penalty_point_vec.push_back(Penalty_point);
            ROS_INFO("Penalty_point is %d, %d", Penalty_point.y, Penalty_point.x);
            call_srv = true;
        }

        imshow("image_no_penalty", image_no_penalty);
        waitKey(3);

        //-----------------------Section VI  计算出斜率相差较大的直线之间的，且位于场地内的交点，并在图中画出来-----------------------//
        int x, y;   //暂存计算出的交点坐标,以左上角为原点，宽度方向为y轴，高度方向为x轴
        vector<MyPoint> crossPoint;     //交点
        MyPoint temp_crossPoint;

        CvFont font;
        double hScale = 1;
        double vScale = 1;
        int lineWidth = 2;    //相当于写字的线条
        cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX | CV_FONT_ITALIC, hScale, vScale, 0, lineWidth);    //初始化字体

        IplImage src(image_line);
        char filename[256];
        int count_white = 0;
        int radius_x = 25;
        int radius_y = 25;
        bool state;
        bool send_image = false;

        int X_num = 0;
        int T_num = 0;
        int I_num = 0;
        int L_num = 0;

        imshow("clean_pic", clean_pic);
        waitKey(3);

        ROS_ERROR("startsstartsstarts");
        for (int i = 0; i < k_b.size(); i++) {

            for (int j = i + 1; j < k_b.size(); j++) {
                int flag_prolong[4] = {0, 0, 0, 0};
                state = calculateCrossPoint(&x, &y, k_b[i].k, k_b[i].b, k_b[j].k, k_b[j].b, image_line, flag_prolong);

                if (state == true && boundary[y] < x) {   //当交点位于图片中时
                    if ((x - radius_x > 0) && (x + radius_x < 480) && (y - radius_y > 0) && (y + radius_y < 640)) {
                        Mat imageroi = clean_pic(Range(x - radius_x, x + radius_x), Range(y - radius_y, y + radius_y));
                        count_white = 0;
                        for (int j = radius_y - 10; j < radius_y + 10; j++)
                            for (int i = radius_x - 10; i < radius_x + 10; i++)
                                if (imageroi.at<unsigned char>(i, j) == 255)
                                    count_white++;
                        if (count_white > 30) {   //imageroi中心区域白色够多
                            count_white = 0;
                            for (int j = 0; j < 2 * radius_y; j++)
                                for (int i = 0; i < 2 * radius_x; i++)
                                    if (imageroi.at<unsigned char>(i, j) == 255)
                                        count_white++;
                            if ((count_white > 200) &&
                                (count_white < (2 * radius_x) * (2 * radius_y) - 600)) {   //imageroi白色总数在一定范围内
                                if (switch_screenshot == 1) {
                                    ////截图
//                                    sprintf(filename, "intersection_%d.png", zzzz++);
//                                    imwrite(filename, imageroi);
                                }
                                sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "mono8",
                                                                               imageroi).toImageMsg();
                                imageroi_pub.publish(msg);
                                ROS_INFO("send message!!!");
                                send_image = true;
                            }
                        }
                    }

                    int count_pause = 0;
                    while (send_image && !cnn_callback) {
                        ros::spinOnce();
                        count_pause++;
                        if (count_pause > 1000)
                            break;
                    }

                    if (send_image) {
                        send_image = false;
                        cnn_callback = false;

                        temp_crossPoint.x = x;
                        temp_crossPoint.y = y;
                        crossPoint.push_back(temp_crossPoint);

                        //ROS_INFO("crosspoint is %d, %d", y, x);

                        call_srv = true;

                        ////judge
                        ROS_WARN("judge is:%d", judge_cnn);


                        if (judge_cnn == 0 &&
                            (flag_prolong[0] + flag_prolong[1] + flag_prolong[2] + flag_prolong[3] == 2) &&
                            (flag_prolong[0] * flag_prolong[1] == 1 || flag_prolong[2] * flag_prolong[3] == 1)) {
//                        if (judge_cnn == 0){
//                            drawLine(k_b[i].k,k_b[i].b,image_line);
//                            drawLine(k_b[j].k,k_b[j].b,image_line);
//                            cvPutText(&src, "*II", cvPoint(y, x - 50), &font, CV_RGB(255, 255, 255));//在图片中输出字符
//                            I_point_vec.push_back(Point2i(y, x));
                            ROS_WARN(" judge is: II");
                        } else if (judge_cnn == 1 &&
                                   (flag_prolong[0] + flag_prolong[1] + flag_prolong[2] + flag_prolong[3] == 2) &&
                                   flag_prolong[0] * flag_prolong[1] == 0 && flag_prolong[2] * flag_prolong[3] == 0) {
//                        else if (judge_cnn == 1) {
                            L_num++;
                            if (L_num <= 3) {
                                drawLine(k_b[i].k, k_b[i].b, image_line);
                                drawLine(k_b[j].k, k_b[j].b, image_line);
                                circle(image_line, Point(y, x), 30, Scalar(255, 0, 0), 2);
//                            cvPutText(&src, "*LL", cvPoint(y, x - 50), &font, CV_RGB(255, 255, 255));//在图片中输出字符
                                cvPutText(&src, "*LL", cvPoint(y, x - 50), &font, Scalar(50));//在图片中输出字符
                                L_point_vec.push_back(Point2i(y, x));
                                ROS_WARN(" judge is: LLLL");
                            }
                        }
//                      else if (judge_cnn == 2&&(flag_prolong[0] + flag_prolong[1] + flag_prolong[2] + flag_prolong[3] == 4)) {
//                        else if (judge_cnn == 2) {
//                            X_num++;
//                            if(X_num<=1&&(flag_prolong[0] + flag_prolong[1] + flag_prolong[2] + flag_prolong[3] == 4)){
//                                drawLine(k_b[i].k,k_b[i].b,image_line);
//                                drawLine(k_b[j].k,k_b[j].b,image_line);
//                                circle(image_line, Point(y, x), 30, Scalar(255, 0, 0), 2);
//                                cvPutText(&src, "*XX", cvPoint(y, x - 50), &font, Scalar(50));//在图片中输出字符
//                                L_point_vec.push_back(Point2i(y, x));
//                                ROS_WARN(" judge is: XX");
//                            }
//                        }
                        else if (judge_cnn == 3 &&
                                 (flag_prolong[0] + flag_prolong[1] + flag_prolong[2] + flag_prolong[3] == 3)) {
//                        else if (judge_cnn == 3) {
                            T_num++;
                            if (T_num <= 2) {
                                drawLine(k_b[i].k, k_b[i].b, image_line);
                                drawLine(k_b[j].k, k_b[j].b, image_line);
                                circle(image_line, Point(y, x), 30, Scalar(255, 0, 0), 2);
//                            cvPutText(&src, "*TT", cvPoint(y, x - 50), &font, CV_RGB(255, 255, 255));//在图片中输出字符
                                cvPutText(&src, "*TT", cvPoint(y, x - 50), &font, Scalar(50));//在图片中输出字符
                                T_point_vec.push_back(Point2i(y, x));
                                ROS_WARN(" judge is: TT");
                            }
                        }

                        ////guess
                        ROS_WARN("guess is:%d", flag_prolong[0] + flag_prolong[1] + flag_prolong[2] + flag_prolong[3]);
//                        if (flag_prolong[0] + flag_prolong[1] + flag_prolong[2] + flag_prolong[3] == 3) {
//                            cvPutText(&src, "TTTT", cvPoint(y, x + 3), &font, CV_RGB(255, 255, 255));//在图片中输出字符
//                            //T_point_vec.push_back(Point2i(y, x));
//                            ROS_WARN(" guess is: TTTT");
//                        }
//                        else if (flag_prolong[0] + flag_prolong[1] + flag_prolong[2] + flag_prolong[3] == 2) {
//                            if (flag_prolong[0] * flag_prolong[1] == 1 || flag_prolong[2] * flag_prolong[3] == 1) {
//                                cvPutText(&src, "IIII", cvPoint(y, x + 3), &font, CV_RGB(255, 255, 255));//在图片中输出字符
//                                //I_point_vec.push_back(Point2i(y, x));
//                                ROS_WARN(" guess is: IIII");
//                            } else {
//                                cvPutText(&src, "LLLL", cvPoint(y, x + 3), &font, CV_RGB(255, 255, 255));//在图片中输出字符
//                                //L_point_vec.push_back(Point2i(y, x));
//                                ROS_WARN(" guess is: LLLL");
//                            }
//                        } else {
//                            //                    cvPutText(&src,"unknown",cvPoint(y, x+3),&font,CV_RGB(255,255,255));//在图片中输出字符
//                            //unknown_point_vec.push_back(Point2i(y, x));
//                            ROS_WARN(" guess is: unknown");
//                        }
                    }
                }
            }
        }
        ROS_ERROR("endsendsendsendsends");

        ROS_INFO("crosspoint_num = %d", crossPoint.size());

        imshow("crosspoint", image_line);
        waitKey(3);

        //publish message to ROS
        //vision::DepthRequest srv;
        landmark_msg.header.stamp = msg_time;

        if (call_srv) {
            call_srv = false;
            int count = 0;
            for (int i = 0; i < Penalty_point_vec.size(); i++) {
                temp_range = 0;
                temp_bearing = 0;

                srv.request.u = Penalty_point_vec.at(i).y;
                srv.request.v = Penalty_point_vec.at(i).x;

                if (depth_request_client.call(srv)) {
                    temp_range = srv.response.depth;
                    landmark_msg.landmark_range[count] = range_correction(temp_range);

                    //calculate the bearing in radians
                    if (Penalty_point_vec.at(i).x > 320) {
                        //bearing direction is positive
                        temp_bearing = atan((Penalty_point_vec.at(i).x - 320) * 7.4 / fx);
                    } else {
                        //bearing direction is negative
                        temp_bearing = -atan((320 - Penalty_point_vec.at(i).x) * 7.4 / fx);
                    }


                    landmark_msg.landmark_type[count] = 5;
                    landmark_msg.landmark_bearing[count] = bearing_correction(Penalty_point_vec.at(i), temp_range,
                                                                              temp_bearing);
                    landmark_msg.landmark_confidence[count] = 1.0;

                    ROS_INFO("Penalty point at %d, bearing is %f, range is %f", count + 1,
                             landmark_msg.landmark_bearing[count] * 180 / PI, landmark_msg.landmark_range[count]);

                    if (count < 19)
                        count++;
                } else {
                    ROS_ERROR("Failed to call service DepthRequest --Penalty");
                }
            }

            for (int i = 0; i < I_point_vec.size(); i++) {
                temp_range = 0;
                temp_bearing = 0;

                srv.request.u = I_point_vec.at(i).y;
                srv.request.v = I_point_vec.at(i).x;
                if (depth_request_client.call(srv)) {
                    temp_range = srv.response.depth;
                    landmark_msg.landmark_range[count] = range_correction(temp_range);

                    //calculate the bearing in radians
                    if (I_point_vec.at(i).x > 320) {
                        //bearing direction is positive
                        temp_bearing = atan((I_point_vec.at(i).x - 320) * 7.4 / fx);
                    } else {
                        //bearing direction is negative
                        temp_bearing = -atan((320 - I_point_vec.at(i).x) * 7.4 / fx);
                    }


                    landmark_msg.landmark_type[count] = 4;
                    landmark_msg.landmark_bearing[count] = bearing_correction(I_point_vec.at(i), temp_range,
                                                                              temp_bearing);
                    landmark_msg.landmark_confidence[count] = 0.8;

                    ROS_INFO("I point at %d, bearing is %f, range is %f", count + 1,
                             landmark_msg.landmark_bearing[count] * 180 / PI, landmark_msg.landmark_range[count]);

                    if (count < 19)
                        count++;
                } else {
                    ROS_ERROR("Failed to call service DepthRequest --X");
                }
            }


            for (int i = 0; i < T_point_vec.size(); i++) {
                temp_range = 0;
                temp_bearing = 0;

                srv.request.u = T_point_vec.at(i).y;
                srv.request.v = T_point_vec.at(i).x;
                if (depth_request_client.call(srv)) {
                    temp_range = srv.response.depth;
                    landmark_msg.landmark_range[count] = range_correction(temp_range);

                    //calculate the bearing in radians
                    if (T_point_vec.at(i).x > 320) {
                        //bearing direction is positive
                        temp_bearing = atan((T_point_vec.at(i).x - 320) * 7.4 / fx);
                    } else {
                        //bearing direction is negative
                        temp_bearing = -atan((320 - T_point_vec.at(i).x) * 7.4 / fx);
                    }


                    landmark_msg.landmark_type[count] = 2;
                    landmark_msg.landmark_bearing[count] = bearing_correction(T_point_vec.at(i), temp_range,
                                                                              temp_bearing);
                    landmark_msg.landmark_confidence[count] = 0.8;

                    ROS_INFO("T point at %d, bearing is %f, range is %f", count + 1,
                             landmark_msg.landmark_bearing[count] * 180 / PI, landmark_msg.landmark_range[count]);

                    if (count < 19)
                        count++;
                } else {
                    ROS_ERROR("Failed to call service DepthRequest --T");
                }
            }

            for (int i = 0; i < L_point_vec.size(); i++) {
                temp_range = 0;
                temp_bearing = 0;

                srv.request.u = L_point_vec.at(i).y;
                srv.request.v = L_point_vec.at(i).x;
                if (depth_request_client.call(srv)) {
                    temp_range = srv.response.depth;

                    landmark_msg.landmark_range[count] = range_correction(temp_range);

                    //calculate the bearing in radians
                    if (L_point_vec.at(i).x > 320) {
                        //bearing direction is positive
                        temp_bearing = atan((L_point_vec.at(i).x - 320) * 7.4 / fx);
                    } else {
                        //bearing direction is negative
                        temp_bearing = -atan((320 - L_point_vec.at(i).x) * 7.4 / fx);
                    }


                    landmark_msg.landmark_type[count] = 3;
                    landmark_msg.landmark_bearing[count] = bearing_correction(L_point_vec.at(i), temp_range,
                                                                              temp_bearing);
                    landmark_msg.landmark_confidence[count] = 0.8;

                    ROS_INFO("L point at %d, bearing is %f, range is %f", count + 1,
                             landmark_msg.landmark_bearing[count] * 180 / PI, landmark_msg.landmark_range[count]);

                    if (count < 19)
                        count++;
                } else {
                    ROS_ERROR("Failed to call service DepthRequest --L");
                }

            }

            landmark_msg.landmark_number = count + 1;//need to change later
            landmark_msg_pub.publish(landmark_msg);
            accept_pic = true;
        }
    }
}

bool FieldDetector::calculateCrossPoint(int *x,int *y,double k1,double b1,double k2,double b2, Mat image,int *flag_prolong)
{
    int count_nonBlackPoint = 0;//用于判断点是否在绿色场地内，
    int WhitePointNum=0;
    double temp_x,temp_y1,temp_y2;
    temp_x = (b2 - b1)/(k1 - k2);
    temp_y1 = k1*temp_x+b1;
    temp_y2 = k2*temp_x+b2;
    (*x) = (int)((temp_y1+temp_y2)/2);//此处做了坐标变换
    (*y) = (int)temp_x;//此处做了坐标变换
    if ((*x) > -1 && (*x) < Height && (*y) > -1 && (*y) < Width)
    {//该点在图片内
        for (int i = (*x)-20;i < (*x)+20;i ++)
            for (int j = (*y) - 20;j < (*y) + 20;j ++)
            {
                if (image.at<unsigned char>(i, j) == 255)
                    count_nonBlackPoint ++;
            }
        if (count_nonBlackPoint > 40){

            for (int j = (*y)+1;j <= (*y)+prolongThresh;j ++)
            {
                int i=k1*j+b1;
                //circle(image, Point(j, i), 1, CV_RGB(255,255,255), 7);
                if (image_no_penalty.at<unsigned char>(i, j) == 255)
                    WhitePointNum ++;
            }
            if(WhitePointNum>whiteNumThresh)
                flag_prolong[0]=1;
            WhitePointNum = 0;

            for (int j = (*y)-prolongThresh;j < (*y);j ++)
            {
                int i=k1*j+b1;
                //circle(image, Point(j, i), 1, CV_RGB(255,255,255), 7);
                if (image_no_penalty.at<unsigned char>(i, j) == 255)
                    WhitePointNum ++;
            }
            if(WhitePointNum>whiteNumThresh)
                flag_prolong[1]=1;
            WhitePointNum = 0;

            for (int j = (*y)+1;j <= (*y)+prolongThresh;j ++)
            {
                int i=k2*j+b2;
                //circle(image, Point(j, i), 1, CV_RGB(255,255,255), 7);
                if (image_no_penalty.at<unsigned char>(i, j) == 255)
                    WhitePointNum ++;
            }
            if(WhitePointNum>whiteNumThresh)
                flag_prolong[2]=1;
            WhitePointNum = 0;

            for (int j = (*y)-prolongThresh;j < (*y);j ++)
            {
                int i=k2*j+b2;
                //circle(image, Point(j, i), 1, CV_RGB(255,255,255), 7);
                if (image_no_penalty.at<unsigned char>(i, j) == 255)
                    WhitePointNum ++;
            }
            if(WhitePointNum>whiteNumThresh)
                flag_prolong[3]=1;

            //ROS_ERROR("num of flags = %d",flag_prolong[0]+flag_prolong[1]+flag_prolong[2]+flag_prolong[3]);

            return true;
        }
        else {
            return false;
        }
    }
        /*if ((*y) > -1 && (*y) < Width && (*x) > -1 && (*x) < boundary[(*y)]){
        return true;
        }*/
    else {
        return false;
    }
}

void FieldDetector::drawLine(double k,double b, Mat image)
{
//在图像中画出斜率为k，截距为b的直线，注意，仅在此，以左上角为原点，宽度方向为x轴，高度方向为y轴
    int h;
    for (int i = 0;i < Width;i ++){
        h = (int)(k * i+b);
        if (h >=0 && h < Height)image.data[coord(h,Height-1)*Width+coord(i,Width-1)] = 200;
    }
    return;
}

double FieldDetector::Distance(double k,double b,int x,int y){//求出点(x,y)到直线(k，b)的距离,以左上角为原点，宽度方向为x轴，高度方向为y轴!!!
//--> y=kx+b --> kx-y+b=0 --> A=k,B=-1,C=b; --> d=fab(Ax0+By0+C)/pow(A^2+B^2)
    //return ((fabs(k*x-y+b))/pow(k*k+1,0.5));
    return ((k*x-y+b)/pow(k*k+1,0.5));//若y>k*x+b，则点（x,y）在直线下方，所以，返回为正时在点（x,y）在直线上方！
}

void FieldDetector::cvThin(Mat& src, Mat& dst, int iterations)
//thinning算法
{
    int n = 0,i = 0,j = 0;
    Mat t_image = dst.clone();

    for(n=0; n<iterations; n++)
    {
        t_image = dst.clone();
        for(i=0; i<Height;  i++)
        {
            for(j=0; j<Width; j++)
            {
                if (t_image.data[i*Width+j] == tempWhite)
                {
                    int ap = 0;
                    int p2 = (i==0)?0:(t_image.data[(i-1)*Width+j] & 0b00000001);
                    int p3 = (i==0 || j==Width-1)?0:(t_image.data[(i-1)*Width+(j+1)] & 0b00000001);
                    if (p2==0 && p3==1)
                    {
                        ap++;
                    }

                    int p4 = (j==Width-1)?0:(t_image.data[i*Width+(j+1)] & 0b00000001);
                    if(p3==0 && p4==1)
                    {
                        ap++;
                    }

                    int p5 = (i==Height-1 || j==Width-1)?0:(t_image.data[(i+1)*Width+(j+1)] & 0b00000001);
                    if(p4==0 && p5==1)
                    {
                        ap++;
                    }

                    int p6 = (i==Height-1)?0:(t_image.data[(i+1)*Width+j] & 0b00000001);
                    if(p5==0 && p6==1)
                    {
                        ap++;
                    }

                    int p7 = (i==Height-1 || j==0)?0:(t_image.data[(i+1)*Width+(j-1)] & 0b00000001);
                    if(p6==0 && p7==1)
                    {
                        ap++;
                    }

                    int p8 = (j==0)?0:(t_image.data[i*Width+(j-1)] & 0b00000001);
                    if(p7==0 && p8==1)
                    {
                        ap++;
                    }

                    int p9 = (i==0 || j==0)?0:(t_image.data[(i-1)*Width+(j-1)] & 0b00000001);
                    if(p8==0 && p9==1)
                    {
                        ap++;
                    }
                    if(p9==0 && p2==1)
                    {
                        ap++;
                    }

                    if((p2+p3+p4+p5+p6+p7+p8+p9)>1 && (p2+p3+p4+p5+p6+p7+p8+p9)<7)
                    {
                        if(ap==1)
                        {
                            if(!(p2 && p4 && p6))
                            {
                                if(!(p4 && p6 && p8))
                                {
                                    dst.data[i*Width+j] = 0;
                                }
                            }
                        }
                    }

                }
            }
        }

        t_image = dst.clone();
        for(i=0; i<Height;  i++)
        {
            for(j=0; j<Width; j++)
            {
                if (t_image.data[i*Width+j] == tempWhite)
                {
                    int ap = 0;
                    int p2 = (i==0)?0:(t_image.data[(i-1)*Width+j] & 0b00000001);
                    int p3 = (i==0 || j==Width-1)?0:(t_image.data[(i-1)*Width+(j+1)] & 0b00000001);
                    if (p2==0 && p3==1)
                    {
                        ap++;
                    }

                    int p4 = (j==Width-1)?0:(t_image.data[i*Width+(j+1)] & 0b00000001);
                    if(p3==0 && p4==1)
                    {
                        ap++;
                    }

                    int p5 = (i==Height-1 || j==Width-1)?0:(t_image.data[(i+1)*Width+(j+1)] & 0b00000001);
                    if(p4==0 && p5==1)
                    {
                        ap++;
                    }

                    int p6 = (i==Height-1)?0:(t_image.data[(i+1)*Width+j] & 0b00000001);
                    if(p5==0 && p6==1)
                    {
                        ap++;
                    }

                    int p7 = (i==Height-1 || j==0)?0:(t_image.data[(i+1)*Width+(j-1)] & 0b00000001);
                    if(p6==0 && p7==1)
                    {
                        ap++;
                    }

                    int p8 = (j==0)?0:(t_image.data[i*Width+(j-1)] & 0b00000001);
                    if(p7==0 && p8==1)
                    {
                        ap++;
                    }

                    int p9 = (i==0 || j==0)?0:(t_image.data[(i-1)*Width+(j-1)] & 0b00000001);
                    if(p8==0 && p9==1)
                    {
                        ap++;
                    }
                    if(p9==0 && p2==1)
                    {
                        ap++;
                    }

                    if((p2+p3+p4+p5+p6+p7+p8+p9)>1 && (p2+p3+p4+p5+p6+p7+p8+p9)<7)
                    {
                        if(ap==1){
                            if(p2*p4*p8==0)
                            {
                                if(p2*p6*p8==0)
                                {
                                    dst.data[i*Width+j] = 0;
                                }
                            }
                        }
                    }

                }
            }
        }

    }
}

int FieldDetector::coord(int x,int y)
{
//校正坐标，防止坐标位于图像外
    if (x < 0)return 0;
    if (x > y)return y;
}

double FieldDetector::countPercent(Mat src,Rect r,int targetColor)
{
//计算矩形r中颜色为targetColor的像素所占的百分比
    int countColor = 0;
    for (int i = r.y;i < r.y + r.height;i ++){
        for (int j = r.x;j < r.x + r.width;j ++){
            if (src.data[i*Width+j] == targetColor)
                countColor ++;
        }
    }
    return (countColor*1.0/(r.height*r.width));
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "fielddetect");

    FieldDetector ic;

    ROS_INFO("Field detection program starting...");

    namedWindow("Trackbar", 10);
    createTrackbar( "Hough", "Trackbar", &Hough_Thresh, 250);
    createTrackbar( "minLength", "Trackbar", &minLineLength, 200);
    createTrackbar( "PtGapL", "Trackbar", &maxGapPointToLine, 200);
    createTrackbar( "prolong", "Trackbar", &prolongThresh, 20);
    createTrackbar( "whiteNum", "Trackbar", &whiteNumThresh, 20);
    createTrackbar( "thinIter", "Trackbar", &thinningIter, 20);


    namedWindow("Trackbar_hsv", 10);
    createTrackbar( "low_h", "Trackbar_hsv", &low_h, 179);
    createTrackbar( "low_s", "Trackbar_hsv", &low_s, 255);
    createTrackbar( "low_v", "Trackbar_hsv", &low_v, 255 );
    createTrackbar( "high_h", "Trackbar_hsv", &high_h, 179);
    createTrackbar( "high_s", "Trackbar_hsv", &high_s, 255);
    createTrackbar( "high_v", "Trackbar_hsv", &high_v, 255);
    createTrackbar( "shot", "Trackbar_hsv", &switch_screenshot, 1);
    createTrackbar( "delta", "Trackbar_hsv", &delta, 100);
//    createTrackbar("Denoise", "HSV_Control", &Noise_Threshold, 100);
    createTrackbar("Size", "Trackbar_hsv", &BlockSize, 9);

    while (ros::ok())
    {
        //ROS_INFO("Spinned once");
        ros::spinOnce();


    }

    return 0;

}
