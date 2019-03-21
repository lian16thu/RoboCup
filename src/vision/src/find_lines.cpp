/** 预处理 -> 罚球点 -> thinning&霍夫变换 -> 求归类后的有效直线 -> 场线交点
    by wuqh 2015-05-12
  */
#include <ros/ros.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <string>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include "vision_define.h"


#define CenterX (Height/2)
#define CenterY (Width/2)
#define COLOR_Orange  0*32//--------->根据新的色表进行修改
#define	COLOR_Yellow  1*32
#define	COLOR_Blue  2*32
#define	COLOR_Green 3*32
#define	COLOR_White  4*32
#define COLOR_OutField 5*32
#define	COLOR_Black  8*32

//image_out1
#define tempWhite 255 //image_out1只有两种颜色，有效的白色用tempWhite表示，其他用0表示

//检测罚球点
#define areaLow_Thresh 20
#define areaHeight_Thresh 600
#define flurLow_Thresh 10
#define flurHigh_Thresh 800
#define maxDisOfPenaltyToBoundary 50 //如果该矩形距离边界太近，则不考虑
#define Percent_lowThresh 0.4 //滤除球的白色
#define Percent_highThresh 0.6//如果白色的面积覆盖率足够大则采用
#define DBSCAN_eps 80
#define ratioEdge 4 //矩形的长宽比

//滤波
#define myBlockSize 5
#define myFlurring_Thresh myBlockSize*myBlockSize/3

//连线连线
#define BLOCK_WIDTH 60
#define BLOCK_HEIGHT 30
#define LINE_SIZE 20

//thinning
#define thinningIter 32

//直线检测
#define Hough_Thresh 100 //在霍夫空间内的阈值，大于该值时判定为直线
#define minLineLength 120
#define maxGapPointToLine 25//直线归类

using namespace cv;
using namespace std;

typedef struct myPoint{//to use vector
    int x;
    int y;
    bool state;//used in judge whether the point is the real crossPoint.
}MyPoint;

typedef struct k_and_b{
	double k;
	double b;
	
	bool operator< (const k_and_b a)const{
		if (k != a.k) return (k < a.k);
		else return (b < a.b);	
	}
}k_AND_b;

class myThin{
    ros::NodeHandle nh;
    image_transport::ImageTransport it;
    image_transport::Subscriber image_sub;
    image_transport::Publisher image_pub1;
    image_transport::Publisher image_pub2;
    image_transport::Publisher image_pub3;
    image_transport::Publisher image_pub4;
    image_transport::Publisher image_pub5;
    image_transport::Publisher image_pub6;
private:
    int boundary[Width];//场地的边界
    int state[Width*Height] = {0};//找罚球点用到的状态数组，记录图片中每个像素的状态 0:未检测   1:检测后合格   2：检测后不合格
public:
    //构造函数
    myThin():it(nh){
        image_sub = it.subscribe("field_img", 1, &myThin::Thin,this);
        //image_sub = it.subscribe("/zed/left/image_rect_color", 1, &myThin::Thin,this);
        image_pub1 = it.advertise("line_out1", 1);
	image_pub2 = it.advertise("line_out2", 1);
	image_pub3 = it.advertise("line_out3", 1);
        image_pub4 = it.advertise("line_out4", 1);
	image_pub5 = it.advertise("line_out5", 1);
	image_pub6 = it.advertise("line_out6", 1);
    }

    //其他函数声明
    void cvThin(Mat& src, Mat& dst, int iterations);//声明细化函数
    void flurring(Mat& src,int blockSize,int flurring_Thresh);
    int coord(int x,int y);//校正坐标
    void drawPoint(int x,int y, sensor_msgs::Image& image);//在图像中画出坐标为(x,y)的点
    bool calculateCrossPoint(int * x,int * y,double k1,double b1,double k2,double b2,sensor_msgs::Image& image);//计算两条直线的交点
    void drawLine(double k,double b,sensor_msgs::Image& image);//在图像中画出斜率为k，截距为b的直线，注意，仅在此，以左上角为原点，宽度方向为x轴，高度方向为y轴
    double Distance(double k,double b,int x,int y);//求出点(x,y)到直线(k，b)的距离，坐标系的要求见函数定义处的注释
    double countPercent(Mat src,Rect r,int targetColor);//计算矩形r中颜色为targetColor的像素所占的百分比
    void setRect(Mat& src,Rect r,int targetColor);//将矩形内的元素置为targetColor
    int DBSCAN(vector<Rect> input_rect, vector<int>& out, double eps = 60, int Minpts = 10);//聚类
    void wuqh(Mat &image);
    int get_endpoint(Mat image, int x, int y, int &end_x, int &end_y);
    //收到场地图片后的回调函数
    void Thin(const sensor_msgs::ImageConstPtr& image)
    {
        sensor_msgs::Image image_out1,image_out2,image_out3,image_out4,image_out5,image_out6;//img_out;
        image_out1 = *image;
	image_out6 = image_out5 = image_out4 = image_out3 = image_out2 = image_out1;
        //img_out = image_out1;//*image;


        //--------Start Section I  预处理-------------------------------------------------//
	//确定场地的边界点：boundary[],场地中只留下了白色信息，其值未tempWhite,其余为0；
        ROS_INFO("yepe1");
        for (int j = 0;j < Width;j ++)
        {
            int i = 0;
            for (i = 0;i < Height;i ++)
            {
                //如果先没有搜到绿色（场地边界）将其置为黑。//如果先搜到了白色，则置为黑；
                if (image_out1.data[i*Width+j] != COLOR_Green)
                {
                    image_out1.data[i*Width+j] = 0;
                }
                //如果搜到了绿色，将该点记为边界点，并在1和2中均置为黑；
                else
                {
                    boundary[j] = i;
                    image_out1.data[i*Width+j] = 0;
                    break;
                }
            }
            if (i == Height)
            {
                boundary[j] = Height;//如果没有找到边界点，则边界点的之为图片的高度。
            }
        }
	//将场地内的白色置为tempWhite，其余的颜色均置为黑（0）
        for (int j = 0;j < Width;j ++){
            for (int i = boundary[j]+1;i < Height;i ++){
                if (image_out1.data[i*Width+j] != COLOR_Green/*image_out1.data[i*Width+j] == COLOR_White || image_out1.data[i*Width+j] == COLOR_Black*/){
                    image_out1.data[i*Width+j] = tempWhite;
                }
                else{
                    image_out1.data[i*Width+j] = 0;
                }
            }
        }	
        //此时信息已预处理完毕，然后将sensor_msg转换为Mat。
        Mat pSrc = Mat::zeros(Height,Width,CV_8UC1);
	for (int i = 0;i < Height;i ++)
		for (int j = 0;j < Width;j ++)
			pSrc.data[i*Width+j] = image_out1.data[i*Width + j];	
	image_pub1.publish(image_out1);
		
	
	//----------------------Section II 检测罚球点 ---------------------------------------//
	//-------[可调参数]areaLow_Thresh，areaHeight_Thresh---------------------------------//
	double penaltyConfidence = 0;	
	int penaltyX;//宽度方向	
	int penaltyY;//高度方向				
	vector<vector<cv::Point> > contours;
	vector<Rect> targetRect;//聚类输入
	vector<int> penaltyClass;//聚类输出
	int penaltyClassNum;//聚类，类数	
	vector<int> classMemberNum;//每一类中的矩形数目
	int memberE1ClassNum = 0;//单个矩形即为一类的数目
	int targetPenaltyNo = -1;//记录最终的罚球点
	
	Mat srcFind = pSrc.clone();
	findContours(srcFind,contours,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);//查找轮廓，对应连通域 
	vector<double> AreaOfConnectedDomain; //记录所有连通域的面积
	for(size_t i = 0; i < contours.size(); i++){  
	    double temp_area = contourArea(contours[i]);// 计算所有连通域的面积
	    AreaOfConnectedDomain.push_back(temp_area);   
	}  	  
	// 对image_out2中的白点进行去噪处理。在后面的部分再进行进一步的罚球点检测。
	for (size_t i = 0; i < contours.size(); i++)  
	{   
	        if (AreaOfConnectedDomain[i] < flurLow_Thresh){//滤除白色杂点{
			cv::Rect r = cv::boundingRect(contours[i]);
			setRect(pSrc,r,0);
		}
		else if (AreaOfConnectedDomain[i] > flurHigh_Thresh){//滤除球的白色{
			cv::Rect r = cv::boundingRect(contours[i]);
			if (countPercent(pSrc,r,tempWhite) > Percent_lowThresh)
				setRect(pSrc,r,0);
		}
		else if ((AreaOfConnectedDomain[i] > areaLow_Thresh) && (AreaOfConnectedDomain[i] < areaHeight_Thresh)){//---------->
			cv::Rect r = cv::boundingRect(contours[i]);  
		   	 cv::rectangle(pSrc, r, cv::Scalar(255));
		}
	}
	/*for (int j = 0;j < Width;j ++){
		drawPoint(coord(boundary[j],Height-1),j,image_out2);		
		for (int i =0;i < Height;i++)
			image_out2.data[i*Width+j] = pSrc.data[i*Width+j];
	}
	image_pub2.publish(image_out2);*/
	//滤波
	flurring(pSrc,myBlockSize,myFlurring_Thresh);
	for (int j = 0;j < Width;j ++){
		//drawPoint(coord(boundary[j],Height-1),j,image_out2);		
		for (int i =0;i < Height;i++)
			image_out2.data[i*Width+j] = pSrc.data[i*Width+j];
	}
	image_pub2.publish(image_out2);
        wuqh(pSrc);
	for (int i =0; i < Height;i ++)
		for (int j = 0;j < Width;j ++)
			image_out4.data[i*Width+j] = pSrc.data[i*Width+j];
	image_pub4.publish(image_out4);
	//-----------Start Section II  thinning算法，霍夫变换----------------------------------//
        //-----------［可调参数］：cvThin中的迭代次数32,HoughLinesP()中的参数--------------------//
        ROS_INFO("yepe2");
        Mat pThin = pSrc.clone();
        cvThin(pSrc,pThin,thinningIter);//----->32为迭代次数，可根据具体的图片进行调整：通过thinning把圆变为点，圆越大，则迭代次数需越大 
	/*for (int i =0; i < Height;i ++)
		for (int j = 0;j < Width;j ++)
			image_out4.data[i*Width+j] = pThin.data[i*Width+j];
	image_pub4.publish(image_out4);*/

	vector<Vec4i> mylines;
        HoughLinesP(pThin, mylines, 1, CV_PI/360, Hough_Thresh, minLineLength, 10 );
        //ROS_INFO("num of lines = %d",mylines.size());


        //-----Start Section III  对由霍夫变换得到的直线进行分类，找出归类后的有效直线-------//
        //----------[可调参数]判断点与直线距离Distance的结果，此处为 15---------------------//
        //-------根据两点式求解k，b: k = (y2-y1)/(x2-x1); b = y1 - (y2-y1)/(x2-x1)*x1;-----//
        //-------y=kx+b --> kx-y+b=0 --> A=k,B=-1,C=b;  d=fab(Ax0+By0+C)/pow(A^2+B^2)-----//
        ROS_INFO("yepe3");
	vector<k_AND_b> k_b;//记录场地中直线的k、b信息
	k_AND_b temp_k_b;
        for (int i = 0;i < mylines.size();i ++){
            Vec4i l = mylines[i];//x1 = l[0],y1 = l[1],x2 = l[2];y2 = l[3]//以左上角为原点，宽度方向为x轴，高度方向为y轴!!!
            temp_k_b.k = (l[3]-l[1])*1.0/(l[2]-l[0]);
            temp_k_b.b = l[1] - (l[3]-l[1])*1.0/(l[2]-l[0])*1.0*l[0];
            if (k_b.size() == 0){
                k_b.push_back(temp_k_b);
            }
            else{
                int j = 0;
                for (j = 0;j < k_b.size();j ++){
                    //如果两个点距离当前直线的距离均小于25，则归并,否则新建曲线
                    if ((fabs(Distance(k_b[j].k,k_b[j].b,l[0],l[1])) < maxGapPointToLine)&&(fabs(Distance(k_b[j].k,k_b[j].b,l[2],l[3])) < maxGapPointToLine)){
                            //ROS_INFO("  similar!j = %d  count_kb = %d",j,k_b.size());
                            k_b[j].k = (k_b[j].k+temp_k_b.k)/2;
                            k_b[j].b = (k_b[j].b+temp_k_b.b)/2;
                            break;
                    }
                }
                //如果没有找到同类点；
                if (j == k_b.size()){
                    k_b.push_back(temp_k_b);
                }
            }
        }

	
	//---------罚球点检测第二部分---------------------------------------------------//
	bool flagPenaltyToLine = true;
	int PenaltyNo = -1;
	int tempHeight = 520;	
	for (size_t i = 0; i < contours.size(); i++)  
	{   
		flagPenaltyToLine = true;
	        if ((AreaOfConnectedDomain[i] > areaLow_Thresh) && (AreaOfConnectedDomain[i] < areaHeight_Thresh)){//---------->
            cv::Rect r = cv::boundingRect(contours[i]);
			if (fabs(boundary[r.x] - r.y) < maxDisOfPenaltyToBoundary){//如果该矩形距离边界太近，则不考虑
				ROS_INFO("test111");				
				continue;
			} 
			if ((r.width*1.0/r.height > ratioEdge) || (r.height*1.0/r.width > ratioEdge)){//如果矩形的长宽比太大，则不考虑
				ROS_INFO("TESTAAA3");
				ROS_INFO("%lf, %lf",r.width*1.0/r.height,r.height*1.0/r.width);
				continue;
			}
			for (int i = 0;i < k_b.size();i ++){//判断罚球点与场线的距离
				if (fabs(Distance(k_b[i].k,k_b[i].b,(r.x + r.width/2),(r.y + r.height/2))) < 20){
					flagPenaltyToLine = false;
					break;
				}	
			}
			if (flagPenaltyToLine == false){
				continue;
				ROS_INFO("TESTAAA4");
			}
			if (countPercent(pSrc,r,tempWhite) > Percent_highThresh) {//如果白色的覆盖率
				//画出所有可能的罚球点
                cv::rectangle(pSrc, r, cv::Scalar(255));
				if (r.y < tempHeight){//找到最高的罚球点
					tempHeight = r.y;
					PenaltyNo = i;
				}
			}
			
		}
	}
	for (int j = 0;j < Width;j ++){
		drawPoint(coord(boundary[j],Height-1),j,image_out5);		
		for (int i =0;i < Height;i++)
			image_out5.data[i*Width+j] = pSrc.data[i*Width+j];
	}
	image_pub5.publish(image_out5);
	if (PenaltyNo != -1){
		cv::Rect r = cv::boundingRect(contours[PenaltyNo]); 
		drawPoint(r.y+r.height/2,r.x+r.width/2,image_out6);
		image_pub6.publish(image_out6);
	}
	//Mat toShowPenalty(Height,Width,CV_8UC1);
	/*bool flagPenaltyToLine = true;
	for (size_t i = 0; i < contours.size(); i++)  
	{   
	        if (AreaOfConnectedDomain[i] > areaLow_Thresh && AreaOfConnectedDomain[i] < areaHeight_Thresh){//罚球点可能出现的情况
			//如果该矩形距离边界太近，则不考虑
			ROS_INFO("TESTAAA1");		        
			cv::Rect r = cv::boundingRect(contours[i]);
			if (fabs(boundary[r.x] - r.y) < maxDisOfPenaltyToBoundary){
//				continue; 
				ROS_INFO("TESTAAA2");}
			//如果矩形的长宽比太大，则不考虑
			else if (r.width*1.0/r.height > ratioEdge || r.height*1.0/r.width > ratioEdge){
				//continue;
				ROS_INFO("TESTAAA3");
				ROS_INFO("%lf, %lf",r.width*1.0/r.height,r.height*1.0/r.width);}
			//如果距离某条直线十分接近，则不考虑
			else {
				for (int i = 0;i < k_b.size();i ++){
					if (fabs(Distance(k_b[i].k,k_b[i].b,(r.x + r.width/2),(r.y + r.height/2))) < 20)
						flagPenaltyToLine = false;
					else {ROS_INFO("Distance = %lf",fabs(Distance(k_b[i].k,k_b[i].b,(r.x + r.width/2),(r.y + r.height/2))) < 20);}	
				}
				if (flagPenaltyToLine == false){
				//continue;
					ROS_INFO("TESTAAA4");}
				//如果白色的面积覆盖率足够大则采用
				else if (countPercent(pSrc,r,tempWhite) > Percent_highThresh) {
					targetRect.push_back(r);
					ROS_INFO("inininininini");
					cv::rectangle(pSrc, targetRect[i], cv::Scalar(180));
				}
			}			
	    	}  
	}*/
	/*//聚类
	penaltyClassNum = DBSCAN(targetRect, penaltyClass, DBSCAN_eps,1);//聚类
	//计算置信度	
	if (penaltyClassNum < 2) penaltyConfidence = 0.9;
		else if (penaltyClassNum < 3) penaltyConfidence = 0.8;
			else if (penaltyClassNum < 4) penaltyConfidence = 0.7;
				else penaltyConfidence = 0.5;

	classMemberNum = vector<int>(penaltyClassNum,0);
	for (int i = 0;i < penaltyClass.size();i++){
		classMemberNum[penaltyClass[i]] ++;
	}
	for (int i = 0;i < classMemberNum.size();i ++){
		if (classMemberNum[i] == 1){
			memberE1ClassNum ++;
			targetPenaltyNo = i;//i即为类别号
		}
	}
	if (memberE1ClassNum > 1)
		targetPenaltyNo = -1;
	else if (memberE1ClassNum == 1){
		for (int i = 0;i < targetRect.size();i ++){
			if (penaltyClass[i] == targetPenaltyNo){//找到了！！！
				ROS_INFO("I find penalty!");
				ROS_INFO("penaltyConfidence = %lf",penaltyConfidence);
				ROS_INFO("penaltyClassNum = %d",penaltyClassNum);
				penaltyY = targetRect[i].y+targetRect[i].height/2;
				penaltyX = targetRect[i].x+targetRect[i].width/2;
				ROS_INFO("X = %d",penaltyY);
				ROS_INFO("Y = %d",penaltyX);
				ROS_INFO("test");
				//cv::cvtColor(pSrc,toShowPenalty,CV_GRAY2BGR);
				drawPoint(penaltyY,penaltyX,image_out3);
				cv::rectangle(pSrc, targetRect[i], cv::Scalar(255));
				ROS_INFO("test");				
				for (int i = 0;i < Height;i ++)
					for (int j = 0;j < Width;j ++){
						image_out5.data[i*Width+j] = pSrc.data[i*Width+j];
					}
			}
		}
	}*/

	
        //---Start Section V  计算出斜率相差较大的直线之间的，且位于场地内的交点，并在图中画出来--//
        //----[可调参数]斜率相近不计算交点的斜率阈值;每一个阈值都需要根据具体的场地来确定。-----//
        //----确定点的性质，规定，最多可以确定10，从视野最远方由左至右依次记为1 2 3 4 5 6 7 8 9 10--//
        ROS_INFO("YEPE4");
        int x, y;//暂存计算出的交点坐标,以左上角为原点，宽度方向为y轴，高度方向为x轴
	vector<MyPoint> crossPoint;  //交点  
	MyPoint temp_crossPoint;    
        //找到位于场地中的直线交点
        for (int i = 0;i < k_b.size();i++){
            for (int j = i+1;j < k_b.size();j ++){
                temp_crossPoint.state = calculateCrossPoint(&x,&y,k_b[i].k,k_b[i].b,k_b[j].k,k_b[j].b,image_out3);
		if (temp_crossPoint.state == true){//当交点位于图片中时，
                    temp_crossPoint.x = x;
                    temp_crossPoint.y = y;
                    drawPoint(x,y,image_out3);
                    crossPoint.push_back(temp_crossPoint);
                }
            }
            drawLine(k_b[i].k,k_b[i].b,image_out3);
        }
        //ROS_INFO("YEPE5,crosspoint_num = %d",crossPoint.size());
	
	image_pub3.publish(image_out3);
    }
};

void myThin::flurring(Mat& src,int blockSize,int flurring_Thresh){
	int ColorState[Width*Height] = {0};//当ColorState[i] > blockSize*blockSize/2时，该点置为tempWhite
	int tempColor = 0;
	for (int i = 0;i < Height;i ++){
		for (int j = 0;j < Width;j ++){
			tempColor = 0;
			//ROS_INFO("test");
			for (int m = i;m < i+blockSize;m ++){//计算blockSize内的白点数量
				for (int n = j;n < j+blockSize;n ++){
					if (src.data[coord(m,Height-1)*Width+coord(n,Width-1)] == tempWhite)
						tempColor ++;
				}
			}
			if (tempColor > flurring_Thresh){//如果blockSize内的白点多
				for (int m = i;m < i+blockSize;m ++)
					for (int n = j;n < j+blockSize;n ++)
						ColorState[coord(m,Height-1)*Width+coord(n,Width-1)] ++;
			}
		}
	}
	for (int i = 0;i < Height;i ++){
		for (int j = 0; j < Width;j ++){
			if (ColorState[i*Width+j] > blockSize*blockSize/2){
				src.data[i*Width+j] = tempWhite;
				//ROS_INFO("test!");
			}
			else {
				src.data[i*Width+j] = 0;
				//ROS_INFO("COLORsTATE = %d",ColorState[i]);			
			}
		}
	}
}

int myThin::coord(int x,int y){
//校正坐标，防止坐标位于图像外
    if (x < 0)return 0;
    if (x > y)return y;
}

double myThin::Distance(double k,double b,int x,int y){//求出点(x,y)到直线(k，b)的距离,以左上角为原点，宽度方向为x轴，高度方向为y轴!!!
//--> y=kx+b --> kx-y+b=0 --> A=k,B=-1,C=b; --> d=fab(Ax0+By0+C)/pow(A^2+B^2)
    //return ((fabs(k*x-y+b))/pow(k*k+1,0.5));
	return ((k*x-y+b)/pow(k*k+1,0.5));//若y>k*x+b，则点（x,y）在直线下方，所以，返回为正时在点（x,y）在直线上方！
}

void myThin::drawPoint(int x,int y, sensor_msgs::Image& image){//在图像中画出坐标为(x,y)的点
    for (int i = x-5;i < x+6;i ++)
        for (int j = y - 5;j < y + 6;j ++){
            image.data[coord(i,Height-1)*Width+coord(j,Width-1)] = 255;
        }
}

bool myThin::calculateCrossPoint(int *x,int *y,double k1,double b1,double k2,double b2, sensor_msgs::Image& image){
    int count_nonBlackPoint = 0;//用于判断点是否在绿色场地内，
    double temp_x,temp_y1,temp_y2;
    temp_x = (b2 - b1)/(k1 - k2);
    temp_y1 = k1*temp_x+b1;
    temp_y2 = k2*temp_x+b2;
    (*x) = (int)((temp_y1+temp_y2)/2);//此处做了坐标变换
    (*y) = (int)temp_x;//此处做了坐标变换
    if ((*x) > -1 && (*x) < Height && (*y) > -1 && (*y) < Width){//该点在图片内
        for (int i = (*x)-20;i < (*x)+20;i ++)
            for (int j = (*y) - 20;j < (*y) + 20;j ++){
                if ((image.data[coord(i,Height-1)*Width+coord(j,Width-1)] == COLOR_Green)||(image.data[coord(i,Height-1)*Width+coord(j,Width-1)] == COLOR_White))
                    count_nonBlackPoint ++;
            }
        if (count_nonBlackPoint > 10){
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

void myThin::drawLine(double k,double b,sensor_msgs::Image& image){
//在图像中画出斜率为k，截距为b的直线，注意，仅在此，以左上角为原点，宽度方向为x轴，高度方向为y轴
    int h;
    for (int i = 0;i < Width;i ++){
        h = (int)(k * i+b);
        if (h >=0 && h < Height)image.data[coord(h,Height-1)*Width+coord(i,Width-1)] = 255;
    }
    return;
}

void myThin::cvThin(Mat& src, Mat& dst, int iterations)
//thinning算法
{
	int n = 0,i = 0,j = 0;
	Mat t_image = dst.clone();

    for(n=0; n<iterations; n++){
        t_image = dst.clone();
        for(i=0; i<Height;  i++){
            for(j=0; j<Width; j++){
                if (t_image.data[i*Width+j] == tempWhite){                
		    int ap = 0;
                    int p2 = (i==0)?0:(t_image.data[(i-1)*Width+j] & 0b00000001);
                    int p3 = (i==0 || j==Width-1)?0:(t_image.data[(i-1)*Width+(j+1)] & 0b00000001);
                    if (p2==0 && p3==1){
                        ap++;
                    }

                    int p4 = (j==Width-1)?0:(t_image.data[i*Width+(j+1)] & 0b00000001);
                    if(p3==0 && p4==1){
                        ap++;
                    }

                    int p5 = (i==Height-1 || j==Width-1)?0:(t_image.data[(i+1)*Width+(j+1)] & 0b00000001);
                    if(p4==0 && p5==1){
                        ap++;
                    }

                    int p6 = (i==Height-1)?0:(t_image.data[(i+1)*Width+j] & 0b00000001);
                    if(p5==0 && p6==1){
                        ap++;
                    }

                    int p7 = (i==Height-1 || j==0)?0:(t_image.data[(i+1)*Width+(j-1)] & 0b00000001);
                    if(p6==0 && p7==1) {
                        ap++;
                    }

                    int p8 = (j==0)?0:(t_image.data[i*Width+(j-1)] & 0b00000001);
                    if(p7==0 && p8==1){
                        ap++;
                    }

                    int p9 = (i==0 || j==0)?0:(t_image.data[(i-1)*Width+(j-1)] & 0b00000001);
                    if(p8==0 && p9==1){
                        ap++;
                    }
                    if(p9==0 && p2==1){
                        ap++;
                    }

                    if((p2+p3+p4+p5+p6+p7+p8+p9)>1 && (p2+p3+p4+p5+p6+p7+p8+p9)<7){
                        if(ap==1){
                            if(!(p2 && p4 && p6)){
                                if(!(p4 && p6 && p8)){
					dst.data[i*Width+j] = 0;
                                }
                            }
                        }
                    }

                }
            }
        }

	t_image = dst.clone();
        for(i=0; i<Height;  i++){
            for(j=0; j<Width; j++){
                if (t_image.data[i*Width+j] == tempWhite){                
		    int ap = 0;
                    int p2 = (i==0)?0:(t_image.data[(i-1)*Width+j] & 0b00000001);
                    int p3 = (i==0 || j==Width-1)?0:(t_image.data[(i-1)*Width+(j+1)] & 0b00000001);
                    if (p2==0 && p3==1){
                        ap++;
                    }

                    int p4 = (j==Width-1)?0:(t_image.data[i*Width+(j+1)] & 0b00000001);
                    if(p3==0 && p4==1){
                        ap++;
                    }

                    int p5 = (i==Height-1 || j==Width-1)?0:(t_image.data[(i+1)*Width+(j+1)] & 0b00000001);
                    if(p4==0 && p5==1){
                        ap++;
                    }

                    int p6 = (i==Height-1)?0:(t_image.data[(i+1)*Width+j] & 0b00000001);
                    if(p5==0 && p6==1){
                        ap++;
                    }

                    int p7 = (i==Height-1 || j==0)?0:(t_image.data[(i+1)*Width+(j-1)] & 0b00000001);
                    if(p6==0 && p7==1) {
                        ap++;
                    }

                    int p8 = (j==0)?0:(t_image.data[i*Width+(j-1)] & 0b00000001);
                    if(p7==0 && p8==1){
                        ap++;
                    }

                    int p9 = (i==0 || j==0)?0:(t_image.data[(i-1)*Width+(j-1)] & 0b00000001);
                    if(p8==0 && p9==1){
                        ap++;
                    }
                    if(p9==0 && p2==1){
                        ap++;
                    }

                    if((p2+p3+p4+p5+p6+p7+p8+p9)>1 && (p2+p3+p4+p5+p6+p7+p8+p9)<7){
                        if(ap==1){
                            if(p2*p4*p8==0){
                                if(p2*p6*p8==0){
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

double myThin::countPercent(Mat src,Rect r,int targetColor){
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

void myThin::setRect(Mat& src,Rect r,int targetColor){
//将矩形内的元素置为targetColor
	for (int m = r.y;m < r.y+r.height;m ++)
		for (int n = r.x;n < r.x+r.width;n ++)
			src.data[coord(m,Height)*Width+coord(n,Width)] = targetColor;
}

int myThin::DBSCAN(vector<Rect> input_rect, vector<int> &out, double eps, int Minpts)
{
	int n = input_rect.size();
	out = vector<int>(n, -1);

	vector<Point> input(n);
	for (int i = 0;i < n; i++)
	{
		input[i].x = input_rect[i].x + input_rect[i].width/2;
		input[i].y = input_rect[i].y + input_rect[i].height/2;
	}

	int head=0, tail=0;
	int *q = new int[n+1];
	int *in_q = new int[n];
	int i, j, c=0;
	int *visited = new int[n];
	int** dist2 = new int*[n];
	for (i=0;i<n;i++) dist2[i] = new int[n];
	for (i=0;i<n;i++)
		for (j=0;j<n;j++)
			dist2[i][j] = pow(input[i].x-input[j].x, 2) + pow(input[i].y-input[j].y, 2);
	for (i=0;i<n;i++)
	{
		visited[i] = 0; //0--not visited; 1--visited;  2--noise
		in_q[i] = 0; //0--not in queue;  1--in queue
		q[i] = 0;
	}
	int flag = 1;
	while (flag)
	{
		for (i=0;i<n;i++)
			if (visited[i] == 0)
			{
				head = 0; tail = 0;
				for (int k=0;k<n;k++)
				{
					in_q[k] = 0; //0--not in queue;  1--in queue
					q[k] = 0;
				}
				int counti = 0;
				for (j=0;j<n;j++)
					if ( dist2[i][j] <= eps*eps)
					{
						counti++;
					}
					if (counti>=Minpts)
					{
						for (j=0;j<n;j++)
							if ( dist2[i][j] <= eps*eps)
							{
								q[tail] = j;
								tail = (tail+1)%(n+1);
								in_q[j] = 1;
							}
							out[i] = c;
							visited[i] = 1;
							while (head != tail)
							{
								int k=q[head];
								head = (head+1) % (n+1);
								//if (visited[k] == 0) //assert out[k] == -1
								if (out[k] == -1)
								{
									out[k] = c;
									visited[k] = 1;
								}
								int countk = 0;
								for (j=0;j<n;j++)
									if (dist2[k][j] <= eps*eps)
									{
										countk++;
									}
									if (countk>=Minpts)
									{
										for (j=0;j<n;j++)
											if (dist2[k][j] <= eps*eps)
											{
												if (in_q[j] == 0)
												{
													q[tail] = j;
													tail = (tail+1)%(n+1);
													in_q[j] = 1;
												}
											}
									}
							}
							c++;
					}
					else visited[i] = 2;
			}
			for (i=0;i<n;i++)
				if (visited[i] == 0) break;
			if (i == n) flag = 0;
	}

	return c;
}

void myThin::wuqh(Mat &image)
{
	Mat lut = Mat::zeros(Height, Width, CV_8UC1);
	for(int i = 0; i < Width; i++)
	{
		for(int j = 0; j < Height; j++)
		{
			int count = 0;
			if(image.at<uchar>(j,i) == 255 && lut.at<uchar>(j,i) == 0)
			{
				int y1 = j, y2 = j;
				while(image.at<uchar>(y1,i) == 255)
				{
					y1--;
					if(y1 == 0)
						break;
				}
				while(image.at<uchar>(y2,i) == 255)
				{
					y2++;
					if(y2 == Height - 1)
						break;
				}
				int start_x = i;
				int start_y = (y1 + y2)/2;

				int end_x,end_y;
				int flag = get_endpoint(image, i, start_y, end_x, end_y);
				if(flag == 0)
				{
					lut.at<uchar>(j, i) = 255;
					continue;
				}

				cout << endl;
				cout << "~~~~~~~~~~" << endl;
				cout << "start:" << start_x << "  " << start_y << endl;
				cout << "end:" << end_x << "  " << end_y << endl;
				cout << "~~~~~~~~~~" << endl;

				float delta_y = (float)(end_y - start_y)/(end_x - start_x);
				for(int k1 = start_x; k1 <= end_x; k1++)
				{
					int center_y = start_y + (k1 - start_x)*delta_y;
					for(int k2 = max(0, center_y - LINE_SIZE/2); k2 < min(Width, center_y + LINE_SIZE); k2++)
					{
						image.at<uchar>(k2,k1) = 255;
						lut.at<uchar>(k2,k1) = 255;
					}
				}
			}
			else
			{
				lut.at<uchar>(j, i) = 255;
			}
			
		}
	}
}

int myThin::get_endpoint(Mat image, int x, int y, int &end_x, int &end_y)
{
	for(int i = min(x + BLOCK_WIDTH/2, Width-1); i <= min(x + BLOCK_WIDTH, Width-1); i++)
	{
		for(int j = max(y - BLOCK_HEIGHT/2, 0); j <= min(y + BLOCK_HEIGHT/2, Height-1); j++)
		{
			if(image.at<uchar>(j,i) == 255)
			{
				end_x = i;
				int y1 = j, y2 = j;
				while(image.at<uchar>(y1,i) == 255)
				{
					y1--;
					if(y1 == 0)
						break;
				}
				while(image.at<uchar>(y2,i) == 255)
				{
					y2++;
					if(y2 == Height - 1)
						break;
				}
				end_y = (y1 + y2)/2;
				return 1;
			}	
		}
	}
	
	return 0 ;
}

int main(int argc, char** argv)
//主函数
{
    ros::init(argc, argv, "thin");
    myThin thin;
    ros::NodeHandle nh2;
    ros::spin();
}

