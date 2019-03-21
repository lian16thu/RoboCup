// Code for implementing the simple visualization of the particle filter localization algorithm including the calculation of
// the robot pose, the ball and obstacle location within the field.
// dependencies: 'field_model.h' header files for the model of the field
//               'localization/MeanPoseConfStamped.h','localization/ParticleSet.h' header files for messages
//
// Maintainer: Sotirios Stasinopoulos email: sotstas@gmail.com
// Last edited: 2014.04.06


#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cstdlib>
#include <sensor_msgs/image_encodings.h>
#include <opencv/cvwimage.h>
#include <opencv/highgui.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <cv_bridge/cv_bridge.h>

#include <boost/foreach.hpp>

#include <localization/ParticleSet.h>
#include <localization/MeanPoseConfStamped.h>
#include <localization/OutputData.h>
#include "field_model.h"

// // Extra OpenCV visualization window if we decide to exclude the final "image_view"
 static const std::string OPENCV_WINDOW = "Image window";

class LocalizationVisualization
{
    ros::NodeHandle lv_nh;

    //Create image transport handle and connect it to the incoming/outgoing topics
    image_transport::ImageTransport it;
    ros::Subscriber sub_particle_set;
    ros::Subscriber sub_mean_pose;    //发送平均位置
    image_transport::Publisher pub;

public:
    localization::OutputData viz_output_data;

    LocalizationVisualization()
        : it(lv_nh)
    {
        sub_particle_set = lv_nh.subscribe("localization/particle_set", 10, &LocalizationVisualization::newParticleSetCb, this);
        sub_mean_pose = lv_nh.subscribe("localization/output_data", 10, &LocalizationVisualization::newOutputDataCb, this);
        pub = it.advertise("localization/visualization", 10);

        //    cv::namedWindow(OPENCV_WINDOW);
        viz_output_data.robotPose.x = viz_output_data.robotPose.y = 0.0;   //初始位置为（0，0）
        viz_output_data.robotPoseConfidence = 1;      //权重为1
        viz_output_data.bBallWasSeen = 0;             //默认看不到球
        viz_output_data.bObstacleWasSeen = 0;         //默认看不到障碍物
        viz_output_data.bOpponentWasSeen = 0;           //默认看不到守门员
    }

    ~LocalizationVisualization()
    {
        //    cv::destroyWindow(OPENCV_WINDOW);
    }

    // Callback function for new output data, including current mean pose, ball position and obstacle position

    void newOutputDataCb(const localization::OutputDataConstPtr& output_data_msg)
    {
        viz_output_data = *output_data_msg;
        //ROS_ERROR("viz: Mean pose (%f,%f,%f) with confidence %f",output_data_msg->robotPose.x, output_data_msg->robotPose.y,
          //      output_data_msg->robotPose.theta, output_data_msg->robotPoseConfidence);      //用ERROR形式显示消息内容

    }

    // Callback function that creates the image and draws upon it the field according to the field_model
    void newParticleSetCb(const localization::ParticleSetConstPtr& particle_set_msg) //使用OPENCV画图
    {
        field_model::FieldModel* field = field_model::FieldModel::getInstance();

        sensor_msgs::Image initial_image;
        initial_image.header.stamp = particle_set_msg->header.stamp;
        initial_image.header.frame_id = "/field";
        initial_image.height = (field->width() + 2) *100; // image height = field width + 1m on each side
        initial_image.width = (field->length() + 2) *100; // image width = field length + 1m on each side
        initial_image.encoding = sensor_msgs::image_encodings::BGR8;
//        ROS_INFO("height: [%d] width: [%d] encoding: [%s]", initial_image.height, initial_image.width, initial_image.encoding.c_str());
        initial_image.is_bigendian = true;
        initial_image.step = 3 * initial_image.width; // line in bytes = 3 colors of 1 byte per pixel
        initial_image.data.resize(initial_image.height*initial_image.step);


        // Create images compatible with OpenCV using cv_bridge
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(initial_image, "bgr8");
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }


        // Drawing of field on green background

        cv_ptr->image = cv::Mat::ones(cv_ptr->image.rows,cv_ptr->image.cols,CV_8UC3);
        cv::rectangle(cv_ptr->image,cv::Point(20,20),cv::Point(180+field->length()*100,180+field->width()*100),
                      cv::Scalar(0,255,0),-1);

        // Drawing field lines    //画场地
        // Main lines and goal area lines
        cv::rectangle(cv_ptr->image,cv::Point(100,100),cv::Point(100+field->length()*100,100+field->width()*100),
                      cv::Scalar(255,255,255),2);
        cv::rectangle(cv_ptr->image,cv::Point(100,100+(100*(field->width()-field->goalAreaWidth())/2)),
                      cv::Point(100+(field->goalAreaDepth()*100),
                                100+100*(field->width()-field->goalAreaWidth())/2+100*field->goalAreaWidth()),
                      cv::Scalar(255,255,255),2);
        cv::rectangle(cv_ptr->image,cv::Point(100+100*(field->length()-field->goalAreaDepth()),
                                              100+(100*(field->width()-field->goalAreaWidth())/2)),
                      cv::Point(100+(field->goalAreaDepth()*100)+100*(field->length()-field->goalAreaDepth()),
                                100+100*(field->width()-field->goalAreaWidth())/2+100*field->goalAreaWidth()),
                      cv::Scalar(255,255,255),2);
        // Center line and circle
        cv::line(cv_ptr->image,cv::Point(100+100*field->length()/2,100),cv::Point(100+100*field->length()/2,100+100*field->width()),
                 cv::Scalar(255,255,255),2);
        cv::circle(cv_ptr->image,cv::Point(100+100*field->length()/2,100+100*field->width()/2),100*field->centerCircleDiameter()/2,
                   cv::Scalar(255,255,255),2);
        cv::circle(cv_ptr->image,cv::Point(100+100*field->length()/2,100+100*field->width()/2),2,
                   cv::Scalar(255,255,255),-1);

        // Goalpost lines
        cv::rectangle(cv_ptr->image,cv::Point(100-60,100+(100*(field->width()-field->goalWidth())/2)),
                      cv::Point(100,100+100*(field->width()-field->goalWidth())/2+100*field->goalWidth()),
                      cv::Scalar(0,255,255),2);
        cv::rectangle(cv_ptr->image,cv::Point(100+100*field->length(),100+(100*(field->width()-field->goalWidth())/2)),
                      cv::Point(100+100*field->length()+60,100+100*(field->width()-field->goalWidth())/2+100*field->goalWidth()),
                      cv::Scalar(0,255,255),2);
        // Penalty markers
        cv::circle(cv_ptr->image,cv::Point(100+100*field->penaltyMarkerDist(),100+100*field->width()/2),2,
                   cv::Scalar(255,255,255),-1);
        cv::circle(cv_ptr->image,cv::Point(100+100*(field->length()-field->penaltyMarkerDist()),100+100*field->width()/2),2,
                   cv::Scalar(255,255,255),-1);



        // Drawing particles from the particle set      //画点

        BOOST_FOREACH(const localization::Particle& p, particle_set_msg->particles)
        {
            // Draw a circle for every particle
            if ((abs(p.pose.x) <= field->length()/2) && (abs(p.pose.x) <= field->width()))
            {
                cv::circle(cv_ptr->image, cv::Point((p.pose.x+1+(field->length()/2))*100,
                                                    (p.pose.y+1+(field->width()/2))*100),
                          5*p.weight, cv::Scalar(125,125,125));
                cv::line(cv_ptr->image, cv::Point((p.pose.x+1+(field->length()/2))*100,(p.pose.y+1+(field->width()/2))*100),
                         cv::Point((p.pose.x+1+(field->length()/2))*100+5*p.weight*cos(p.pose.theta),
                                  (p.pose.y+1+(field->width()/2))*100+5*p.weight*sin(p.pose.theta)),
                         cv::Scalar(125,125,125));
            }
            else
           {
                //ROS_ERROR("a particle is out of the field");
            }
        }

        // Drawing robot mean pose     //画平均位置
        // we add +1m to all poses for the margin coverage

        cv::circle(cv_ptr->image, cv::Point((viz_output_data.robotPose.x+1+(field->length()/2))*100,
                                            (viz_output_data.robotPose.y+1+(field->width()/2))*100),
                   50*viz_output_data.robotPoseConfidence, cv::Scalar(0,0,255));
        cv::line(cv_ptr->image, cv::Point((viz_output_data.robotPose.x+1+(field->length()/2))*100,
                                          (viz_output_data.robotPose.y+1+(field->width()/2))*100),
                 cv::Point((viz_output_data.robotPose.x+1+(field->length()/2))*100+
                           50*viz_output_data.robotPoseConfidence*cos(viz_output_data.robotPose.theta),
                           (viz_output_data.robotPose.y+1+(field->width()/2))*100+
                           50*viz_output_data.robotPoseConfidence*sin(viz_output_data.robotPose.theta)),
                 cv::Scalar(0,0,255));

                // Drawing robot head mean pose     //画平均位置
                // we add +1m to all poses for the margin coverage

                cv::circle(cv_ptr->image, cv::Point((viz_output_data.robotHeadPose.x+1+(field->length()/2))*100,
                                                    (viz_output_data.robotHeadPose.y+1+(field->width()/2))*100),
                           30*viz_output_data.robotPoseConfidence, cv::Scalar(0,0,0));
                cv::line(cv_ptr->image, cv::Point((viz_output_data.robotHeadPose.x+1+(field->length()/2))*100,
                                                  (viz_output_data.robotHeadPose.y+1+(field->width()/2))*100),
                         cv::Point((viz_output_data.robotHeadPose.x+1+(field->length()/2))*100+
                                   30*viz_output_data.robotPoseConfidence*cos(viz_output_data.robotHeadPose.theta),
                                   (viz_output_data.robotHeadPose.y+1+(field->width()/2))*100+
                                   30*viz_output_data.robotPoseConfidence*sin(viz_output_data.robotHeadPose.theta)),
                         cv::Scalar(0,0,0));

        // Drawing ball with orange
        if (viz_output_data.bBallWasSeen) //it could have been seen but the position may not have been transformed
                                          // so it can be using the initial (0,0)
        {
            ROS_ERROR("visualization: ball at (%f,%f)",viz_output_data.ballCenterOnField.x,viz_output_data.ballCenterOnField.y);
            cv::circle(cv_ptr->image, cv::Point((viz_output_data.ballCenterOnField.x+1+(field->length()/2))*100,
                                                (viz_output_data.ballCenterOnField.y+1+(field->width()/2))*100),
                       11, cv::Scalar(0,140,255),-1);
        }

        // Drawing obstacles with blue
        if (viz_output_data.bObstacleWasSeen) //it could have been seen but the position may not have been transformed
                                              // so it can be using the initial (0,0)
        {
            for (int k=0;k<viz_output_data.obstacleCenterOnField.size();k++)
            {
                geometry_msgs::Point temp_point = viz_output_data.obstacleCenterOnField[k];
                ROS_ERROR("visualization: obstacle at (%f,%f)",temp_point.x,temp_point.y);
                cv::circle(cv_ptr->image, cv::Point((temp_point.x+1+(field->length()/2))*100,
                                                    (temp_point.y+1+(field->width()/2))*100),
                           viz_output_data.obstacleRadiusOnField[k], cv::Scalar(255,0,0),-1);
    //                       viz_output_data.obstacleRadiusOnField[k], cv::Scalar(255,0,0),-1);
    //            viz_output_data.obstacleCenterOnField.pop_back();
            }
        }

        // Drawing goalkeeper with purple
        if (viz_output_data.bOpponentWasSeen)
        {
            geometry_msgs::Point temp_point = viz_output_data.opponentCenterOnField;
            cv::circle(cv_ptr->image, cv::Point((temp_point.x+1+(field->length()/2))*100,
                                                (temp_point.y+1+(field->width()/2))*100),
                       viz_output_data.opponentRadiusOnField, cv::Scalar(128,128,0),-1);
        }


        //     // Update GUI Window
             cv::imshow(OPENCV_WINDOW, cv_ptr->image);
             cv::waitKey(3);

        //Publishing the drawn image
        if (sensor_msgs::ImagePtr image_with_particles = cv_ptr->toImageMsg())
        {
            pub.publish(image_with_particles);
        }
        else
            fprintf(stderr,"Couldn't publish visualization image");


    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "localization_visualization");

    LocalizationVisualization lv;
    //ROS_INFO("After visualization initialization");

    //ROS_INFO("Before calling sleep");
    ros::Rate rate(20); // because step period is 1/20 = 0.05sec
    rate.sleep(); // delay before initializing in order to give time to the pf_localization to produce a mean pose value
    rate.sleep();
    rate.sleep();

    while(ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
