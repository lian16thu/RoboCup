#include <math.h>
#include "Definitions.h"
#include "tools.h"
#include "../Xabsl/XabslEngine/XabslEngine.h"

#include "basic_behavior.h"
#include "xabsl-debug-interface.h"

#include "std_msgs/Float64.h"
#include "std_msgs/Int8.h"

using namespace xabsl;
using namespace decision;

MyErrorHandler errorHandler;
bool isInitializing = true;
std_msgs::String console_msg;
stringstream console_strstream;
//int delay_down_flag = 0;// Delay
//int delay_up_flag = 0;// Delay

void xabslEngineRegister(xabsl::Engine *pEngine, MyErrorHandler &errorHandler);
bool processHeadMode();

std::string demo_type;

void CBonSerialReceived(const decision::SerialReceived::ConstPtr serial_input_msg)
{
    currentFrame.decision_serial_input_data = *serial_input_msg;

    //    ROS_INFO("Serial input received from serial node: %f %f %f %f %f %f %f %f \n",currentFrame.decision_serial_input_data.received_data[0],currentFrame.decision_serial_input_data.received_data[1],
    //            currentFrame.decision_serial_input_data.received_data[2],currentFrame.decision_serial_input_data.received_data[3],currentFrame.decision_serial_input_data.received_data[4],
    //            currentFrame.decision_serial_input_data.received_data[5],currentFrame.decision_serial_input_data.received_data[6],currentFrame.decision_serial_input_data.received_data[7]);

    currentFrame.odom_x = currentFrame.decision_serial_input_data.received_data[0];
    currentFrame.odom_y = currentFrame.decision_serial_input_data.received_data[1];
    currentFrame.odom_orientation = currentFrame.decision_serial_input_data.received_data[2];

    currentFrame.front_to_waist_x = serial_input_msg->received_data[5];;
    currentFrame.front_to_waist_y = serial_input_msg->received_data[6];;

    if (currentFrame.decision_serial_input_data.received_data[9]==0)
        currentFrame.is_robot_moving = false;
    else if (currentFrame.decision_serial_input_data.received_data[9]==1)
        currentFrame.is_robot_moving = true;
    else
        ROS_INFO("Wrong robot moving code");

    // Switches

    if (currentFrame.decision_serial_input_data.received_data[12]==0)
        currentFrame.remote_switch_3 = false;
    else if (currentFrame.decision_serial_input_data.received_data[12]==1)
        currentFrame.remote_switch_3 = true;
    else
        ROS_INFO("Wrong switch 3 code");
    if (currentFrame.decision_serial_input_data.received_data[13]==0)
        currentFrame.remote_switch_4 = false;
    else if (currentFrame.decision_serial_input_data.received_data[13]==1)
        currentFrame.remote_switch_4 = true;
    else
        ROS_INFO("Wrong switch 4 code");

    //ROS_ERROR("Kidnapped:%d",currentFrame.decision_serial_input_data.received_data[11]);

}

//double to string helper function
string doubleToString(double number){

    //this function has a number input and string output
    std::stringstream ss;
    ss << number;
    return ss.str();
}


void CBonRGBImageReceived(const sensor_msgs::ImageConstPtr& Image_msg)
{

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

    cv::Mat &image_raw = cv_ptr->image;
    int width = image_raw.cols;
    int height = image_raw.rows;

    image_raw.copyTo(image_raw);

    cv::startWindowThread();

    //display the rotation and translation matrix results

    std::string state_description;
    cv::Scalar state_color;

    switch((int)(currentFrame.state_code))
    {   case 0:
    {state_description = "Initialize";
        state_color = cv::Scalar(0,0,0);
        break;
    }
    case 1:
    {
        state_description = "Follow Trajectory 1";
        state_color = cv::Scalar(0,255,0);
        break;
    }
    case 2:
    {
        state_description = "Go to Pose Traj.1";
        state_color = cv::Scalar(0,0,255);
        break;
    }
    case 3:
    {
        state_description = "Avoid Obstacle Traj.1";
        state_color = cv::Scalar(0,255,0);
        break;
    }
    case 4:
    {

        state_description = "Follow Trajectory 2";
        state_color = cv::Scalar(0,0,255);
        break;
    }
    case 5:
    {
        state_description = "Go to Pose Traj.2";
        state_color = cv::Scalar(0,0,255);
        break;
    }
    case 6:
    {
        state_description = "Wait for robot to stop";
        state_color = cv::Scalar(0,0,255);
        break;
    }
    case 7:
    {

        state_description = "Locate Ball";
        state_color = cv::Scalar(128,0,127);
        break;
    }
    case 8:
    {
        state_description = "Go to Ball";
        state_color = cv::Scalar(255,0,0);
        break;
    }
    case 9:
    {
        state_description = "Kick Ball";
        state_color = cv::Scalar(128,128,0);
        break;
    }
    case 10:
    {
        state_description = "Action Over";
        state_color = cv::Scalar(255,255,255);
        break;
    }
    case 11:
    {
        state_description = "Action Over";
        state_color = cv::Scalar(255,255,255);
        break;
    }
    default:
    {
        ROS_INFO("Unknown State");
    }
    }


    cv::putText(image_raw, state_description.c_str() , cv::Point(50,100), cv::FONT_HERSHEY_SIMPLEX, 1.2, state_color, 2, cv::LINE_8);


    cv::namedWindow("Decision State", cv::WINDOW_FREERATIO);
    cv::imshow("Decision State", image_raw);
    cv::waitKey(5);

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "decision_node");
    ros::NodeHandle nodeHandle;

    nodeHandle.param<std::string>("/decision_node/demo_type", demo_type, "game");

    /*******************sevice Client******************/ // NO serviceto be used in this version
    //    ros::ServiceClient walkClient = nodeHandle.serviceClient<walk>(WALK_CONTROL_SRV);
    //    ros::ServiceClient headClient = nodeHandle.serviceClient<head>(HEAD_CONTROL_SRV);
    //    ros::ServiceClient OrientationClient = nodeHandle.serviceClient<Pathplaning_the>(ORI_CONTROL_SRV);

    /*******************message Subscriber*************/
    ros::Subscriber subscriber_gameControl = nodeHandle.subscribe(GAMECONTROL_OUTPUT_TOPIC, 10, CBOnControllingReceived);
    ros::Subscriber subscriber_goal = nodeHandle.subscribe(GOAL_RECOG_OUTPUT_TOPIC, 1, CBOnGoalReceived);
    ros::Subscriber subscriber_ball = nodeHandle.subscribe(BALL_RECOG_OUTPUT_TOPIC, 1, CBOnBallReceived);
    ros::Subscriber subscriber_opponents = nodeHandle.subscribe(OPPONENTS_RECOG_OUTPUT_TOPIC, 1, CBOnOpponentsReceived);
    ros::Subscriber subscriber_localization = nodeHandle.subscribe(LOCALIZATION_OUTPUT_TOPIC, 1, CBOnLocalizationReceived);
    ros::Subscriber subscriber_image_rgb = nodeHandle.subscribe("/zed/left/image_rect_color", 10, CBonRGBImageReceived);

    /*******************message publisher*************/
    ros::Publisher publish_decision_console = nodeHandle.advertise<std_msgs::String>("decision_console", 10);

    /*******************publisher of head rotation*************/
    ros::Publisher publish_head_rotation = nodeHandle.advertise<head_motion::head_pose>("/decision/head_command", 10);

    /*******************publisher of localization initial pose*************/
    ros::Publisher publish_loc_init_pose = nodeHandle.advertise<geometry_msgs::Pose2D>("/decision/loc_init_pose", 10);

    /*******************Serial publisher/subscriber*************/
    ros::Subscriber subscriber_serial_receiver = nodeHandle.subscribe("decision/serial_receiver", 10, CBonSerialReceived);
    ros::Publisher command_to_serial_pub = nodeHandle.advertise<decision::SerialReceived>("/decision/command_to_serial", 10);


    /*******************Xabsl*************************/
    Engine *pEngine = new Engine(errorHandler, &getSystemTime);

    MyFileInputSource inputSource("/home/nvidia/robocup2018_ws/src/decision/intermediate_code/2018game.dat");
    if (demo_type == "game")
        inputSource = "/home/nvidia/robocup2018_ws/src/decision/intermediate_code/2018game.dat";  //change that according to the required behavior
    else if (demo_type == "gotoball")
        inputSource = "/home/nvidia/robocup2018_ws/src/decision/intermediate_code/2018gotoball.dat";  //change that according to the required behavior
    else if (demo_type == "attacker")
        inputSource = "/home/nvidia/robocup2018_ws/src/decision/intermediate_code/2018attacker.dat";  //change that according to the required behavior
    else if (demo_type == "loctest")
        inputSource = "/home/nvidia/robocup2018_ws/src/decision/intermediate_code/2018loctest.dat";  //change that according to the required behavior
    else if (demo_type == "highkick")
        inputSource = "/home/nvidia/robocup2018_ws/src/decision/intermediate_code/2018highkick.dat";  //change that according to the required behavior
    else if (demo_type == "rollingball")
        inputSource = "/home/nvidia/robocup2018_ws/src/decision/intermediate_code/2018rollingball.dat";  //change that according to the required behavior
    else if (demo_type == "penalty")
        inputSource = "/home/nvidia/robocup2018_ws/src/decision/intermediate_code/2018penalty.dat";  //change that according to the required behavior
    else
            ROS_INFO("Wrong demo type");

    xabslEngineRegister(pEngine, errorHandler);
    /*******************Xabsl_behavior****************/


    behavior_initialize_motors _behavior_initialize_motors(errorHandler);
    behavior_nothing _behavior_nothing(errorHandler);
    behavior_end_game _behavior_end_game(errorHandler);
    behavior_initialize _behavior_initialize(errorHandler);
    behavior_rotate_around_center _behavior_rotate_around_center(errorHandler);
    behavior_walk _behavior_walk(errorHandler);
    behavior_stop_walk _behavior_stop_walk(errorHandler);
    behavior_step_forward _behavior_step_forward(errorHandler);
    behavior_step_backward _behavior_step_backward(errorHandler);
    behavior_step_left _behavior_step_left(errorHandler);
    behavior_step_right _behavior_step_right(errorHandler);
    behavior_kick_ball_soft _behavior_kick_ball_soft(errorHandler);
    behavior_kick_ball_mid _behavior_kick_ball_mid(errorHandler);
    behavior_kick_ball_strong _behavior_kick_ball_strong(errorHandler);
    behavior_approach_pose _behavior_approach_pose(errorHandler);
    behavior_approach_ball _behavior_approach_ball(errorHandler);

    behavior_rotate_before_walk _behavior_rotate_before_walk(errorHandler);
    behavior_walk_to_ball _behavior_walk_to_ball(errorHandler);
    behavior_walk_to_pose _behavior_walk_to_pose(errorHandler);
    behavior_rotate_after_walk _behavior_rotate_after_walk(errorHandler);
    behavior_high_kick _behavior_high_kick(errorHandler);
    behavior_connected_rotate _behavior_connected_rotate(errorHandler);
    behavior_kick_ball_right _behavior_kick_ball_right(errorHandler);
    behavior_kick_ball_left _behavior_kick_ball_left(errorHandler);

    pEngine->registerBasicBehavior(_behavior_initialize_motors);
    pEngine->registerBasicBehavior(_behavior_nothing);
    pEngine->registerBasicBehavior(_behavior_end_game);
    pEngine->registerBasicBehavior(_behavior_initialize);
    pEngine->registerBasicBehavior(_behavior_rotate_around_center);
    pEngine->registerBasicBehavior(_behavior_walk);
    pEngine->registerBasicBehavior(_behavior_stop_walk);
    pEngine->registerBasicBehavior(_behavior_step_forward);
    pEngine->registerBasicBehavior(_behavior_step_backward);
    pEngine->registerBasicBehavior(_behavior_step_left);
    pEngine->registerBasicBehavior(_behavior_step_right);
    pEngine->registerBasicBehavior(_behavior_kick_ball_soft);
    pEngine->registerBasicBehavior(_behavior_kick_ball_mid);
    pEngine->registerBasicBehavior(_behavior_kick_ball_strong);

    pEngine->registerBasicBehavior(_behavior_approach_pose);
    pEngine->registerBasicBehavior(_behavior_approach_ball);

    pEngine->registerBasicBehavior(_behavior_rotate_before_walk);
    pEngine->registerBasicBehavior(_behavior_walk_to_ball);
    pEngine->registerBasicBehavior(_behavior_walk_to_pose);
    pEngine->registerBasicBehavior(_behavior_rotate_after_walk);
    pEngine->registerBasicBehavior(_behavior_high_kick);
    pEngine->registerBasicBehavior(_behavior_connected_rotate);
    pEngine->registerBasicBehavior(_behavior_kick_ball_right);
    pEngine->registerBasicBehavior(_behavior_kick_ball_left);



    pEngine->createOptionGraph(inputSource);
    ROS_INFO("PAUSE!");
    ros::Rate loop_rate(MAIN_NODE_RUNNING_RATE);
    ROS_INFO("Initializing xabsl engine...\n");
    debug_interface dbg(pEngine);
    ROS_INFO("xabsl engine initialized, FSM starts to run...\n");

    int roundCount = 0;

    // Initialize values for variables

    currentFrame.foundBallTime = ros::Time::now().toSec();
    currentFrame.foundGoalTime = ros::Time::now().toSec();
    currentFrame.is_robot_moving = false;
    currentFrame.cannot_kick = false;

    currentFrame.ballFoundHeadAngleYaw = 0;

    currentFrame.pose_previous_x_dist = 0.0;
    currentFrame.pose_previous_y_dist = 0.0;
    currentFrame.pose_previous_bearing_diff = 0.0;
    currentFrame.approach_pose = false;
    currentFrame.reached_pose = false;

    currentFrame.robot_goal_x = 0.0;
    currentFrame.robot_goal_y = 0.0;
    currentFrame.robot_goal_theta = 0.0;

    currentFrame.robot_goal_bearing_loc = 0.0;

    currentFrame.only_turn = false;

    currentFrame.previous_robot_goal_x = 0.0;
    currentFrame.previous_robot_goal_y = 0.0;
    currentFrame.previous_robot_goal_theta = 0.0;

    currentFrame.robot_to_target_range = 0.0;
    currentFrame.robot_to_target_theta = 0.0;

    currentFrame.received_localization.robotPose.x = 0.0;
    currentFrame.received_localization.robotPose.y = 0.0;
    currentFrame.received_localization.robotPose.theta = 0.0;

    currentFrame.shoot_position_x = 0;
    currentFrame.shoot_position_y = 0;
    currentFrame.shoot_position_theta = 0;
    currentFrame.needed_turn_angle = 0;

    currentFrame.near_target_pose = false;

    currentFrame.update_command = false;

    currentFrame.no_refresh_ball = false;
    currentFrame.front_to_waist_x = 0;
    currentFrame.front_to_waist_y = 0;
    currentFrame.front_to_camera_x = 0;
    currentFrame.front_to_camera_y = 0;

    currentFrame.rolling_ball_to_robot_time = 100;

    //Switches
    currentFrame.remote_switch_1 = false;
    currentFrame.remote_switch_2 = false;
    currentFrame.remote_switch_3 = false;
    currentFrame.remote_switch_4 = false;

    for(int i=0;i<currentFrame.decision_serial_input_data.received_data.size();i++)
    {
        currentFrame.decision_serial_input_data.received_data[i] = 0;
        currentFrame.decision_serial_output_data.received_data[i] = 0;
    }

    currentFrame.loc_init_pose = 0;
    currentFrame.previous_penalized = false;
    currentFrame.previous_initial = true;
    currentFrame.previous_ready = false;
    currentFrame.init_pose_changed = false;

    currentFrame.isPenalized = false;
    currentFrame.secsTillUnpenalised = 0;
    currentFrame.secondaryTime = 0 ;

    // Get the value for attacking left or right
    nodeHandle.param<bool>("attack_left",currentFrame.attack_left, true);
    nodeHandle.getParam("attack_left",currentFrame.attack_left);

    currentFrame.ball_x = 0;
    currentFrame.ball_y = 0;

    currentFrame.isDropBall = false;

    while (ros::ok())
    {
        console_strstream.str("");

        console_strstream << "\nround: " << ++roundCount << endl;

        ros::spinOnce();



        currentFrame.FlushData();

        //        currentFrame.cannot_kick = (broadcast.InputPacket[8] == 1);

        // Debug
        //currentFrame.isReady = true;
        //currentFrame.isGameStart = true;
        //        currentFrame.isGameOver = false;
        //currentFrame.isAttacker = true;///debug*/
        //        currentFrame.sec_state = 0;
        //        currentFrame.sec_state_info = 0;

        //        if (roundCount < 80)
        //        {
        //            //Debug
        //            currentFrame.isBallSeen = true;
        //        currentFrame.ballBearing = -0.2;
        //            currentFrame.ballRange = 4.0;
        //            currentFrame.is_within_tol = false;
        //            currentFrame.is_near_enough = false;
        //            ROS_ERROR("Phase 1");
        //        }
        //        else if ((roundCount >=80) && (roundCount<180))
        //        {
        //            currentFrame.ballBearing = -0.2;
        //            currentFrame.is_within_tol = true;
        //            currentFrame.is_left = true;
        //            currentFrame.is_near_enough = false;
        //            ROS_ERROR("Phase 2");
        //        }
        //        else if (roundCount >=180)
        //        {
        //            currentFrame.is_near_enough = true;
        //            ROS_ERROR("Phase 3");
        //        }

        currentFrame.PrintReceivedData();
        //currentFrame includes the message subscribed from the service

        ROS_INFO("before execute");

        pEngine->execute();

        console_strstream << dbg.showDebugInfo().str();

        ////////////debug
        console_msg.data = console_strstream.str();
        publish_decision_console.publish(console_msg);

        cout << console_strstream.str() << endl;

        // Debug
        currentFrame.update_command = true;

        if (roundCount != 1)
        {
            processHeadMode();
        }


        // Publish the demanded head angle
        head_motion::head_pose head_pose_msg;
        head_pose_msg.pitch = currentFrame.headAnglePitch; //degrees
        head_pose_msg.yaw = currentFrame.headAngleYaw; //degrees

        ROS_INFO("Sent head rotation pitch: %f,yaw: %f",currentFrame.headAnglePitch ,currentFrame.headAngleYaw);

        publish_head_rotation.publish(head_pose_msg);
        ROS_INFO("Sent head rotation pitch: %d,yaw: %d", head_pose_msg.pitch , head_pose_msg.yaw );

        // Publish the initial pose at each moment for use by localization
        geometry_msgs::Pose2D init_pose_msg;
        switch((int)(currentFrame.loc_init_pose))
        {   case 0:
        {   init_pose_msg.x = 0;
            init_pose_msg.y = 0;
            init_pose_msg.theta = 0;
            break;
        }
        case 1:
        {
            init_pose_msg.x = -2.4;
            init_pose_msg.y = -3;
            init_pose_msg.theta = 1.57;
            break;
        }
        case -1:
        {
            init_pose_msg.x = -2.4;
            init_pose_msg.y = 3;
            init_pose_msg.theta = -1.57;
            break;
        }
        default:
        {
            ROS_INFO("Unknown Loc Initial Pose");
        }
        }

        if (currentFrame.init_pose_changed) // publish this only when the init pose changed
        {
            ROS_ERROR("Publish loc initial pose:%f",currentFrame.loc_init_pose);
            publish_loc_init_pose.publish(init_pose_msg);
            currentFrame.init_pose_changed = false;
        }

        ////////////debug
        console_msg.data = console_strstream.str();
        publish_decision_console.publish(console_msg);

        cout << console_strstream.str() << endl;


        // Publish the commands over the serial port if we have an update command
        if (currentFrame.update_command) // need to set this true each time we want a new command sent out
        {
            decision::SerialReceived serial_output_msg;

            serial_output_msg.header.stamp = ros::Time::now();
            for (int i=0;i<serial_output_msg.received_data.size();i++)
            {
                if ((currentFrame.decision_serial_output_data.received_data[i]>-1000)&&(currentFrame.decision_serial_output_data.received_data[i]<1000))
                {
                    serial_output_msg.received_data[i] = currentFrame.decision_serial_output_data.received_data[i];
                }
                else
                {
                    serial_output_msg.received_data[i] = 0;
                    ROS_INFO("Invalid value to be sent from main node, sent 0");
                }

            }

            //Debug
            //            serial_output_msg.received_data[0] = 7;
            //            serial_output_msg.received_data[1] = 0.2; //-0.15 - 0.3
            //            serial_output_msg.received_data[2] = 0.0; //-0.06 - 0.06
            //            serial_output_msg.received_data[3] = 0.5; //-0.6 - 0.6

            //            serial_output_msg.received_data[0] = 3;
            //            serial_output_msg.received_data[1] = 0.9; //-0.15 - 0.3
            //            serial_output_msg.received_data[2] = 0.2; //-0.06 - 0.06
            //            serial_output_msg.received_data[3] = 0.0; //-0.6 - 0.6

            //            serial_output_msg.received_data[0] = 6;
            //            serial_output_msg.received_data[1] = 0.65; //-0.15 - 0.3
            //            serial_output_msg.received_data[2] = 0.25; //-0.06 - 0.06
            //            serial_output_msg.received_data[3] = 0.0; //-0.6 - 0.6


            command_to_serial_pub.publish(serial_output_msg);
            currentFrame.update_command = false;
            //ROS_INFO("decision: Sent new command to serial");
        }

        loop_rate.sleep();

    }

    return 0;
}

// Function that
void xabslEngineRegister(xabsl::Engine *pEngine, MyErrorHandler &errorHandler)
{
    pEngine->registerEnumElement("_orientation", "_orientation.Up", Up);
    pEngine->registerEnumElement("_orientation", "_orientation.Down", Down);
    pEngine->registerEnumElement("_orientation", "_orientation.Left", Left);
    pEngine->registerEnumElement("_orientation", "_orientation.Right", Right);
    pEngine->registerEnumElement("_orientation", "_orientation.Mid", Mid);

    pEngine->registerEnumElement("_headMode", "_headMode.FarLeft", FarLeft);
    pEngine->registerEnumElement("_headMode", "_headMode.FarMid", FarMid);
    pEngine->registerEnumElement("_headMode", "_headMode.FarRight", FarRight);
    pEngine->registerEnumElement("_headMode", "_headMode.FarRightBack", FarRightBack);
    pEngine->registerEnumElement("_headMode", "_headMode.FarLeftBack", FarLeftBack);
    pEngine->registerEnumElement("_headMode", "_headMode.CloseLeft", CloseLeft);
    pEngine->registerEnumElement("_headMode", "_headMode.CloseMid", CloseMid);
    pEngine->registerEnumElement("_headMode", "_headMode.CloseRight", CloseRight);
    pEngine->registerEnumElement("_headMode", "_headMode.CloseRightBack", CloseRightBack);
    pEngine->registerEnumElement("_headMode", "_headMode.CloseLeftBack", CloseLeftBack);
    pEngine->registerEnumElement("_headMode", "_headMode.HorizontalTrack", HorizontalTrack);
    pEngine->registerEnumElement("_headMode", "_headMode.VerticalTrack", VerticalTrack);
    pEngine->registerEnumElement("_headMode", "_headMode.BothTrack", BothTrack);
    pEngine->registerEnumElement("_headMode", "_headMode.RollingBallLeft", RollingBallLeft);
    pEngine->registerEnumElement("_headMode", "_headMode.RollingBallRight", RollingBallRight);
    pEngine->registerEnumElement("_headMode", "_headMode.RollingBallSearchLeft", RollingBallSearchLeft);
    pEngine->registerEnumElement("_headMode", "_headMode.RollingBallSearchRight", RollingBallSearchRight);

    pEngine->registerEnumElement("_attackMode", "_attackMode.kick_kick", kick_kick);
    pEngine->registerEnumElement("_attackMode", "_attackMode.dribble_kick", dribble_kick);


    //Game
    pEngine->registerBooleanInputSymbol("isReady", &currentFrame.isReady);
    pEngine->registerBooleanInputSymbol("isSet", &currentFrame.isSet);
    pEngine->registerBooleanInputSymbol("isGameStart", &currentFrame.isGameStart);
    pEngine->registerBooleanInputSymbol("isGameOver", &currentFrame.isGameOver);
    pEngine->registerDecimalInputSymbol("timeLeft", &currentFrame.timeLeft);
    pEngine->registerBooleanInputSymbol("isPathPlanningRequested", &currentFrame.Pathplan);
    pEngine->registerBooleanInputSymbol("isAttacker", &currentFrame.isAttacker);
    pEngine->registerBooleanInputSymbol("pause", &currentFrame.pause);
    pEngine->registerDecimalInputSymbol("sec_state", &currentFrame.sec_state);
    pEngine->registerDecimalInputSymbol("sec_state_info", &currentFrame.sec_state_info);
    pEngine->registerBooleanInputSymbol("isShooter", &currentFrame.isShooter);
    pEngine->registerBooleanInputSymbol("isPenalized", &currentFrame.isPenalized);
    pEngine->registerDecimalInputSymbol("secsTillUnpenalised", &currentFrame.secsTillUnpenalised);
    pEngine->registerDecimalInputSymbol("secondaryTime", &currentFrame.secondaryTime);
    pEngine->registerBooleanInputSymbol("isDropBall", &currentFrame.isDropBall);


    //Ball
    pEngine->registerBooleanInputSymbol("isBallSeen", &currentFrame.isBallSeen);
    pEngine->registerDecimalInputSymbol("ballRange", &currentFrame.ballRange);
    pEngine->registerDecimalInputSymbol("ballBearing", &currentFrame.ballBearing);
    pEngine->registerDecimalInputSymbol("ball.x", &currentFrame.ball_x);
    pEngine->registerDecimalInputSymbol("ball.y", &currentFrame.ball_y);
    pEngine->registerDecimalInputSymbol("ballLoc.World.x", &currentFrame.ballLoc_world_x);
    pEngine->registerDecimalInputSymbol("ballLoc.World.y", &currentFrame.ballLoc_world_y);
    pEngine->registerDecimalInputSymbol("lostBallDir", &currentFrame.lostBallDir);

    pEngine->registerBooleanInputSymbol("first_time_track_ball", &currentFrame.first_time_track_ball);
    pEngine->registerBooleanInputSymbol("ball_moved_while_moving", &currentFrame.ball_moved_while_moving);
    pEngine->registerDecimalInputSymbol("expected_ball_bearing", &currentFrame.expected_ball_bearing);
    pEngine->registerDecimalInputSymbol("ballFoundHeadAngleYaw", &currentFrame.ballFoundHeadAngleYaw);
    pEngine->registerBooleanInputSymbol("cannot_kick", &currentFrame.cannot_kick);
    pEngine->registerDecimalInputSymbol("front_distance", &currentFrame.front_distance);
    pEngine->registerDecimalInputSymbol("side_distance", &currentFrame.side_distance);

    pEngine->registerDecimalInputSymbol("rolling_ball_to_robot_time", &currentFrame.rolling_ball_to_robot_time);

    /// 2018
    pEngine->registerBooleanInputSymbol("is_within_tol", &currentFrame.is_within_tol);
    pEngine->registerDecimalInputSymbol("theta_start_walking", &currentFrame.theta_start_walking);
    pEngine->registerDecimalInputSymbol("tol_theta", &currentFrame.tol_theta);
    pEngine->registerDecimalInputSymbol("virtualGoalRange", &currentFrame.virtualGoalRange);
    pEngine->registerDecimalInputSymbol("virtualGoalBearing", &currentFrame.virtualGoalBearing);
    pEngine->registerDecimalInputSymbol("shoot_position_x", &currentFrame.shoot_position_x);
    pEngine->registerDecimalInputSymbol("shoot_position_y", &currentFrame.shoot_position_y);
    pEngine->registerDecimalInputSymbol("needed_turn_angle", &currentFrame.needed_turn_angle);
    pEngine->registerDecimalInputSymbol("shoot_position_theta", &currentFrame.shoot_position_theta);
    pEngine->registerDecimalInputSymbol("shoot_position_bearing", &currentFrame.shoot_position_bearing);
    pEngine->registerDecimalInputSymbol("shoot_position_range", &currentFrame.shoot_position_range);
    pEngine->registerDecimalInputSymbol("ballRangeToRightLeg", &currentFrame.ballRangeToRightLeg);

    //Goal
    pEngine->registerBooleanInputSymbol("isGoalSeen", &currentFrame.isGoalSeen);
    pEngine->registerDecimalInputSymbol("goalCenterRange", &currentFrame.goalCenterRange);  // should add the range as well, since we have it
    pEngine->registerDecimalInputSymbol("goalCenterBearing", &currentFrame.goalCenterBearing);
    pEngine->registerDecimalInputSymbol("goalLocLeft.World.x", &currentFrame.goalLocLeft_world_x);
    pEngine->registerDecimalInputSymbol("goalLocLeft.World.y", &currentFrame.goalLocLeft_world_y);
    pEngine->registerDecimalInputSymbol("goalLocRight.World.x", &currentFrame.goalLocRight_world_x);
    pEngine->registerDecimalInputSymbol("goalLocRight.World.y", &currentFrame.goalLocRight_world_y);
    pEngine->registerDecimalInputSymbol("goalFoundHeadAngleYaw", &currentFrame.goalFoundHeadAngleYaw);
    pEngine->registerDecimalInputSymbol("robot_goal_bearing_loc", &currentFrame.robot_goal_bearing_loc);
    pEngine->registerDecimalInputSymbol("robot_goal_bearing_odom", &currentFrame.robot_goal_bearing_odom);


    //Opponent
    pEngine->registerBooleanInputSymbol("isOpponentSeen", &currentFrame.isOpponentSeen);
    pEngine->registerDecimalInputSymbol("opponentRange", &currentFrame.opponentRange);
    pEngine->registerDecimalInputSymbol("opponentCenterBearing", &currentFrame.opponentCenterBearing);
    pEngine->registerDecimalInputSymbol("opponentCenter.World.x", &currentFrame.opponentCenter_world_x);
    pEngine->registerDecimalInputSymbol("opponentCenter.World.y", &currentFrame.opponentCenter_world_y);

    //Robot
    pEngine->registerDecimalInputSymbol("robotLoc.x", &currentFrame.robotLoc_x);
    pEngine->registerDecimalInputSymbol("robotLoc.y", &currentFrame.robotLoc_y);
    pEngine->registerDecimalInputSymbol("robotLoc.theta", &currentFrame.robotLoc_theta);
    pEngine->registerDecimalInputSymbol("locConfidence", &currentFrame.locConfidence);
    pEngine->registerDecimalInputSymbol("gyroBody", &currentFrame.gyroBody);
    pEngine->registerBooleanInputSymbol("is_robot_moving", &currentFrame.is_robot_moving);
    pEngine->registerDecimalInputSymbol("odom_orientation", &currentFrame.odom_orientation);


    pEngine->registerDecimalInputSymbol("kickDestTheta", &currentFrame.kickDestTheta); //not used now

    pEngine->registerDecimalInputSymbol("headAngleYaw", &currentFrame.headAngleYaw);
    pEngine->registerDecimalInputSymbol("headAnglePitch", &currentFrame.headAnglePitch);

    //Pose
    pEngine->registerBooleanInputSymbol("approachPose", &currentFrame.approach_pose);
    pEngine->registerBooleanInputSymbol("reachedPose", &currentFrame.reached_pose);
    pEngine->registerDecimalInputSymbol("robot_to_target_theta", &currentFrame.robot_to_target_theta);
    pEngine->registerDecimalInputSymbol("robot_to_target_range", &currentFrame.robot_to_target_range);


    //Remote switches
    pEngine->registerBooleanInputSymbol("remoteSwitch1", &currentFrame.remote_switch_1);
    pEngine->registerBooleanInputSymbol("remoteSwitch2", &currentFrame.remote_switch_2);
    pEngine->registerBooleanInputSymbol("remoteSwitch3", &currentFrame.remote_switch_3);
    pEngine->registerBooleanInputSymbol("remoteSwitch4", &currentFrame.remote_switch_4);

    //Output
    pEngine->registerDecimalOutputSymbol("kick_angle", &currentFrame.kickangle);
    pEngine->registerDecimalOutputSymbol("kick_speed", &currentFrame.kick_speed);
    pEngine->registerDecimalOutputSymbol("kick_leg", &currentFrame.kick_leg);
    pEngine->registerDecimalOutputSymbol("target_range", &currentFrame.target_range);
    pEngine->registerDecimalOutputSymbol("target_bearing", &currentFrame.target_bearing);
    pEngine->registerDecimalOutputSymbol("target_x", &currentFrame.target_x);
    pEngine->registerDecimalOutputSymbol("target_y", &currentFrame.target_y);
    pEngine->registerDecimalOutputSymbol("target_theta", &currentFrame.target_theta);
    pEngine->registerEnumeratedOutputSymbol("headMode", "_headMode", (int *)(&currentFrame.headMode));
    pEngine->registerEnumeratedOutputSymbol("lastBallDir", "_headMode", (int *)(&currentFrame.lastBallDir));
    pEngine->registerEnumeratedOutputSymbol("nextGoalDir", "_headMode", (int *)(&currentFrame.nextGoalDir));
    pEngine->registerDecimalOutputSymbol("ball_found_head_angle_yaw", &currentFrame.ball_found_head_angle_yaw);
    pEngine->registerDecimalOutputSymbol("goal_found_head_angle_yaw", &currentFrame.goal_found_head_angle_yaw);

    pEngine->registerBooleanOutputSymbol("first_kick", &currentFrame.first_kick);
    pEngine->registerDecimalOutputSymbol("turned", &currentFrame.turned);
    pEngine->registerBooleanOutputSymbol("no_refresh_ball", &currentFrame.no_refresh_ball);

    pEngine->registerDecimalOutputSymbol("stateCode", &currentFrame.state_code);

    pEngine->registerDecimalOutputSymbol("robot_goal_x", &currentFrame.robot_goal_x);
    pEngine->registerDecimalOutputSymbol("robot_goal_y", &currentFrame.robot_goal_y);
    pEngine->registerDecimalOutputSymbol("robot_goal_theta", &currentFrame.robot_goal_theta);

}

//// Change this to send the head mode over Serial
bool processHeadMode()
{


    if ((currentFrame.headMode != HorizontalTrack) &&
            (currentFrame.headMode != VerticalTrack) &&
            (currentFrame.headMode != BothTrack))
    {
        switch (currentFrame.headMode)
        {
        case FarLeft:
            currentFrame.headAngleYaw = 90;
            currentFrame.headAnglePitch = 25;
            break;

        case FarMid:
            currentFrame.headAngleYaw = 0;
            currentFrame.headAnglePitch = 25;
            break;

        case FarRight:
            currentFrame.headAngleYaw = -90;
            currentFrame.headAnglePitch = 25;
            break;

        case FarRightBack:
            currentFrame.headAngleYaw = -135;
            currentFrame.headAnglePitch = 25;
            break;

        case FarLeftBack:
            currentFrame.headAngleYaw = 135;
            currentFrame.headAnglePitch = 25;
            break;

        case CloseLeft:
            currentFrame.headAngleYaw = 90;
            currentFrame.headAnglePitch = 57;
            break;

        case CloseMid:
            currentFrame.headAngleYaw = 0;
            currentFrame.headAnglePitch = 57;
            break;

        case CloseRight:
            currentFrame.headAngleYaw = -90;
            currentFrame.headAnglePitch = 57;
            break;

        case CloseRightBack:
            currentFrame.headAngleYaw = -135;
            currentFrame.headAnglePitch = 57;
            break;

        case CloseLeftBack:
            currentFrame.headAngleYaw = 135;
            currentFrame.headAnglePitch = 57;
            break;

        case RollingBallLeft:
            currentFrame.headAngleYaw = 10;
            currentFrame.headAnglePitch = 45;
            break;

        case RollingBallRight:
            currentFrame.headAngleYaw = -16;
            currentFrame.headAnglePitch = 45;
            break;

        case RollingBallSearchLeft:
            currentFrame.headAngleYaw = 60;
            currentFrame.headAnglePitch = 25;
            break;

        case RollingBallSearchRight:
            currentFrame.headAngleYaw = -60;
            currentFrame.headAnglePitch = 25;
            break;
        }
    }
    else
    {
        //            // Calculate expected ball bearing

        //            // x,y,theta 4,5,6
        //            if (broadcast.InputPacket[5] == 0)
        //            {
        //                currentFrame.expected_ball_bearing = broadcast.InputPacket[6] - asin(broadcast.InputPacket[4] / currentFrame.ballRange
        //                        *sin(-currentFrame.headAngleYaw/180*M_PI));
        //            }
        //            else
        //            {
        //                currentFrame.expected_ball_bearing = broadcast.InputPacket[6] - asin(sqrt(pow(broadcast.InputPacket[4],2)+pow(broadcast.InputPacket[5],2)) / currentFrame.ballRange
        //                        *sin(M_PI/2-currentFrame.headAngleYaw/180*M_PI-atan(broadcast.InputPacket[4]/broadcast.InputPacket[5])));
        //            }

        //            cout << "ballRange" << currentFrame.ballRange << endl;
        //            cout << "ballBearing" << currentFrame.ballBearing << endl;
        //            cout << "expected_ball_bearing" << currentFrame.expected_ball_bearing << endl;

        if (currentFrame.headMode == BothTrack)
        {
            //                if ( (fabs(currentFrame.ballBearing - currentFrame.expected_ball_bearing )*180/M_PI > 15) && currentFrame.isBallSeen == 1)  //if more than 10 degrees to the right, turn head right
            //                {
            //                    currentFrame.ball_moved_while_moving = true;
            //                }
            //                else
            //                {
            //                    currentFrame.ball_moved_while_moving = false;
            //                }


            //                if (currentFrame.first_time_track_ball)
            //                {
            //                    currentFrame.headAngleYaw += currentFrame.ballBearingCamera;
            //                    currentFrame.first_time_track_ball = false;
            //                }
            //                else if (fabs(last_robot_orientation - broadcast.InputPacket[6])/M_PI*180 > 7)
            //                {
            //                    broadcast.OutputPacket[4] += (last_robot_orientation - broadcast.InputPacket[6])/M_PI*180;
            //                }
            //                else
            if (((currentFrame.ballBearingCamera*180/M_PI) > 20) && (currentFrame.isBallSeen))  //if more than 10 degrees to the right, turn head right
            {
                currentFrame.headAngleYaw += 1.5;
                //                if(currentFrame.headAnglePitch < -40)
                //                    broadcast.OutputPacket[4] -= 5;
                //                else
                //                    broadcast.OutputPacket[4] -= 10;
            }
            else if (((currentFrame.ballBearingCamera*180/M_PI) < -20) && (currentFrame.isBallSeen))
            {
                currentFrame.headAngleYaw -= 1.5;
                //                if(currentFrame.headAnglePitch < -40)
                //                    broadcast.OutputPacket[4] += 5;
                //                else
                //                    broadcast.OutputPacket[4] += 10;
            }
            else
            {
                currentFrame.headAngleYaw += 0;
            }
            //Safety
            if (fabs(currentFrame.headAngleYaw) >70)
            {
                if (currentFrame.headAngleYaw>0)
                    currentFrame.headAngleYaw = 70;
                else
                    currentFrame.headAngleYaw = -70;
            }

            if (currentFrame.ballRange > 1.2 && currentFrame.isBallSeen == 1)  //if range more that 1 meter, head up
            {

                currentFrame.headAnglePitch = 25;
            }
            else if (currentFrame.ballRange <= 1.0 && currentFrame.ballRange >= 0 && currentFrame.isBallSeen == 1)
            {

                currentFrame.headAnglePitch = 57;
            }
            else
            {
                currentFrame.headAnglePitch += 0;
            }

        }


        if (currentFrame.headMode == HorizontalTrack)
        {
            if (((currentFrame.ballBearingCamera*180/M_PI) > 20) && (currentFrame.isBallSeen))  //if more than 10 degrees to the right, turn head right
            {
                currentFrame.headAngleYaw += 1.5;
            }
            else if (((currentFrame.ballBearingCamera*180/M_PI) < -20) && (currentFrame.isBallSeen))
            {
                currentFrame.headAngleYaw -= 1.5;

            }
            else
            {
                currentFrame.headAngleYaw += 0;
            }
            //Safety
            if (fabs(currentFrame.headAngleYaw) >135)
            {
                if (currentFrame.headAngleYaw>0)
                    currentFrame.headAngleYaw = 135;
                else
                    currentFrame.headAngleYaw = -135;
            }

        }
    }

    currentFrame.head_angle_yaw_list.push_back(currentFrame.headAngleYaw);


    ROS_INFO("Pitch_request: %f, Yaw_request: %f",currentFrame.headAnglePitch, currentFrame.headAngleYaw);

    //        last_robot_orientation = broadcast.InputPacket[6];
}
