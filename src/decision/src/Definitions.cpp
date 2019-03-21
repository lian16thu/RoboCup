#include "Definitions.h"




// Callback functions for the received messages: goalpost, game controller, ball, opponent, head gyro, and localization
void CBOnGoalReceived(const vision::Goalpost::ConstPtr &msg)
{
    currentFrame.received_goalpost = *msg;

    //if ((currentFrame.received_goalpost.goalpost_number == 2)||(currentFrame.received_goalpost.goalpost_number == 3))
    if ((currentFrame.received_goalpost.goalpost_number > 0) && (currentFrame.headAnglePitch == 25))
    {
        currentFrame.isGoalSeen = true;
        currentFrame.foundGoalTime = ros::Time::now().toSec();


        //    ROS_INFO("goalFoundHeadAngleYaw: error check ");


        //    ROS_INFO("goalFoundHeadAngleYaw:%f with delay:%f ",currentFrame.goalFoundHeadAngleYaw,
        //             currentFrame.head_angle_yaw_list.size() - trunc(ros::Time::now().toSec()-currentFrame.received_goalpost.header.stamp.toSec()));

        if (((currentFrame.head_angle_yaw_list.size() - ceil(MAIN_NODE_RUNNING_RATE*(ros::Time::now().toSec()-currentFrame.received_goalpost.header.stamp.toSec()))) >10) )

        {
            currentFrame.goalFoundHeadAngleYaw = currentFrame.head_angle_yaw_list.at(currentFrame.head_angle_yaw_list.size() -
                                                                                     ceil(MAIN_NODE_RUNNING_RATE*(ros::Time::now().toSec()-currentFrame.received_goalpost.header.stamp.toSec())) -1);
        }
        else
        {
            currentFrame.goalFoundHeadAngleYaw = 0;
        }



        ROS_INFO("goalFoundHeadAngleYaw:%f with delay:%f ",currentFrame.goalFoundHeadAngleYaw,
                 ros::Time::now().toSec()-currentFrame.received_goalpost.header.stamp.toSec());
    }
}

void CBOnControllingReceived(const gamecontroller::gameControl::ConstPtr &msg)
{
    currentFrame.received_gamecontrol = *msg;
    //    ROS_INFO("Received from Gamecontroler:State:%d,Sec_State:%d,time:%d",
    //             msg->state,msg->secondaryState,msg->secsRemaining);
}

void CBOnBallReceived(const vision::Ball::ConstPtr &msg)
{
    currentFrame.received_ball = *msg;

    if (currentFrame.received_ball.ball_range < 9)
    {
        currentFrame.isBallSeen = true;

        currentFrame.foundBallTime = ros::Time::now().toSec();

        //ROS_INFO("ballFoundHeadAngleYaw: error check ");

        if ((currentFrame.head_angle_yaw_list.size() - ceil(MAIN_NODE_RUNNING_RATE*(ros::Time::now().toSec()-currentFrame.received_ball.header.stamp.toSec()))) >10 )
        {
            currentFrame.ballFoundHeadAngleYaw = currentFrame.head_angle_yaw_list.at(currentFrame.head_angle_yaw_list.size() -
                                                                                     ceil(MAIN_NODE_RUNNING_RATE*(ros::Time::now().toSec()-currentFrame.received_ball.header.stamp.toSec())) -1);
        }
        else
        {
            currentFrame.ballFoundHeadAngleYaw = 0;
        }

        ROS_INFO("ballFoundHeadAngleYaw:%f with delay:%f ",currentFrame.ballFoundHeadAngleYaw,
                 ros::Time::now().toSec()-currentFrame.received_ball.header.stamp.toSec());
        double ball_distance_x ;
        double ball_distance_y ;

    }



    //currentFrame.cannot_kick = !(kick_check(currentFrame.received_ball.ball_center_x, currentFrame.received_ball.ball_center_y));
    //if (currentFrame.cannot_kick)
    //{
    //    currentFrame.ball_image_x = currentFrame.received_ball.ball_center_x; // ball_x
    //    currentFrame.ball_image_y = currentFrame.received_ball.ball_center_y; // ball_y
    //}

    //currentFrame.front_distance = kick_check_x(currentFrame.received_ball.ball_center_x, currentFrame.received_ball.ball_center_y);
    //currentFrame.side_distance = kick_check_y(currentFrame.received_ball.ball_center_x, currentFrame.received_ball.ball_center_y);
}

void CBOnOpponentsReceived(const vision::Opponents::ConstPtr &msg)
{
    currentFrame.received_opponents = *msg;
}


void CBOnLocalizationReceived(const localization::OutputData::ConstPtr &msg)
{
    currentFrame.received_localization = *msg;

    currentFrame.last_received_loc_time = msg->header.stamp.toSec();

    ROS_INFO("Received localization: x=%f, y=%f, theta=%f, conf=%f",
             currentFrame.received_localization.robotPose.x,currentFrame.received_localization.robotPose.y,
             currentFrame.received_localization.robotPose.theta,currentFrame.received_localization.robotPoseConfidence);

}



// Function that refreshes all the data received by the callback functions
void DataStructure::FlushData()
{

    //this->kickDestTheta = lastFrame.kickDestTheta;//keep the value, this value is changed in the service "angle"
    lastFrame = *this;

    ///Game
    this->timeLeft = static_cast<double>(received_gamecontrol.secsRemaining);

    this->isReady = received_gamecontrol.state == 1;
    this->isSet = received_gamecontrol.state == 2;
    this->isGameStart = received_gamecontrol.state == 3;
    this->isGameOver = received_gamecontrol.state == 4; /// need to change this because rules have changed!!!
    // Need to see how to add the secondary states
    //    if (((this->isAttacker) && !(static_cast<bool>(received_gamecontrol.kickOffTeam == 29))) ||
    //            (!(this->isAttacker) && (static_cast<bool>(received_gamecontrol.kickOffTeam ==29))) )
    //        this->attacker_changed = true;
    //    else
    //        this->attacker_changed = false;
    this->isAttacker =  static_cast<bool>(received_gamecontrol.kickOffTeam ==29);
    // STATE2_NORMAL               0
    // STATE2_PENALTYSHOOT         1
    // STATE2_OVERTIME             2
    // STATE2_TIMEOUT              3
    // STATE2_DIRECT_FREEKICK      4
    // STATE2_INDIRECT_FREEKICK    5
    // STATE2_PENALTYKICK          6
    if ((received_gamecontrol.secondaryState == 1) || (received_gamecontrol.secondaryState == 3) || (received_gamecontrol.secondaryState == 4) ||
            (received_gamecontrol.secondaryState == 5) || (received_gamecontrol.secondaryState == 6))
        this->pause = true;
    else
        this->pause = false;
    this->sec_state = received_gamecontrol.secondaryState;
    this->sec_state_info = received_gamecontrol.secondaryStateInfo;
    this->isShooter = static_cast<bool>(received_gamecontrol.secondaryStateTeam ==29);
    this->isPenalized = static_cast<bool>(!(received_gamecontrol.penalty == 0));
    this->secsTillUnpenalised = static_cast<double>(received_gamecontrol.secsTillUnpenalised);
    this->secondaryTime = static_cast<double>(received_gamecontrol.secondaryTime);

    ROS_ERROR("Previous penalized:%d, current penalized:%d, previous initial:%d, previous ready:%d, current ready:%d",
              currentFrame.previous_penalized,this->isPenalized,currentFrame.previous_initial,currentFrame.previous_ready,this->isReady);

    if (!currentFrame.previous_penalized && this->isPenalized)
    {
        //Go outside
        if (currentFrame.attack_left)
            this->loc_init_pose = -1;
        else
            this->loc_init_pose = 1;
        currentFrame.init_pose_changed = true;
    }
    else if (currentFrame.previous_initial && this->isReady && !this->isAttacker)
    {
        // Start from outside
        if (currentFrame.attack_left)
            this->loc_init_pose = -1;
        else
            this->loc_init_pose = 1;

        currentFrame.init_pose_changed = true;
    }
    else if ((currentFrame.previous_initial && this->isReady && this->isAttacker) ||
             (!currentFrame.previous_ready && this->isReady && this->isAttacker))
    {
        // Start from center
        this->loc_init_pose = 0;

        currentFrame.init_pose_changed = true;
    }
    else if (!currentFrame.previous_ready && this->isReady && received_gamecontrol.kickOffTeam == 128)
    {
        // Start from outside
        if (currentFrame.attack_left)
            this->loc_init_pose = -1;
        else
            this->loc_init_pose = 1;

        currentFrame.init_pose_changed = true;

    }

    currentFrame.isDropBall = received_gamecontrol.kickOffTeam == 128;


    this->previous_penalized = this->isPenalized;
    this->previous_initial = received_gamecontrol.state == 0;
    this->previous_ready = this->isReady;

    ///Ball
    if (currentFrame.isBallSeen == true)
    {
        if (ros::Time::now().toSec() - currentFrame.foundBallTime > 1) //
        {
            this->isBallSeen = false;
        }
    }

    if (!this->no_refresh_ball) {
        this->ballBearingCamera = static_cast<double>(received_ball.ball_bearing);
        //this->ballBearing = static_cast<double>(received_ball.ball_bearing) + currentFrame.ballFoundHeadAngleYaw/180*M_PI ;
        this->ballBearing = static_cast<double>(received_ball.ball_bearing);
        this->ballRange = static_cast<double>(received_ball.ball_range);  //as agreed with 2D

        this->ball_x = this->ballRange * cos(this->ballBearing);
        this->ball_y = this->ballRange * sin(this->ballBearing);

        this->ballRangeToRightLeg = sqrt(pow(this->ballRange*sin(this->ballBearing) - 0.1025,2) +
                                         pow(this->ballRange*cos(this->ballBearing),2));


        this->ballLoc_world_x = static_cast<double>(received_localization.ballCenterOnField.x);
        this->ballLoc_world_y = static_cast<double>(received_localization.ballCenterOnField.y);
        this->ball_image_x = static_cast<double>(received_ball.ball_center_x);
        this->ball_image_y = static_cast<double>(received_ball.ball_center_y);

    }

    if (static_cast<double>(received_ball.kick_time)<30 && static_cast<double>(received_ball.kick_time)>0)
    {

        this->rolling_ball_to_robot_time = static_cast<double>(received_ball.kick_time);
        ROS_INFO("Rolling ball is coming in :%f",this->rolling_ball_to_robot_time);
        ROS_ERROR("Rolling ball is coming in :%f",this->rolling_ball_to_robot_time);
    }

    cout << "ball_image_x:" << this->ball_image_x << "ball_image_y:" << this->ball_image_y << "ballBearingCamera:" << this->ballBearingCamera << " ballBearing:" << this->ballBearing << endl << " ballRange:" << this->ballRange << endl;
    cout << "ballLoc_world_x:" << this->ballLoc_world_x << "ballLoc_world_y:" << this->ballLoc_world_y << endl;

    ///Goal
    if (currentFrame.isGoalSeen == true)
    {
        if ((ros::Time::now().toSec() - currentFrame.foundGoalTime > 5)) //Delay to lose the goal
        {
            this->isGoalSeen = false;
        }
    }


    //    if ((currentFrame.received_goalpost.goalpost_number == 2)||(currentFrame.received_goalpost.goalpost_number == 3))
    //    {

    //            this->goalCenterBearing = (static_cast<double>(received_goalpost.goalpost_left_bearing) + static_cast<double>(received_goalpost.goalpost_right_bearing)) / 2;
    //            this->goalCenterRange = (static_cast<double>(received_goalpost.goalpost_left_range) + static_cast<double>(received_goalpost.goalpost_right_range)) / 2;

    //    }

    if (currentFrame.isGoalSeen  && (currentFrame.headAnglePitch == 25))
    {
        this->goalCenterBearing = (static_cast<double>(received_goalpost.goalpost_center_bearing)) ;
        this->goalCenterRange = (static_cast<double>(received_goalpost.goalpost_center_range)) ;
    }

    this->goalLocLeft_world_x = 4.5;
    this->goalLocLeft_world_y = -1.3;
    this->goalLocRight_world_x = 4.5;
    this->goalLocRight_world_y = 1.3;


    //    this->robot_goal_bearing_odom =90- atan2((4.5-_odom_x),(_odom_y-0)) -_odom_orientation;


    //    ROS_INFO("Odom x:%f,y:%f,orient:%f,robot_goal_bearing_odom:%f, test1:%f",_odom_x,_odom_y,_odom_orientation,this->robot_goal_bearing_odom,
    //             test1);



    /// 2018


    double delta_x = 4.5 - currentFrame.received_localization.robotPose.x;
    double delta_y = 0 - currentFrame.received_localization.robotPose.y;
    this->virtualGoalRange = sqrt(pow(delta_x,2) + pow(delta_y,2));
    double cosVirtualGoalBearing = delta_x / this->virtualGoalRange;
    this->virtualGoalBearing = acos(cosVirtualGoalBearing);
    bool is_left = this->ballRange * cos(this->ballBearing) < this->virtualGoalRange * cosVirtualGoalBearing;
    double R = is_left ? this->ballRange / (4 * sin(this->ballBearing)) : this->ballRange / (2 * sin(this->ballBearing));      // radius of arc walking
    double theta_by_R = is_left ? asin(this->ballRange / (4 * R_TOL)) : asin(this->ballRange / (2 * R_TOL));
    // theta_start_walking = min(THETA_TOL, theta_by_R);

    this->theta_start_walking = THETA_TOL;

    // is_within_tol = (abs(ballBearing) < THETA_TOL) && (R > R_TOL);  check this

    if ((fabs(this->ballBearing) < THETA_TOL) && (this->isBallSeen))
        this->is_within_tol = true;         // true if the angle between the body and the direction toward the ball is small enough
    else
        this->is_within_tol = false;

    double new_angle = 0;
    new_angle = this->ballBearing>0 ? this->ballBearing + 10/180*M_PI : this->ballBearing - 10/180*M_PI;

    const float maxKickDistance = 0.5;
    float kickDistance = maxKickDistance < ballRange ? maxKickDistance : ballRange;
    float ballDistanceToGoal = sqrt(pow(ballRange, 2) + pow(goalCenterRange, 2) - 2*ballRange*goalCenterRange*cos(ballBearing - goalCenterBearing));
    float alpha = asin(ballRange * sin(ballBearing - goalCenterBearing) / ballDistanceToGoal);
    this->shoot_position_theta = -alpha + goalCenterBearing;
    this->shoot_position_y = -kickDistance * sin(shoot_position_theta) + ballRange * sin(ballBearing) + 0.10; // to adjust for the difference between the center and the right foot
    this->shoot_position_x = ballRange * cos(ballBearing) - kickDistance * cos(shoot_position_theta);

    this->needed_turn_angle = fabs(atan2(this->shoot_position_y,this->shoot_position_x) - this->shoot_position_theta);

    if (this->shoot_position_x < 0)
        this->shoot_position_x = 0;

    if (this->shoot_position_theta >2*M_PI)
        this->shoot_position_theta += -2*M_PI;
    else if (this->shoot_position_theta < -2*M_PI)
        this->shoot_position_theta += 2*M_PI;

    // shoot position based on localization
    float goalLocCenter_world_x = (goalLocLeft_world_x + goalLocRight_world_x) / 2;
    float goalLocCenter_world_y = (goalLocLeft_world_y + goalLocRight_world_y) / 2;
    ballDistanceToGoal = sqrt(pow(ballLoc_world_x - goalLocCenter_world_x, 2) + pow(ballLoc_world_y - goalLocCenter_world_y, 2));
    this->shoot_position_world_x = goalLocCenter_world_x + (ballLoc_world_x - goalLocLeft_world_x) * (ballDistanceToGoal + kickDistance) / ballDistanceToGoal;
    this->shoot_position_world_y = goalLocCenter_world_y + (ballLoc_world_y - goalLocLeft_world_y) * (ballDistanceToGoal + kickDistance) / ballDistanceToGoal;
    this->shoot_position_range = sqrt(pow(shoot_position_world_x - robotLoc_x, 2) + pow(shoot_position_world_y - robotLoc_y, 2));
    this->shoot_position_bearing = atan2(shoot_position_world_y - robotLoc_y, shoot_position_world_x - robotLoc_x);

    if (this->locConfidence > 0.5) {
        this->shoot_position_x = shoot_position_range * cos(shoot_position_bearing);
        this->shoot_position_y = shoot_position_range * sin(shoot_position_bearing);
    }

    /// End 2018





    //Opponents
    // For one opponent now
    this->isOpponentSeen = static_cast<bool>(received_opponents.opponent_detected);
    if (this->isOpponentSeen)
    {
        this->opponentCenterBearing = static_cast<double>(received_opponents.opponent_bearing[0]);
        this->opponentRange = static_cast<double>(received_opponents.opponent_range[0]);
        cout << "opponentBearing:" << this->opponentCenterBearing << endl << "opponentRange:" << this->opponentRange << endl;
        this->opponentCenter_world_x = static_cast<double>(received_localization.opponentCenterOnField.x);
        this->opponentCenter_world_y = static_cast<double>(received_localization.opponentCenterOnField.y);
    }

    // Random opponent
    int opponent_detected_rand = rand() % 100;
    if (opponent_detected_rand>33)
        this->isOpponentSeen = true;
    else
        this->isOpponentSeen = false;
    int opponent_bearing_rand = rand() % 100;
    if (opponent_bearing_rand > 49)
        this->opponentCenterBearing = 1;
    else
        this->opponentCenterBearing = -1;


    //Robot
    this->robotLoc_x = static_cast<double>(received_localization.robotPose.x);
    this->robotLoc_y = static_cast<double>(received_localization.robotPose.y);
    this->robotLoc_theta = static_cast<double>(received_localization.robotPose.theta);
    this->locConfidence = static_cast<double>(received_localization.robotPoseConfidence);

    ///2018

    this->robot_to_target_theta = atan2(currentFrame.robot_goal_y - this->robotLoc_y,currentFrame.robot_goal_x - this->robotLoc_x)
            - this->robotLoc_theta;

    this->robot_to_target_range = sqrt(pow(currentFrame.robot_goal_y - this->robotLoc_y,2)
                                       + pow(currentFrame.robot_goal_x - this->robotLoc_x,2));

    //atan because we don't want to estimate based on direction
    this->robot_goal_bearing_loc = atan(this->robotLoc_y/(this->robotLoc_x-4.5));

    /// End of 2018

    //#endif
}

// Function that prints out the game state, the robot perceived state, and if the ball and goal were seen
void DataStructure::PrintReceivedData(void)
{
    console_strstream << "game state: ";

    switch (currentFrame.received_gamecontrol.state)
    {
    case 0:
        console_strstream << "UnKnown" << endl;
        break;
    case 1:
        console_strstream << "Ready" << endl;
        break;
    case 2:
        console_strstream << "Set" << endl;
        break;
    case 3:
        console_strstream << "Play" << endl;
        break;
    case 4:
        console_strstream << "Finish" << endl;
        break;
    default:
        console_strstream << "UnKnown" << endl;
        break;
    }   //////////////////////////////////////////////////match
    if(currentFrame.isReady == true)
        console_strstream << "Ready" <<endl;
    else
        console_strstream << "UnKnown" <<endl;
    if(currentFrame.isGameStart == true)
        console_strstream << "Play" <<endl;
    else
        console_strstream << "UnKnown" <<endl;
    if(currentFrame.isGameOver == true)
        console_strstream << "Finish" <<endl;
    else
        console_strstream << "UnKnown" <<endl;
    ///////////////////////////////////////////////////////////debug

    console_strstream << "time: " << timeLeft << endl;

    console_strstream <<"kickangle:  " << kickangle << endl;
    console_strstream << "isBallSeen: " << (isBallSeen ? "true" : "false") << endl;
    if (!isBallSeen)
    {
    }
    else
    {
        console_strstream << "ballRange: " << ballRange << "ballBearing: "  << ballBearing << "ballBearingCamera: " << ballBearingCamera  << "within_tol: " << is_within_tol << endl;
    }

    console_strstream << "isGoalSeen: " << (isGoalSeen ? "true" : "false") << endl;
    if (!isGoalSeen)
    {
    }
    else
    {
        console_strstream << "goalRange: " << goalCenterRange << "goalBearing: " << goalCenterBearing << endl;
    }

    console_strstream << "isOpponentSeen: " << (isOpponentSeen ? "true" : "false") << endl;
    if (!isOpponentSeen)
    {
    }
    else
    {
        console_strstream << "opponentRange: " << opponentRange << " opponentBearing: " << opponentCenterBearing << endl;
    }

    console_strstream << "locConfidence: " << locConfidence << " robot_moved: " << robot_moved << endl;

    console_strstream << "isAttacker: " << (isAttacker ? "true" : " false") << "kick_angle: " << kickangle << endl;

    console_strstream << "is_robot_moving: " << (is_robot_moving ? "true" : "false") << "sec_state: " << sec_state << endl;


    console_strstream << "target_range: " << target_range << " target_bearing: " << target_bearing << endl;
    console_strstream << " target_x: " << target_x << " target_y: " << target_y << " target_theta: " << target_theta << endl;
    console_strstream << "gyroBody: " << gyroBody << " odom_orientation: " << odom_orientation << endl;

    console_strstream << "ballFoundHeadAngleYaw: " << ballFoundHeadAngleYaw << " goalFoundHeadAngleYaw: " << goalFoundHeadAngleYaw << endl;
    console_strstream << "cannot_kick: " << cannot_kick << "needed_turn_angle: " << needed_turn_angle << endl;

    console_strstream << "robotLoc_x: " << currentFrame.robotLoc_x << " robotLoc_y: " << currentFrame.robotLoc_y << " robotLoc_theta: "
                      << (currentFrame.robotLoc_theta/M_PI*180) << " locConfidence: " << currentFrame.locConfidence << endl;
    console_strstream << "robot_goal_x: " << currentFrame.robot_goal_x << " robot_goal_y: " << currentFrame.robot_goal_y << " robot_goal_theta: "
                      << (currentFrame.robot_goal_theta/M_PI*180) << endl;
    console_strstream << " robot_to_target_range: " << currentFrame.robot_to_target_range << " robot_to_target_theta: " << currentFrame.robot_to_target_theta << endl;
    console_strstream << "odom_x: " << currentFrame.odom_x << " odom_y: " << currentFrame.odom_y << " odom_orientation: "
                      << (currentFrame.odom_orientation/M_PI*180) << endl;
    console_strstream << " state_code: " << currentFrame.state_code << " robot_moving: " << currentFrame.is_robot_moving << " rolling_ball_to_robot_time: " << currentFrame.rolling_ball_to_robot_time << endl;
    console_strstream << " robot_goal_bearing_loc: " << currentFrame.robot_goal_bearing_loc << endl;



}


//Function that corrects the body gyro angle and just calculates the difference from the initial yaw angle
double dealWithBodyYawTheta(double inputYawTheta)
{
    if (isInitializing)
    {
        return inputYawTheta;
    }
    else
    {
        double result;
        result = inputYawTheta - currentFrame.gyroInitBodyYawTheta;
        if (result > 180.0)
        {
            result -= 360.0;
        }
        else
        {
            if (result <= -180.0)
            {
                result += 360.0;
            }
        }
        return result;
    }
}

DataStructure currentFrame;
DataStructure lastFrame;
double gyroInitYawTheta;

bool kick_check(double camera_x , double camera_y){
    double r_kick_x = (-0.2142 * camera_y + 112.016)/100.0;
    double r_kick_y = (-0.216 * camera_x + 65.793)/100.0;
    bool cankick = true;
    double x_distance, y_distance;

    //To save time, if can not kick, y_distance is the distance of r_kick_y to
    //the nearest y we can kick.
    if(r_kick_y > -0.05 && r_kick_y <= 0) {
        y_distance = 0.0;
    }
    else if(r_kick_y >= -0.2 && r_kick_y <= -0.05) {
        y_distance = 0.0;
    }
    else if(r_kick_y < -0.2) {
        y_distance = -0.2 - r_kick_y;
        cankick  = false;
    }
    else if(r_kick_y > 0 && r_kick_y < 0.05) {
        y_distance = 0.0;
    }
    else if(r_kick_y >= 0.05 && r_kick_y <= 0.2) {
        y_distance = 0.0;
    }
    else if(r_kick_y > 0.2) {
        y_distance = 0.2 - r_kick_y;
        cankick = false;
    }

    //x_distance should be >0 ,unless an error occured in vision
    if(r_kick_x < 0){
        x_distance = r_kick_x - 0.23;
        cankick = false;
    }

    //To kick far, if can not kick, x_distance is the distance of r_kick_x to 0.33
    if(r_kick_x < 0.23 && r_kick_x >= 0) {
        x_distance = 0.0;
    }
    else if(r_kick_x >= 0.23 && r_kick_x <= 0.38) {
        x_distance = 0.0;
    }
    else if(r_kick_x > 0.38 && r_kick_x <= 0.43) {
        if(abs(r_kick_y) <= 0.09) {
            x_distance = 0.0;
        }
        else if(abs(r_kick_y) > 0.09 && abs(r_kick_y) <= 0.2) {
            if(r_kick_x > (-0.455 * abs(r_kick_y) + 0.471)) {
                x_distance = r_kick_x - (-0.455 * abs(r_kick_y) + 0.471);
                cankick = false;
            }
            x_distance = 0.0;
        }
        else if (abs(r_kick_y) > 0.2){
            x_distance = r_kick_x - 0.38;
            cankick = false;
        }
    }
    else if(r_kick_x > 0.43) {
        if(abs(r_kick_y) <= 0.09){
            x_distance = r_kick_x - 0.43;
            cankick = false;
        }
        else if(abs(r_kick_y) > 0.09 && abs(r_kick_y) <= 0.2){
            x_distance = r_kick_x - (-0.455 * abs(r_kick_y) + 0.471);
            cankick = false;
        }
        else if(abs(r_kick_y) > 0.2){
            x_distance = r_kick_x - 0.38;
            cankick = false;
        }
    }

    return cankick;
}

double kick_check_x(double camera_x , double camera_y){
    double r_kick_x = (-0.2142 * camera_y + 112.016)/100.0;
    double r_kick_y = (-0.216 * camera_x + 65.793)/100.0;
    bool cankick = true;
    double x_distance, y_distance;

    //To save time, if can not kick, y_distance is the distance of r_kick_y to
    //the nearest y we can kick.
    if(r_kick_y > -0.05 && r_kick_y <= 0) {
        y_distance = 0.0;
    }
    else if(r_kick_y >= -0.2 && r_kick_y <= -0.05) {
        y_distance = 0.0;
    }
    else if(r_kick_y < -0.2) {
        y_distance = -0.2 - r_kick_y;
        cankick  = false;
    }
    else if(r_kick_y > 0 && r_kick_y < 0.05) {
        y_distance = 0.0;
    }
    else if(r_kick_y >= 0.05 && r_kick_y <= 0.2) {
        y_distance = 0.0;
    }
    else if(r_kick_y > 0.2) {
        y_distance = 0.2 - r_kick_y;
        cankick = false;
    }

    //x_distance should be >0 ,unless an error occured in vision
    if(r_kick_x < 0){
        x_distance = r_kick_x - 0.23;
        cankick = false;
    }

    //To kick far, if can not kick, x_distance is the distance of r_kick_x to 0.33
    if(r_kick_x < 0.23 && r_kick_x >= 0) {
        x_distance = 0.0;
    }
    else if(r_kick_x >= 0.23 && r_kick_x <= 0.38) {
        x_distance = 0.0;
    }
    else if(r_kick_x > 0.38 && r_kick_x <= 0.43) {
        if(abs(r_kick_y) <= 0.09) {
            x_distance = 0.0;
        }
        else if(abs(r_kick_y) > 0.09 && abs(r_kick_y) <= 0.2) {
            if(r_kick_x > (-0.455 * abs(r_kick_y) + 0.471)) {
                x_distance = r_kick_x - (-0.455 * abs(r_kick_y) + 0.471);
                cankick = false;
            }
            x_distance = 0.0;
        }
        else if (abs(r_kick_y) > 0.2){
            x_distance = r_kick_x - 0.38;
            cankick = false;
        }
    }
    else if(r_kick_x > 0.43) {
        if(abs(r_kick_y) <= 0.09){
            x_distance = r_kick_x - 0.43;
            cankick = false;
        }
        else if(abs(r_kick_y) > 0.09 && abs(r_kick_y) <= 0.2){
            x_distance = r_kick_x - (-0.455 * abs(r_kick_y) + 0.471);
            cankick = false;
        }
        else if(abs(r_kick_y) > 0.2){
            x_distance = r_kick_x - 0.38;
            cankick = false;
        }
    }

    return x_distance;
}

double kick_check_y(double camera_x , double camera_y){
    double r_kick_x = (-0.2142 * camera_y + 112.016)/100.0;
    double r_kick_y = (-0.216 * camera_x + 65.793)/100.0;
    bool cankick = true;
    double x_distance, y_distance;

    //To save time, if can not kick, y_distance is the distance of r_kick_y to
    //the nearest y we can kick.
    if(r_kick_y > -0.05 && r_kick_y <= 0) {
        y_distance = 0.0;
    }
    else if(r_kick_y >= -0.2 && r_kick_y <= -0.05) {
        y_distance = 0.0;
    }
    else if(r_kick_y < -0.2) {
        y_distance = -0.2 - r_kick_y;
        cankick  = false;
    }
    else if(r_kick_y > 0 && r_kick_y < 0.05) {
        y_distance = 0.0;
    }
    else if(r_kick_y >= 0.05 && r_kick_y <= 0.2) {
        y_distance = 0.0;
    }
    else if(r_kick_y > 0.2) {
        y_distance = 0.2 - r_kick_y;
        cankick = false;
    }

    //x_distance should be >0 ,unless an error occured in vision
    if(r_kick_x < 0){
        x_distance = r_kick_x - 0.23;
        cankick = false;
    }

    //To kick far, if can not kick, x_distance is the distance of r_kick_x to 0.33
    if(r_kick_x < 0.23 && r_kick_x >= 0) {
        x_distance = 0.0;
    }
    else if(r_kick_x >= 0.23 && r_kick_x <= 0.38) {
        x_distance = 0.0;
    }
    else if(r_kick_x > 0.38 && r_kick_x <= 0.43) {
        if(abs(r_kick_y) <= 0.09) {
            x_distance = 0.0;
        }
        else if(abs(r_kick_y) > 0.09 && abs(r_kick_y) <= 0.2) {
            if(r_kick_x > (-0.455 * abs(r_kick_y) + 0.471)) {
                x_distance = r_kick_x - (-0.455 * abs(r_kick_y) + 0.471);
                cankick = false;
            }
            x_distance = 0.0;
        }
        else if (abs(r_kick_y) > 0.2){
            x_distance = r_kick_x - 0.38;
            cankick = false;
        }
    }
    else if(r_kick_x > 0.43) {
        if(abs(r_kick_y) <= 0.09){
            x_distance = r_kick_x - 0.43;
            cankick = false;
        }
        else if(abs(r_kick_y) > 0.09 && abs(r_kick_y) <= 0.2){
            x_distance = r_kick_x - (-0.455 * abs(r_kick_y) + 0.471);
            cankick = false;
        }
        else if(abs(r_kick_y) > 0.2){
            x_distance = r_kick_x - 0.38;
            cankick = false;
        }
    }

    return y_distance;
}


