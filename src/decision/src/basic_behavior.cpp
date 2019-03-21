#include "basic_behavior.h"
#include "Definitions.h"

// Need to change this to send the commands over the UDP program
// No need for the service now, because walk and head are controlled
// by the second NUC. Also, the robot body orientation Imu is received
// by the second NUC, so that information will be received along with
// the odometry data.

#define STEP_LENGTH 0.15
#define STEP_WIDTH 0.08


/*

decision_serial_output_data
received_data[0]: Command Type

0- Nothing
1-Walk with speed mode
3-Walk with target point mode
4-Kick left leg
5-Kick right leg
6-Kick (Use with ball_x, ball_y)


FLOAT[1]: v_x for 1, target_x for 3, ball_x for 6
FLOAT[2]: v_y for 1, target_y for 3, ball_y for 6
FLOAT[3]: v_theta for 1, target_theta for 3
FLOAT[4]:
FLOAT[5]:
FLOAT[6]:


*/

void behavior_initialize_motors::execute()
{
    //cout << "[BEHAVIOR]:initialize motors";
    printf("[BEHAVIOR]:initialize motors");

    currentFrame.decision_serial_output_data.received_data[0] = 0;
    currentFrame.decision_serial_output_data.received_data[1] = 0;
    currentFrame.decision_serial_output_data.received_data[2] = 0;
    currentFrame.decision_serial_output_data.received_data[3] = 0;
    currentFrame.decision_serial_output_data.received_data[4] = 0;
    currentFrame.decision_serial_output_data.received_data[5] = 0;
    currentFrame.decision_serial_output_data.received_data[6] = 0;

}

void behavior_nothing::execute()
{
    //cout << "[BEHAVIOR]:nothing";
    printf("[BEHAVIOR]:nothing");

    currentFrame.decision_serial_output_data.received_data[0] = 0;
    currentFrame.decision_serial_output_data.received_data[1] = 0;
    currentFrame.decision_serial_output_data.received_data[2] = 0;
    currentFrame.decision_serial_output_data.received_data[3] = 0;
    currentFrame.decision_serial_output_data.received_data[4] = 0;
    currentFrame.decision_serial_output_data.received_data[5] = 0;
    currentFrame.decision_serial_output_data.received_data[6] = 0;

    currentFrame.ball_moved_while_moving = false;
    currentFrame.expected_ball_bearing = currentFrame.ballBearing ;
    currentFrame.first_time_track_ball = true;

}

void behavior_end_game::execute()
{
    //cout << "[BEHAVIOR]:end game";
    printf("[BEHAVIOR]:end game");

    currentFrame.decision_serial_output_data.received_data[0] = 0;
    currentFrame.decision_serial_output_data.received_data[1] = 0;
    currentFrame.decision_serial_output_data.received_data[2] = 0;
    currentFrame.decision_serial_output_data.received_data[3] = 0;
    currentFrame.decision_serial_output_data.received_data[4] = 0;
    currentFrame.decision_serial_output_data.received_data[5] = 0;
    currentFrame.decision_serial_output_data.received_data[6] = 0;

}

void behavior_initialize::execute()
{
    //cout << "[BEHAVIOR]:initialize";
    printf("[BEHAVIOR]:initialize");

    currentFrame.decision_serial_output_data.received_data[0] = 0;
    currentFrame.decision_serial_output_data.received_data[1] = 0;
    currentFrame.decision_serial_output_data.received_data[2] = 0;
    currentFrame.decision_serial_output_data.received_data[3] = 0;
    currentFrame.decision_serial_output_data.received_data[4] = 0;
    currentFrame.decision_serial_output_data.received_data[5] = 0;
    currentFrame.decision_serial_output_data.received_data[6] = 0;

    isInitializing = false;
}


void behavior_rotate_around_center::execute()
{
    printf("[BEHAVIOR]:turn around a center:%f for a given angle:%f",currentFrame.target_x,currentFrame.target_theta);

    currentFrame.decision_serial_output_data.received_data[0] = 7;  // change this when the behavior is ready
    currentFrame.decision_serial_output_data.received_data[1] = currentFrame.target_x;
    currentFrame.decision_serial_output_data.received_data[2] = 0;

    if (currentFrame.target_theta >M_PI)
        currentFrame.target_theta += -2*M_PI;
    else if (currentFrame.target_theta < -M_PI)
        currentFrame.target_theta += 2*M_PI;

    currentFrame.decision_serial_output_data.received_data[3] = currentFrame.target_theta;
    currentFrame.decision_serial_output_data.received_data[4] = 0;
    currentFrame.decision_serial_output_data.received_data[5] = 0;
    currentFrame.decision_serial_output_data.received_data[6] = 0;

    currentFrame.robot_moved = true;

}


void behavior_walk::execute()
{
    //cout << "[BEHAVIOR]:walk to a specific target range and bearing";
    printf("[BEHAVIOR]:walk to a specific target range and bearing");

    currentFrame.decision_serial_output_data.received_data[0] = 1;

    currentFrame.decision_serial_output_data.received_data[1] = V_X_MAX/3;
    currentFrame.decision_serial_output_data.received_data[2] = 0;
    currentFrame.decision_serial_output_data.received_data[3] = 0;

    currentFrame.decision_serial_output_data.received_data[4] = 0;
    currentFrame.decision_serial_output_data.received_data[5] = 0;
    currentFrame.decision_serial_output_data.received_data[6] = 0;

    //currentFrame.robot_moved = true;

}

void behavior_stop_walk::execute()
{
    //cout << "[BEHAVIOR]:walk to a specific target range and bearing";
    printf("[BEHAVIOR]:walk to a specific target range and bearing");

    currentFrame.decision_serial_output_data.received_data[0] = 1;

    currentFrame.decision_serial_output_data.received_data[1] = 0;
    currentFrame.decision_serial_output_data.received_data[2] = 0;
    currentFrame.decision_serial_output_data.received_data[3] = 0;
    currentFrame.decision_serial_output_data.received_data[4] = 0;
    currentFrame.decision_serial_output_data.received_data[5] = 0;
    currentFrame.decision_serial_output_data.received_data[6] = 0;

    //currentFrame.robot_moved = true;

}

void behavior_step_forward::execute()
{
    //cout << "[BEHAVIOR]:take a step forward of distance ";
    printf("[BEHAVIOR]:take a step forward of distance ");

    currentFrame.decision_serial_output_data.received_data[0] = 0;
    currentFrame.decision_serial_output_data.received_data[1] = 0;
    currentFrame.decision_serial_output_data.received_data[2] = 0;
    currentFrame.decision_serial_output_data.received_data[3] = 0;
    currentFrame.decision_serial_output_data.received_data[4] = 0;
    currentFrame.decision_serial_output_data.received_data[5] = 0;
    currentFrame.decision_serial_output_data.received_data[6] = 0;

    currentFrame.robot_moved = true;

}

void behavior_step_backward::execute()
{
    //cout << "[BEHAVIOR]:take a step backward of distance ";
    printf("[BEHAVIOR]:take a step backward of distance ");

    currentFrame.decision_serial_output_data.received_data[0] = 0;
    currentFrame.decision_serial_output_data.received_data[1] = 0;
    currentFrame.decision_serial_output_data.received_data[2] = 0;
    currentFrame.decision_serial_output_data.received_data[3] = 0;
    currentFrame.decision_serial_output_data.received_data[4] = 0;
    currentFrame.decision_serial_output_data.received_data[5] = 0;
    currentFrame.decision_serial_output_data.received_data[6] = 0;

    currentFrame.robot_moved = true;

}

void behavior_step_left::execute()
{
    //cout << "[BEHAVIOR]:take a step left of distance ";
    printf("[BEHAVIOR]:take a step left of distance ");

    currentFrame.decision_serial_output_data.received_data[0] = 0;
    currentFrame.decision_serial_output_data.received_data[1] = 0;
    currentFrame.decision_serial_output_data.received_data[2] = 0;
    currentFrame.decision_serial_output_data.received_data[3] = 0;
    currentFrame.decision_serial_output_data.received_data[4] = 0;
    currentFrame.decision_serial_output_data.received_data[5] = 0;
    currentFrame.decision_serial_output_data.received_data[6] = 0;

    currentFrame.robot_moved = true;

}

void behavior_step_right::execute()
{
    //cout << "[BEHAVIOR]:take a step right of distance ";
    printf("[BEHAVIOR]:take a step right of distance ");

    currentFrame.decision_serial_output_data.received_data[0] = 0;
    currentFrame.decision_serial_output_data.received_data[1] = 0;
    currentFrame.decision_serial_output_data.received_data[2] = 0;
    currentFrame.decision_serial_output_data.received_data[3] = 0;
    currentFrame.decision_serial_output_data.received_data[4] = 0;
    currentFrame.decision_serial_output_data.received_data[5] = 0;
    currentFrame.decision_serial_output_data.received_data[6] = 0;

    currentFrame.robot_moved = true;

}

void behavior_kick_ball_soft::execute()
{
    printf("[BEHAVIOR]: Rolling ball kick ");

    currentFrame.decision_serial_output_data.received_data[0] = 14;
    currentFrame.decision_serial_output_data.received_data[1] = 0;
    currentFrame.decision_serial_output_data.received_data[2] = 0;
    currentFrame.decision_serial_output_data.received_data[3] = 0;
    currentFrame.decision_serial_output_data.received_data[4] = 0;
    currentFrame.decision_serial_output_data.received_data[5] = 0;
    currentFrame.decision_serial_output_data.received_data[6] = 0;

    currentFrame.robot_moved = true;
}


void behavior_kick_ball_mid::execute()
{
    //cout << "[BEHAVIOR]:kick the ball with medium strength ";
    printf("[BEHAVIOR]:kick the ball with medium strength  ");

    if (currentFrame.ballBearing>0)
    {
        currentFrame.decision_serial_output_data.received_data[0] = 4;
        ROS_INFO("Kick ball with left leg");
    }
    else
    {
        currentFrame.decision_serial_output_data.received_data[0] = 5;
        ROS_INFO("Kick ball with right leg");
    }

    currentFrame.decision_serial_output_data.received_data[1] = 0;
    currentFrame.decision_serial_output_data.received_data[2] = 0;
    currentFrame.decision_serial_output_data.received_data[3] = 0;
    currentFrame.decision_serial_output_data.received_data[4] = 0;
    currentFrame.decision_serial_output_data.received_data[5] = 0;
    currentFrame.decision_serial_output_data.received_data[6] = 0;

    currentFrame.robot_moved = true;
}

void behavior_kick_ball_strong::execute()
{
    printf("[BEHAVIOR]:Walking kick ball at range:%f and bearing:%f",currentFrame.ballRange,currentFrame.ballBearing/M_PI*180);


    currentFrame.decision_serial_output_data.received_data[0] = 6;
    currentFrame.decision_serial_output_data.received_data[1] = currentFrame.ballRange * cos(currentFrame.ballBearing)*1.05 + 0.02;
    currentFrame.decision_serial_output_data.received_data[2] = currentFrame.ballRange * sin(currentFrame.ballBearing);
    currentFrame.decision_serial_output_data.received_data[3] = 0.0; //currentFrame.ballBearing;
    currentFrame.decision_serial_output_data.received_data[4] = 0;
    currentFrame.decision_serial_output_data.received_data[5] = 0;
    currentFrame.decision_serial_output_data.received_data[6] = 0;

    //    // Debug
    //    currentFrame.decision_serial_output_data.received_data[1] = 0.2;
    //    currentFrame.decision_serial_output_data.received_data[2] = -0.1;
    //    currentFrame.decision_serial_output_data.received_data[3] = 0;

    ROS_INFO("[BEHAVIOR]:kick the ball with high strength at (%f,%f,%f)",currentFrame.decision_serial_output_data.received_data[1],
            currentFrame.decision_serial_output_data.received_data[2],currentFrame.decision_serial_output_data.received_data[3]);



    currentFrame.robot_moved = true;
}


void behavior_high_kick::execute()
{
    printf("[BEHAVIOR]: Perform high kick");


    currentFrame.decision_serial_output_data.received_data[0] = 8;
    currentFrame.decision_serial_output_data.received_data[1] = 0;
    currentFrame.decision_serial_output_data.received_data[2] = 0;
    currentFrame.decision_serial_output_data.received_data[3] = 0.0; //currentFrame.ballBearing;
    currentFrame.decision_serial_output_data.received_data[4] = 0;
    currentFrame.decision_serial_output_data.received_data[5] = 0;
    currentFrame.decision_serial_output_data.received_data[6] = 0;

    currentFrame.robot_moved = true;
}

void behavior_approach_ball::execute()
{
    //cout << "[BEHAVIOR]:kick the ball with medium strength ";
    printf("[BEHAVIOR]:approach ball with target x:%f,y:%f,theta:%f",currentFrame.target_x,currentFrame.target_y,currentFrame.target_theta);



    currentFrame.decision_serial_output_data.received_data[0] = 3;
    currentFrame.decision_serial_output_data.received_data[1] = currentFrame.target_x;
    //    if (currentFrame.target_x<0.1)
    //        currentFrame.decision_serial_output_data.received_data[1] = 0;
    currentFrame.decision_serial_output_data.received_data[2] = currentFrame.target_y;
    //    if (fabs(currentFrame.target_y)<0.1)
    //        currentFrame.decision_serial_output_data.received_data[2] = 0;


    currentFrame.decision_serial_output_data.received_data[3] = currentFrame.target_theta;
    // Change the target theta according to the point we want to reach
    //currentFrame.decision_serial_output_data.received_data[3] = atan2(currentFrame.target_y,currentFrame.target_x);

    currentFrame.decision_serial_output_data.received_data[4] = 0;
    currentFrame.decision_serial_output_data.received_data[5] = 0;
    currentFrame.decision_serial_output_data.received_data[6] = 0;

    //    // Debug
    //    currentFrame.decision_serial_output_data.received_data[1] = 0.8;
    //    currentFrame.decision_serial_output_data.received_data[2] = 0.1;

    currentFrame.robot_moved = true;

}

void behavior_approach_pose::execute()
{

    int data_counter = 0;
    /// Walk with goal pose

    currentFrame.decision_serial_output_data.received_data[0] = 3;

    currentFrame.decision_serial_output_data.received_data[1] = currentFrame.robot_to_target_range
            * cos(currentFrame.robot_to_target_theta);
    currentFrame.decision_serial_output_data.received_data[2] =  currentFrame.robot_to_target_range
            * sin(currentFrame.robot_to_target_theta);
    currentFrame.decision_serial_output_data.received_data[3] = currentFrame.robot_to_target_theta;

    ROS_INFO("Sent target pose (%f,%f,%f)",currentFrame.decision_serial_output_data.received_data[1],currentFrame.decision_serial_output_data.received_data[2],
            currentFrame.decision_serial_output_data.received_data[3]);

    data_counter = 3;

    for(int i=data_counter+1;i<currentFrame.decision_serial_output_data.received_data.size();i++)
    {
        currentFrame.decision_serial_output_data.received_data[i] = 0;
    }

}

/*
 * Additions 2018 for opt_go_to_ball
 */

void behavior_rotate_before_walk::execute()
{
    //cout << "[BEHAVIOR]:kick the ball with medium strength ";
    printf("[BEHAVIOR]:behavior_rotate_before_walk ");

    currentFrame.decision_serial_output_data.received_data[0] = 3;
    currentFrame.decision_serial_output_data.received_data[1] = 0;
    currentFrame.decision_serial_output_data.received_data[2] = 0;
    currentFrame.decision_serial_output_data.received_data[3] =  currentFrame.ballBearing;

    currentFrame.decision_serial_output_data.received_data[4] = 0;
    currentFrame.decision_serial_output_data.received_data[5] = 0;
    currentFrame.decision_serial_output_data.received_data[6] = 0;


    currentFrame.robot_moved = true;

}

void behavior_walk_to_pose::execute()
{
    printf("[BEHAVIOR]: behavior_walk_to_pose ");

    currentFrame.decision_serial_output_data.received_data[0] = 1;
    currentFrame.decision_serial_output_data.received_data[1] = 0.25;  //v_x
    currentFrame.decision_serial_output_data.received_data[2] = 0;  //v_y

    currentFrame.decision_serial_output_data.received_data[3] = 0.4 * currentFrame.robot_to_target_theta;  //v_theta
    currentFrame.decision_serial_output_data.received_data[4] = 0;
    currentFrame.decision_serial_output_data.received_data[5] = 0;
    currentFrame.decision_serial_output_data.received_data[6] = 0;

    currentFrame.robot_moved = true;

    ROS_INFO("Walk to pose with rot speed:%f",currentFrame.decision_serial_output_data.received_data[3]);

}

void behavior_walk_to_ball::execute()
{
    printf("[BEHAVIOR]: behavior_walk_to_ball ");

    currentFrame.decision_serial_output_data.received_data[0] = 1;
    currentFrame.decision_serial_output_data.received_data[1] = 0.2;  //v_x
    currentFrame.decision_serial_output_data.received_data[2] = 0;  //v_y
    currentFrame.decision_serial_output_data.received_data[3] = 0.4 * currentFrame.ballBearing;  //v_theta
    currentFrame.decision_serial_output_data.received_data[4] = 0;
    currentFrame.decision_serial_output_data.received_data[5] = 0;
    currentFrame.decision_serial_output_data.received_data[6] = 0;

    currentFrame.robot_moved = true;

}


void behavior_rotate_after_walk::execute()
{
    //cout << "[BEHAVIOR]:kick the ball with medium strength ";
    printf("[BEHAVIOR]:kick the ball with medium strength  ");

    currentFrame.decision_serial_output_data.received_data[0] = 3;
    currentFrame.decision_serial_output_data.received_data[1] = currentFrame.ballRange * cos(currentFrame.ballBearing);
    currentFrame.decision_serial_output_data.received_data[2] = currentFrame.ballRange * sin(currentFrame.ballBearing);
    currentFrame.decision_serial_output_data.received_data[3] = 0;
    currentFrame.decision_serial_output_data.received_data[4] = 0;
    currentFrame.decision_serial_output_data.received_data[5] = 0;
    currentFrame.decision_serial_output_data.received_data[6] = 0;

    //    currentFrame.decision_serial_output_data.received_data[1] = 0.8;
    //    currentFrame.decision_serial_output_data.received_data[2] = 0.1;

    currentFrame.robot_moved = true;

}

void behavior_connected_rotate::execute()
{
    printf("[BEHAVIOR]:behavior_connected_rotate  ");

    currentFrame.decision_serial_output_data.received_data[0] = 11;
    currentFrame.decision_serial_output_data.received_data[1] = currentFrame.ballRange * cos(currentFrame.ballBearing);
    currentFrame.decision_serial_output_data.received_data[2] = currentFrame.ballRange * sin(currentFrame.ballBearing);
    if ((currentFrame.ballLoc_world_x > 2.5) && (fabs(currentFrame.ballLoc_world_y) < 1.0))
        currentFrame.decision_serial_output_data.received_data[3] = -currentFrame.robotLoc_theta;
    else
        currentFrame.decision_serial_output_data.received_data[3] = -currentFrame.robotLoc_theta + currentFrame.robot_goal_bearing_loc;
    currentFrame.decision_serial_output_data.received_data[4] = 0;
    currentFrame.decision_serial_output_data.received_data[5] = 0;
    currentFrame.decision_serial_output_data.received_data[6] = 0;

    //    currentFrame.decision_serial_output_data.received_data[1] = 0.8;
    //    currentFrame.decision_serial_output_data.received_data[2] = 0.1;

    currentFrame.robot_moved = true;

}

void behavior_kick_ball_right::execute()
{
    printf("[BEHAVIOR]:behavior_kick_ball_right  ");

    currentFrame.decision_serial_output_data.received_data[0] = 12;
    currentFrame.decision_serial_output_data.received_data[1] = 0;
    currentFrame.decision_serial_output_data.received_data[2] = 0;
    currentFrame.decision_serial_output_data.received_data[3] = 0;
    currentFrame.decision_serial_output_data.received_data[4] = 0;
    currentFrame.decision_serial_output_data.received_data[5] = 0;
    currentFrame.decision_serial_output_data.received_data[6] = 0;

    //    currentFrame.decision_serial_output_data.received_data[1] = 0.8;
    //    currentFrame.decision_serial_output_data.received_data[2] = 0.1;

    currentFrame.robot_moved = true;

}

void behavior_kick_ball_left::execute()
{
    printf("[BEHAVIOR]:behavior_kick_ball_left  ");

    currentFrame.decision_serial_output_data.received_data[0] = 13;
    currentFrame.decision_serial_output_data.received_data[1] = 0;
    currentFrame.decision_serial_output_data.received_data[2] = 0;
    currentFrame.decision_serial_output_data.received_data[3] = 0;
    currentFrame.decision_serial_output_data.received_data[4] = 0;
    currentFrame.decision_serial_output_data.received_data[5] = 0;
    currentFrame.decision_serial_output_data.received_data[6] = 0;

    //    currentFrame.decision_serial_output_data.received_data[1] = 0.8;
    //    currentFrame.decision_serial_output_data.received_data[2] = 0.1;

    currentFrame.robot_moved = true;

}
