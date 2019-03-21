#pragma once
#include "ros/ros.h"
#include "RoboCupGameControlData.h"
#include <netinet/in.h>    // for sockaddr_in
#include <sys/types.h>    // for socket
#include <sys/socket.h>    // for socket
#include <stdio.h>        // for printf
#include <stdlib.h>        // for exit
#include <string.h>        // for bzero

#define THU_HEPHAESTUS_ID 29
#define PLAYER_ID 0

class GameController
{
public:
	int myTeamID;
	int playerOnCourt;
	int m_socket;

	GameController(ros::Publisher* pPub);
	~GameController();
	ros::Publisher *pPublisher;
	bool Execute();
	bool Initialize();
protected:
private:
	bool ParsePacket(RoboCupGameControlData *pControlData);
	bool MyStringIsSame(char * strFir,char * strSec); 
};
