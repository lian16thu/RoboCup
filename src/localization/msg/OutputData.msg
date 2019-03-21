std_msgs/Header header 				# for time stamp
geometry_msgs/Pose2D robotPose        		# Pose of the robot according to the particle filter localization
geometry_msgs/Pose2D robotHeadPose              # Pose of the robot head according to the particle filter localization
float32 robotPoseConfidence               	# confidence 0..1 for the robot_pose
bool bBallWasSeen                               # boolean variable for ball detection
geometry_msgs/Point ballCenterInImage		# coordinates of ball center in the image
float32 ballDistance				# ball distance from robot as seen in the image
float32 ballAngle				# ball angle from center as seen in the image, [-90,90]
geometry_msgs/Point ballCenterOnField		# coordinates of ball center on field relative to robot localization
bool bOpponentWasSeen                           # boolean variable for opponent detection
geometry_msgs/Point opponentLeftEndInImage	# coordinates of opponent left end point in the image
geometry_msgs/Point opponentRightEndInImage	# coordinates of opponent right end point in the image
float32 opponentDistance			# opponent distance from robot as seen in the image
float32 opponentAngle				# opponent angle from center as seen in the image, [-90,90]
geometry_msgs/Point opponentCenterOnField	# coordinates of opponent center on field relative to robot localization
float32 opponentRadiusOnField			# estimated radius of opponent on field
bool bObstacleWasSeen                           # boolean variable for obstacle detection
int32 iObstacleNumber                           # index of obstacle
geometry_msgs/Point[5] obstacleLeftEndInImage	# coordinates of obstacle left end point in the image
geometry_msgs/Point[5] obstacleRightEndInImage	# coordinates of obstacle right end point in the image
float32[5] obstacleDistance                     # obstacle distance from robot as seen in the image
float32[5] obstacleAngle			# obstacle angle from center as seen in the image, [-90,90]
geometry_msgs/Point[5] obstacleCenterOnField	# coordinates of obstacle center on field relative to robot localization
float32[5] obstacleRadiusOnField		# estimated radius of obstacle on field

