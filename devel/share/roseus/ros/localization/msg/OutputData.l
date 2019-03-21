;; Auto-generated. Do not edit!


(when (boundp 'localization::OutputData)
  (if (not (find-package "LOCALIZATION"))
    (make-package "LOCALIZATION"))
  (shadow 'OutputData (find-package "LOCALIZATION")))
(unless (find-package "LOCALIZATION::OUTPUTDATA")
  (make-package "LOCALIZATION::OUTPUTDATA"))

(in-package "ROS")
;;//! \htmlinclude OutputData.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass localization::OutputData
  :super ros::object
  :slots (_header _robotPose _robotHeadPose _robotPoseConfidence _bBallWasSeen _ballCenterInImage _ballDistance _ballAngle _ballCenterOnField _bOpponentWasSeen _opponentLeftEndInImage _opponentRightEndInImage _opponentDistance _opponentAngle _opponentCenterOnField _opponentRadiusOnField _bObstacleWasSeen _iObstacleNumber _obstacleLeftEndInImage _obstacleRightEndInImage _obstacleDistance _obstacleAngle _obstacleCenterOnField _obstacleRadiusOnField ))

(defmethod localization::OutputData
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:robotPose __robotPose) (instance geometry_msgs::Pose2D :init))
    ((:robotHeadPose __robotHeadPose) (instance geometry_msgs::Pose2D :init))
    ((:robotPoseConfidence __robotPoseConfidence) 0.0)
    ((:bBallWasSeen __bBallWasSeen) nil)
    ((:ballCenterInImage __ballCenterInImage) (instance geometry_msgs::Point :init))
    ((:ballDistance __ballDistance) 0.0)
    ((:ballAngle __ballAngle) 0.0)
    ((:ballCenterOnField __ballCenterOnField) (instance geometry_msgs::Point :init))
    ((:bOpponentWasSeen __bOpponentWasSeen) nil)
    ((:opponentLeftEndInImage __opponentLeftEndInImage) (instance geometry_msgs::Point :init))
    ((:opponentRightEndInImage __opponentRightEndInImage) (instance geometry_msgs::Point :init))
    ((:opponentDistance __opponentDistance) 0.0)
    ((:opponentAngle __opponentAngle) 0.0)
    ((:opponentCenterOnField __opponentCenterOnField) (instance geometry_msgs::Point :init))
    ((:opponentRadiusOnField __opponentRadiusOnField) 0.0)
    ((:bObstacleWasSeen __bObstacleWasSeen) nil)
    ((:iObstacleNumber __iObstacleNumber) 0)
    ((:obstacleLeftEndInImage __obstacleLeftEndInImage) (let (r) (dotimes (i 5) (push (instance geometry_msgs::Point :init) r)) r))
    ((:obstacleRightEndInImage __obstacleRightEndInImage) (let (r) (dotimes (i 5) (push (instance geometry_msgs::Point :init) r)) r))
    ((:obstacleDistance __obstacleDistance) (make-array 5 :initial-element 0.0 :element-type :float))
    ((:obstacleAngle __obstacleAngle) (make-array 5 :initial-element 0.0 :element-type :float))
    ((:obstacleCenterOnField __obstacleCenterOnField) (let (r) (dotimes (i 5) (push (instance geometry_msgs::Point :init) r)) r))
    ((:obstacleRadiusOnField __obstacleRadiusOnField) (make-array 5 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _header __header)
   (setq _robotPose __robotPose)
   (setq _robotHeadPose __robotHeadPose)
   (setq _robotPoseConfidence (float __robotPoseConfidence))
   (setq _bBallWasSeen __bBallWasSeen)
   (setq _ballCenterInImage __ballCenterInImage)
   (setq _ballDistance (float __ballDistance))
   (setq _ballAngle (float __ballAngle))
   (setq _ballCenterOnField __ballCenterOnField)
   (setq _bOpponentWasSeen __bOpponentWasSeen)
   (setq _opponentLeftEndInImage __opponentLeftEndInImage)
   (setq _opponentRightEndInImage __opponentRightEndInImage)
   (setq _opponentDistance (float __opponentDistance))
   (setq _opponentAngle (float __opponentAngle))
   (setq _opponentCenterOnField __opponentCenterOnField)
   (setq _opponentRadiusOnField (float __opponentRadiusOnField))
   (setq _bObstacleWasSeen __bObstacleWasSeen)
   (setq _iObstacleNumber (round __iObstacleNumber))
   (setq _obstacleLeftEndInImage __obstacleLeftEndInImage)
   (setq _obstacleRightEndInImage __obstacleRightEndInImage)
   (setq _obstacleDistance __obstacleDistance)
   (setq _obstacleAngle __obstacleAngle)
   (setq _obstacleCenterOnField __obstacleCenterOnField)
   (setq _obstacleRadiusOnField __obstacleRadiusOnField)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:robotPose
   (&rest __robotPose)
   (if (keywordp (car __robotPose))
       (send* _robotPose __robotPose)
     (progn
       (if __robotPose (setq _robotPose (car __robotPose)))
       _robotPose)))
  (:robotHeadPose
   (&rest __robotHeadPose)
   (if (keywordp (car __robotHeadPose))
       (send* _robotHeadPose __robotHeadPose)
     (progn
       (if __robotHeadPose (setq _robotHeadPose (car __robotHeadPose)))
       _robotHeadPose)))
  (:robotPoseConfidence
   (&optional __robotPoseConfidence)
   (if __robotPoseConfidence (setq _robotPoseConfidence __robotPoseConfidence)) _robotPoseConfidence)
  (:bBallWasSeen
   (&optional __bBallWasSeen)
   (if __bBallWasSeen (setq _bBallWasSeen __bBallWasSeen)) _bBallWasSeen)
  (:ballCenterInImage
   (&rest __ballCenterInImage)
   (if (keywordp (car __ballCenterInImage))
       (send* _ballCenterInImage __ballCenterInImage)
     (progn
       (if __ballCenterInImage (setq _ballCenterInImage (car __ballCenterInImage)))
       _ballCenterInImage)))
  (:ballDistance
   (&optional __ballDistance)
   (if __ballDistance (setq _ballDistance __ballDistance)) _ballDistance)
  (:ballAngle
   (&optional __ballAngle)
   (if __ballAngle (setq _ballAngle __ballAngle)) _ballAngle)
  (:ballCenterOnField
   (&rest __ballCenterOnField)
   (if (keywordp (car __ballCenterOnField))
       (send* _ballCenterOnField __ballCenterOnField)
     (progn
       (if __ballCenterOnField (setq _ballCenterOnField (car __ballCenterOnField)))
       _ballCenterOnField)))
  (:bOpponentWasSeen
   (&optional __bOpponentWasSeen)
   (if __bOpponentWasSeen (setq _bOpponentWasSeen __bOpponentWasSeen)) _bOpponentWasSeen)
  (:opponentLeftEndInImage
   (&rest __opponentLeftEndInImage)
   (if (keywordp (car __opponentLeftEndInImage))
       (send* _opponentLeftEndInImage __opponentLeftEndInImage)
     (progn
       (if __opponentLeftEndInImage (setq _opponentLeftEndInImage (car __opponentLeftEndInImage)))
       _opponentLeftEndInImage)))
  (:opponentRightEndInImage
   (&rest __opponentRightEndInImage)
   (if (keywordp (car __opponentRightEndInImage))
       (send* _opponentRightEndInImage __opponentRightEndInImage)
     (progn
       (if __opponentRightEndInImage (setq _opponentRightEndInImage (car __opponentRightEndInImage)))
       _opponentRightEndInImage)))
  (:opponentDistance
   (&optional __opponentDistance)
   (if __opponentDistance (setq _opponentDistance __opponentDistance)) _opponentDistance)
  (:opponentAngle
   (&optional __opponentAngle)
   (if __opponentAngle (setq _opponentAngle __opponentAngle)) _opponentAngle)
  (:opponentCenterOnField
   (&rest __opponentCenterOnField)
   (if (keywordp (car __opponentCenterOnField))
       (send* _opponentCenterOnField __opponentCenterOnField)
     (progn
       (if __opponentCenterOnField (setq _opponentCenterOnField (car __opponentCenterOnField)))
       _opponentCenterOnField)))
  (:opponentRadiusOnField
   (&optional __opponentRadiusOnField)
   (if __opponentRadiusOnField (setq _opponentRadiusOnField __opponentRadiusOnField)) _opponentRadiusOnField)
  (:bObstacleWasSeen
   (&optional __bObstacleWasSeen)
   (if __bObstacleWasSeen (setq _bObstacleWasSeen __bObstacleWasSeen)) _bObstacleWasSeen)
  (:iObstacleNumber
   (&optional __iObstacleNumber)
   (if __iObstacleNumber (setq _iObstacleNumber __iObstacleNumber)) _iObstacleNumber)
  (:obstacleLeftEndInImage
   (&rest __obstacleLeftEndInImage)
   (if (keywordp (car __obstacleLeftEndInImage))
       (send* _obstacleLeftEndInImage __obstacleLeftEndInImage)
     (progn
       (if __obstacleLeftEndInImage (setq _obstacleLeftEndInImage (car __obstacleLeftEndInImage)))
       _obstacleLeftEndInImage)))
  (:obstacleRightEndInImage
   (&rest __obstacleRightEndInImage)
   (if (keywordp (car __obstacleRightEndInImage))
       (send* _obstacleRightEndInImage __obstacleRightEndInImage)
     (progn
       (if __obstacleRightEndInImage (setq _obstacleRightEndInImage (car __obstacleRightEndInImage)))
       _obstacleRightEndInImage)))
  (:obstacleDistance
   (&optional __obstacleDistance)
   (if __obstacleDistance (setq _obstacleDistance __obstacleDistance)) _obstacleDistance)
  (:obstacleAngle
   (&optional __obstacleAngle)
   (if __obstacleAngle (setq _obstacleAngle __obstacleAngle)) _obstacleAngle)
  (:obstacleCenterOnField
   (&rest __obstacleCenterOnField)
   (if (keywordp (car __obstacleCenterOnField))
       (send* _obstacleCenterOnField __obstacleCenterOnField)
     (progn
       (if __obstacleCenterOnField (setq _obstacleCenterOnField (car __obstacleCenterOnField)))
       _obstacleCenterOnField)))
  (:obstacleRadiusOnField
   (&optional __obstacleRadiusOnField)
   (if __obstacleRadiusOnField (setq _obstacleRadiusOnField __obstacleRadiusOnField)) _obstacleRadiusOnField)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; geometry_msgs/Pose2D _robotPose
    (send _robotPose :serialization-length)
    ;; geometry_msgs/Pose2D _robotHeadPose
    (send _robotHeadPose :serialization-length)
    ;; float32 _robotPoseConfidence
    4
    ;; bool _bBallWasSeen
    1
    ;; geometry_msgs/Point _ballCenterInImage
    (send _ballCenterInImage :serialization-length)
    ;; float32 _ballDistance
    4
    ;; float32 _ballAngle
    4
    ;; geometry_msgs/Point _ballCenterOnField
    (send _ballCenterOnField :serialization-length)
    ;; bool _bOpponentWasSeen
    1
    ;; geometry_msgs/Point _opponentLeftEndInImage
    (send _opponentLeftEndInImage :serialization-length)
    ;; geometry_msgs/Point _opponentRightEndInImage
    (send _opponentRightEndInImage :serialization-length)
    ;; float32 _opponentDistance
    4
    ;; float32 _opponentAngle
    4
    ;; geometry_msgs/Point _opponentCenterOnField
    (send _opponentCenterOnField :serialization-length)
    ;; float32 _opponentRadiusOnField
    4
    ;; bool _bObstacleWasSeen
    1
    ;; int32 _iObstacleNumber
    4
    ;; geometry_msgs/Point[5] _obstacleLeftEndInImage
    (apply #'+ (send-all _obstacleLeftEndInImage :serialization-length))
    ;; geometry_msgs/Point[5] _obstacleRightEndInImage
    (apply #'+ (send-all _obstacleRightEndInImage :serialization-length))
    ;; float32[5] _obstacleDistance
    (* 4    5)
    ;; float32[5] _obstacleAngle
    (* 4    5)
    ;; geometry_msgs/Point[5] _obstacleCenterOnField
    (apply #'+ (send-all _obstacleCenterOnField :serialization-length))
    ;; float32[5] _obstacleRadiusOnField
    (* 4    5)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; geometry_msgs/Pose2D _robotPose
       (send _robotPose :serialize s)
     ;; geometry_msgs/Pose2D _robotHeadPose
       (send _robotHeadPose :serialize s)
     ;; float32 _robotPoseConfidence
       (sys::poke _robotPoseConfidence (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; bool _bBallWasSeen
       (if _bBallWasSeen (write-byte -1 s) (write-byte 0 s))
     ;; geometry_msgs/Point _ballCenterInImage
       (send _ballCenterInImage :serialize s)
     ;; float32 _ballDistance
       (sys::poke _ballDistance (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _ballAngle
       (sys::poke _ballAngle (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; geometry_msgs/Point _ballCenterOnField
       (send _ballCenterOnField :serialize s)
     ;; bool _bOpponentWasSeen
       (if _bOpponentWasSeen (write-byte -1 s) (write-byte 0 s))
     ;; geometry_msgs/Point _opponentLeftEndInImage
       (send _opponentLeftEndInImage :serialize s)
     ;; geometry_msgs/Point _opponentRightEndInImage
       (send _opponentRightEndInImage :serialize s)
     ;; float32 _opponentDistance
       (sys::poke _opponentDistance (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _opponentAngle
       (sys::poke _opponentAngle (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; geometry_msgs/Point _opponentCenterOnField
       (send _opponentCenterOnField :serialize s)
     ;; float32 _opponentRadiusOnField
       (sys::poke _opponentRadiusOnField (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; bool _bObstacleWasSeen
       (if _bObstacleWasSeen (write-byte -1 s) (write-byte 0 s))
     ;; int32 _iObstacleNumber
       (write-long _iObstacleNumber s)
     ;; geometry_msgs/Point[5] _obstacleLeftEndInImage
     (dolist (elem _obstacleLeftEndInImage)
       (send elem :serialize s)
       )
     ;; geometry_msgs/Point[5] _obstacleRightEndInImage
     (dolist (elem _obstacleRightEndInImage)
       (send elem :serialize s)
       )
     ;; float32[5] _obstacleDistance
     (dotimes (i 5)
       (sys::poke (elt _obstacleDistance i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[5] _obstacleAngle
     (dotimes (i 5)
       (sys::poke (elt _obstacleAngle i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; geometry_msgs/Point[5] _obstacleCenterOnField
     (dolist (elem _obstacleCenterOnField)
       (send elem :serialize s)
       )
     ;; float32[5] _obstacleRadiusOnField
     (dotimes (i 5)
       (sys::poke (elt _obstacleRadiusOnField i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; geometry_msgs/Pose2D _robotPose
     (send _robotPose :deserialize buf ptr-) (incf ptr- (send _robotPose :serialization-length))
   ;; geometry_msgs/Pose2D _robotHeadPose
     (send _robotHeadPose :deserialize buf ptr-) (incf ptr- (send _robotHeadPose :serialization-length))
   ;; float32 _robotPoseConfidence
     (setq _robotPoseConfidence (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; bool _bBallWasSeen
     (setq _bBallWasSeen (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; geometry_msgs/Point _ballCenterInImage
     (send _ballCenterInImage :deserialize buf ptr-) (incf ptr- (send _ballCenterInImage :serialization-length))
   ;; float32 _ballDistance
     (setq _ballDistance (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _ballAngle
     (setq _ballAngle (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; geometry_msgs/Point _ballCenterOnField
     (send _ballCenterOnField :deserialize buf ptr-) (incf ptr- (send _ballCenterOnField :serialization-length))
   ;; bool _bOpponentWasSeen
     (setq _bOpponentWasSeen (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; geometry_msgs/Point _opponentLeftEndInImage
     (send _opponentLeftEndInImage :deserialize buf ptr-) (incf ptr- (send _opponentLeftEndInImage :serialization-length))
   ;; geometry_msgs/Point _opponentRightEndInImage
     (send _opponentRightEndInImage :deserialize buf ptr-) (incf ptr- (send _opponentRightEndInImage :serialization-length))
   ;; float32 _opponentDistance
     (setq _opponentDistance (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _opponentAngle
     (setq _opponentAngle (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; geometry_msgs/Point _opponentCenterOnField
     (send _opponentCenterOnField :deserialize buf ptr-) (incf ptr- (send _opponentCenterOnField :serialization-length))
   ;; float32 _opponentRadiusOnField
     (setq _opponentRadiusOnField (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; bool _bObstacleWasSeen
     (setq _bObstacleWasSeen (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; int32 _iObstacleNumber
     (setq _iObstacleNumber (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; geometry_msgs/Point[5] _obstacleLeftEndInImage
   (dotimes (i 5)
     (send (elt _obstacleLeftEndInImage i) :deserialize buf ptr-) (incf ptr- (send (elt _obstacleLeftEndInImage i) :serialization-length))
     )
   ;; geometry_msgs/Point[5] _obstacleRightEndInImage
   (dotimes (i 5)
     (send (elt _obstacleRightEndInImage i) :deserialize buf ptr-) (incf ptr- (send (elt _obstacleRightEndInImage i) :serialization-length))
     )
   ;; float32[5] _obstacleDistance
   (dotimes (i (length _obstacleDistance))
     (setf (elt _obstacleDistance i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     )
   ;; float32[5] _obstacleAngle
   (dotimes (i (length _obstacleAngle))
     (setf (elt _obstacleAngle i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     )
   ;; geometry_msgs/Point[5] _obstacleCenterOnField
   (dotimes (i 5)
     (send (elt _obstacleCenterOnField i) :deserialize buf ptr-) (incf ptr- (send (elt _obstacleCenterOnField i) :serialization-length))
     )
   ;; float32[5] _obstacleRadiusOnField
   (dotimes (i (length _obstacleRadiusOnField))
     (setf (elt _obstacleRadiusOnField i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     )
   ;;
   self)
  )

(setf (get localization::OutputData :md5sum-) "0d5bb91e3132d7eb05a2ae36456b86d6")
(setf (get localization::OutputData :datatype-) "localization/OutputData")
(setf (get localization::OutputData :definition-)
      "std_msgs/Header header 				# for time stamp
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


================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
# 0: no frame
# 1: global frame
string frame_id

================================================================================
MSG: geometry_msgs/Pose2D
# Deprecated
# Please use the full 3D pose.

# In general our recommendation is to use a full 3D representation of everything and for 2D specific applications make the appropriate projections into the plane for their calculations but optimally will preserve the 3D information during processing.

# If we have parallel copies of 2D datatypes every UI and other pipeline will end up needing to have dual interfaces to plot everything. And you will end up with not being able to use 3D tools for 2D use cases even if they're completely valid, as you'd have to reimplement it with different inputs and outputs. It's not particularly hard to plot the 2D pose or compute the yaw error for the Pose message and there are already tools and libraries that can do this for you.


# This expresses a position and orientation on a 2D manifold.

float64 x
float64 y
float64 theta

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

")



(provide :localization/OutputData "0d5bb91e3132d7eb05a2ae36456b86d6")


