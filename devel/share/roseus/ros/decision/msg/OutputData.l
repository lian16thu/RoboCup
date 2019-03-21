;; Auto-generated. Do not edit!


(when (boundp 'decision::OutputData)
  (if (not (find-package "DECISION"))
    (make-package "DECISION"))
  (shadow 'OutputData (find-package "DECISION")))
(unless (find-package "DECISION::OUTPUTDATA")
  (make-package "DECISION::OUTPUTDATA"))

(in-package "ROS")
;;//! \htmlinclude OutputData.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass decision::OutputData
  :super ros::object
  :slots (_header _robotPose _robotPoseConfidence _bBallWasSeen _ballCenterInImage _ballDistance _ballAngle _ballCenterOnField _bObstacleWasSeen _iObstacleNumber _obstacleLeftEndInImage _obstacleRightEndInImage _obstacleDistance _obstacleAngle _obstacleCenterOnField _obstacleRadiusOnField _bKeeperWasSeen _keeperLeftStartOnField _keeperRightEndOnField ))

(defmethod decision::OutputData
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:robotPose __robotPose) (instance geometry_msgs::Pose2D :init))
    ((:robotPoseConfidence __robotPoseConfidence) 0.0)
    ((:bBallWasSeen __bBallWasSeen) nil)
    ((:ballCenterInImage __ballCenterInImage) (instance geometry_msgs::Point :init))
    ((:ballDistance __ballDistance) 0.0)
    ((:ballAngle __ballAngle) 0.0)
    ((:ballCenterOnField __ballCenterOnField) (instance geometry_msgs::Point :init))
    ((:bObstacleWasSeen __bObstacleWasSeen) nil)
    ((:iObstacleNumber __iObstacleNumber) 0)
    ((:obstacleLeftEndInImage __obstacleLeftEndInImage) (let (r) (dotimes (i 0) (push (instance geometry_msgs::Point :init) r)) r))
    ((:obstacleRightEndInImage __obstacleRightEndInImage) (let (r) (dotimes (i 0) (push (instance geometry_msgs::Point :init) r)) r))
    ((:obstacleDistance __obstacleDistance) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:obstacleAngle __obstacleAngle) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:obstacleCenterOnField __obstacleCenterOnField) (let (r) (dotimes (i 0) (push (instance geometry_msgs::Point :init) r)) r))
    ((:obstacleRadiusOnField __obstacleRadiusOnField) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:bKeeperWasSeen __bKeeperWasSeen) nil)
    ((:keeperLeftStartOnField __keeperLeftStartOnField) (instance geometry_msgs::Point :init))
    ((:keeperRightEndOnField __keeperRightEndOnField) (instance geometry_msgs::Point :init))
    )
   (send-super :init)
   (setq _header __header)
   (setq _robotPose __robotPose)
   (setq _robotPoseConfidence (float __robotPoseConfidence))
   (setq _bBallWasSeen __bBallWasSeen)
   (setq _ballCenterInImage __ballCenterInImage)
   (setq _ballDistance (float __ballDistance))
   (setq _ballAngle (float __ballAngle))
   (setq _ballCenterOnField __ballCenterOnField)
   (setq _bObstacleWasSeen __bObstacleWasSeen)
   (setq _iObstacleNumber (round __iObstacleNumber))
   (setq _obstacleLeftEndInImage __obstacleLeftEndInImage)
   (setq _obstacleRightEndInImage __obstacleRightEndInImage)
   (setq _obstacleDistance __obstacleDistance)
   (setq _obstacleAngle __obstacleAngle)
   (setq _obstacleCenterOnField __obstacleCenterOnField)
   (setq _obstacleRadiusOnField __obstacleRadiusOnField)
   (setq _bKeeperWasSeen __bKeeperWasSeen)
   (setq _keeperLeftStartOnField __keeperLeftStartOnField)
   (setq _keeperRightEndOnField __keeperRightEndOnField)
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
  (:bKeeperWasSeen
   (&optional __bKeeperWasSeen)
   (if __bKeeperWasSeen (setq _bKeeperWasSeen __bKeeperWasSeen)) _bKeeperWasSeen)
  (:keeperLeftStartOnField
   (&rest __keeperLeftStartOnField)
   (if (keywordp (car __keeperLeftStartOnField))
       (send* _keeperLeftStartOnField __keeperLeftStartOnField)
     (progn
       (if __keeperLeftStartOnField (setq _keeperLeftStartOnField (car __keeperLeftStartOnField)))
       _keeperLeftStartOnField)))
  (:keeperRightEndOnField
   (&rest __keeperRightEndOnField)
   (if (keywordp (car __keeperRightEndOnField))
       (send* _keeperRightEndOnField __keeperRightEndOnField)
     (progn
       (if __keeperRightEndOnField (setq _keeperRightEndOnField (car __keeperRightEndOnField)))
       _keeperRightEndOnField)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; geometry_msgs/Pose2D _robotPose
    (send _robotPose :serialization-length)
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
    ;; bool _bObstacleWasSeen
    1
    ;; int32 _iObstacleNumber
    4
    ;; geometry_msgs/Point[] _obstacleLeftEndInImage
    (apply #'+ (send-all _obstacleLeftEndInImage :serialization-length)) 4
    ;; geometry_msgs/Point[] _obstacleRightEndInImage
    (apply #'+ (send-all _obstacleRightEndInImage :serialization-length)) 4
    ;; float32[] _obstacleDistance
    (* 4    (length _obstacleDistance)) 4
    ;; float32[] _obstacleAngle
    (* 4    (length _obstacleAngle)) 4
    ;; geometry_msgs/Point[] _obstacleCenterOnField
    (apply #'+ (send-all _obstacleCenterOnField :serialization-length)) 4
    ;; float32[] _obstacleRadiusOnField
    (* 4    (length _obstacleRadiusOnField)) 4
    ;; bool _bKeeperWasSeen
    1
    ;; geometry_msgs/Point _keeperLeftStartOnField
    (send _keeperLeftStartOnField :serialization-length)
    ;; geometry_msgs/Point _keeperRightEndOnField
    (send _keeperRightEndOnField :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; geometry_msgs/Pose2D _robotPose
       (send _robotPose :serialize s)
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
     ;; bool _bObstacleWasSeen
       (if _bObstacleWasSeen (write-byte -1 s) (write-byte 0 s))
     ;; int32 _iObstacleNumber
       (write-long _iObstacleNumber s)
     ;; geometry_msgs/Point[] _obstacleLeftEndInImage
     (write-long (length _obstacleLeftEndInImage) s)
     (dolist (elem _obstacleLeftEndInImage)
       (send elem :serialize s)
       )
     ;; geometry_msgs/Point[] _obstacleRightEndInImage
     (write-long (length _obstacleRightEndInImage) s)
     (dolist (elem _obstacleRightEndInImage)
       (send elem :serialize s)
       )
     ;; float32[] _obstacleDistance
     (write-long (length _obstacleDistance) s)
     (dotimes (i (length _obstacleDistance))
       (sys::poke (elt _obstacleDistance i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[] _obstacleAngle
     (write-long (length _obstacleAngle) s)
     (dotimes (i (length _obstacleAngle))
       (sys::poke (elt _obstacleAngle i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; geometry_msgs/Point[] _obstacleCenterOnField
     (write-long (length _obstacleCenterOnField) s)
     (dolist (elem _obstacleCenterOnField)
       (send elem :serialize s)
       )
     ;; float32[] _obstacleRadiusOnField
     (write-long (length _obstacleRadiusOnField) s)
     (dotimes (i (length _obstacleRadiusOnField))
       (sys::poke (elt _obstacleRadiusOnField i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; bool _bKeeperWasSeen
       (if _bKeeperWasSeen (write-byte -1 s) (write-byte 0 s))
     ;; geometry_msgs/Point _keeperLeftStartOnField
       (send _keeperLeftStartOnField :serialize s)
     ;; geometry_msgs/Point _keeperRightEndOnField
       (send _keeperRightEndOnField :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; geometry_msgs/Pose2D _robotPose
     (send _robotPose :deserialize buf ptr-) (incf ptr- (send _robotPose :serialization-length))
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
   ;; bool _bObstacleWasSeen
     (setq _bObstacleWasSeen (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; int32 _iObstacleNumber
     (setq _iObstacleNumber (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; geometry_msgs/Point[] _obstacleLeftEndInImage
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _obstacleLeftEndInImage (let (r) (dotimes (i n) (push (instance geometry_msgs::Point :init) r)) r))
     (dolist (elem- _obstacleLeftEndInImage)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; geometry_msgs/Point[] _obstacleRightEndInImage
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _obstacleRightEndInImage (let (r) (dotimes (i n) (push (instance geometry_msgs::Point :init) r)) r))
     (dolist (elem- _obstacleRightEndInImage)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; float32[] _obstacleDistance
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _obstacleDistance (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _obstacleDistance i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32[] _obstacleAngle
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _obstacleAngle (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _obstacleAngle i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; geometry_msgs/Point[] _obstacleCenterOnField
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _obstacleCenterOnField (let (r) (dotimes (i n) (push (instance geometry_msgs::Point :init) r)) r))
     (dolist (elem- _obstacleCenterOnField)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; float32[] _obstacleRadiusOnField
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _obstacleRadiusOnField (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _obstacleRadiusOnField i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; bool _bKeeperWasSeen
     (setq _bKeeperWasSeen (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; geometry_msgs/Point _keeperLeftStartOnField
     (send _keeperLeftStartOnField :deserialize buf ptr-) (incf ptr- (send _keeperLeftStartOnField :serialization-length))
   ;; geometry_msgs/Point _keeperRightEndOnField
     (send _keeperRightEndOnField :deserialize buf ptr-) (incf ptr- (send _keeperRightEndOnField :serialization-length))
   ;;
   self)
  )

(setf (get decision::OutputData :md5sum-) "bd064faf37c8b9975f133c15e75ac7aa")
(setf (get decision::OutputData :datatype-) "decision/OutputData")
(setf (get decision::OutputData :definition-)
      "#	data published by the localization node.

std_msgs/Header header 								# for time stamp
geometry_msgs/Pose2D robotPose        				# Pose of the robot according to the particle filter localization	
float32 robotPoseConfidence               			# confidence 0..1 for the robot_pose
bool bBallWasSeen                               	# boolean variable for ball detection
geometry_msgs/Point ballCenterInImage				# coordinates of ball center in the image
float32 ballDistance								# ball distance from robot as seen in the image
float32 ballAngle									# ball angle from center as seen in the image, [-90,90]
geometry_msgs/Point ballCenterOnField				# coordinates of ball center on field relative to robot localization	
bool bObstacleWasSeen                           	# boolean variable for obstacle detection
int32 iObstacleNumber                           	# index of obstacle, if 2 found this is 2
geometry_msgs/Point[] obstacleLeftEndInImage		# coordinates of obstacle left end point in the image
geometry_msgs/Point[] obstacleRightEndInImage		# coordinates of obstacle right end point in the image
float32[] obstacleDistance							# obstacle distance from robot as seen in the image
float32[] obstacleAngle								# obstacle angle from center as seen in the image, [-90,90]
geometry_msgs/Point[] obstacleCenterOnField			# coordinates of obstacle center on field relative to robot localization	
float32[] obstacleRadiusOnField						# estimated radius of obstacle on field
bool bKeeperWasSeen                             	# boolean variable for goalkeeper detection
geometry_msgs/Point keeperLeftStartOnField      	# coordinates of goalkeeper left leg start point on field
geometry_msgs/Point keeperRightEndOnField       	# coordinates of goalkeeper right leg end point on field

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



(provide :decision/OutputData "bd064faf37c8b9975f133c15e75ac7aa")


