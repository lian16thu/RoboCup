;; Auto-generated. Do not edit!


(when (boundp 'localization::MeanPoseConfStamped)
  (if (not (find-package "LOCALIZATION"))
    (make-package "LOCALIZATION"))
  (shadow 'MeanPoseConfStamped (find-package "LOCALIZATION")))
(unless (find-package "LOCALIZATION::MEANPOSECONFSTAMPED")
  (make-package "LOCALIZATION::MEANPOSECONFSTAMPED"))

(in-package "ROS")
;;//! \htmlinclude MeanPoseConfStamped.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass localization::MeanPoseConfStamped
  :super ros::object
  :slots (_header _robotPose _robotPoseConfidence ))

(defmethod localization::MeanPoseConfStamped
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:robotPose __robotPose) (instance geometry_msgs::Pose2D :init))
    ((:robotPoseConfidence __robotPoseConfidence) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _robotPose __robotPose)
   (setq _robotPoseConfidence (float __robotPoseConfidence))
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
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; geometry_msgs/Pose2D _robotPose
    (send _robotPose :serialization-length)
    ;; float32 _robotPoseConfidence
    4
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
   ;;
   self)
  )

(setf (get localization::MeanPoseConfStamped :md5sum-) "87d282eb9ec7ff5b36abcb25698c079d")
(setf (get localization::MeanPoseConfStamped :datatype-) "localization/MeanPoseConfStamped")
(setf (get localization::MeanPoseConfStamped :definition-)
      "std_msgs/Header header 				# for time stamp
geometry_msgs/Pose2D robotPose        		# Pose of the robot according to the particle filter localization
float32 robotPoseConfidence

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

")



(provide :localization/MeanPoseConfStamped "87d282eb9ec7ff5b36abcb25698c079d")


