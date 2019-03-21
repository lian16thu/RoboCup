; Auto-generated. Do not edit!


(cl:in-package localization-msg)


;//! \htmlinclude MeanPoseConfStamped.msg.html

(cl:defclass <MeanPoseConfStamped> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (robotPose
    :reader robotPose
    :initarg :robotPose
    :type geometry_msgs-msg:Pose2D
    :initform (cl:make-instance 'geometry_msgs-msg:Pose2D))
   (robotPoseConfidence
    :reader robotPoseConfidence
    :initarg :robotPoseConfidence
    :type cl:float
    :initform 0.0))
)

(cl:defclass MeanPoseConfStamped (<MeanPoseConfStamped>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MeanPoseConfStamped>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MeanPoseConfStamped)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name localization-msg:<MeanPoseConfStamped> is deprecated: use localization-msg:MeanPoseConfStamped instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <MeanPoseConfStamped>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader localization-msg:header-val is deprecated.  Use localization-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'robotPose-val :lambda-list '(m))
(cl:defmethod robotPose-val ((m <MeanPoseConfStamped>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader localization-msg:robotPose-val is deprecated.  Use localization-msg:robotPose instead.")
  (robotPose m))

(cl:ensure-generic-function 'robotPoseConfidence-val :lambda-list '(m))
(cl:defmethod robotPoseConfidence-val ((m <MeanPoseConfStamped>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader localization-msg:robotPoseConfidence-val is deprecated.  Use localization-msg:robotPoseConfidence instead.")
  (robotPoseConfidence m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MeanPoseConfStamped>) ostream)
  "Serializes a message object of type '<MeanPoseConfStamped>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'robotPose) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'robotPoseConfidence))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MeanPoseConfStamped>) istream)
  "Deserializes a message object of type '<MeanPoseConfStamped>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'robotPose) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'robotPoseConfidence) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MeanPoseConfStamped>)))
  "Returns string type for a message object of type '<MeanPoseConfStamped>"
  "localization/MeanPoseConfStamped")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MeanPoseConfStamped)))
  "Returns string type for a message object of type 'MeanPoseConfStamped"
  "localization/MeanPoseConfStamped")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MeanPoseConfStamped>)))
  "Returns md5sum for a message object of type '<MeanPoseConfStamped>"
  "87d282eb9ec7ff5b36abcb25698c079d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MeanPoseConfStamped)))
  "Returns md5sum for a message object of type 'MeanPoseConfStamped"
  "87d282eb9ec7ff5b36abcb25698c079d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MeanPoseConfStamped>)))
  "Returns full string definition for message of type '<MeanPoseConfStamped>"
  (cl:format cl:nil "std_msgs/Header header 				# for time stamp~%geometry_msgs/Pose2D robotPose        		# Pose of the robot according to the particle filter localization~%float32 robotPoseConfidence~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose2D~%# Deprecated~%# Please use the full 3D pose.~%~%# In general our recommendation is to use a full 3D representation of everything and for 2D specific applications make the appropriate projections into the plane for their calculations but optimally will preserve the 3D information during processing.~%~%# If we have parallel copies of 2D datatypes every UI and other pipeline will end up needing to have dual interfaces to plot everything. And you will end up with not being able to use 3D tools for 2D use cases even if they're completely valid, as you'd have to reimplement it with different inputs and outputs. It's not particularly hard to plot the 2D pose or compute the yaw error for the Pose message and there are already tools and libraries that can do this for you.~%~%~%# This expresses a position and orientation on a 2D manifold.~%~%float64 x~%float64 y~%float64 theta~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MeanPoseConfStamped)))
  "Returns full string definition for message of type 'MeanPoseConfStamped"
  (cl:format cl:nil "std_msgs/Header header 				# for time stamp~%geometry_msgs/Pose2D robotPose        		# Pose of the robot according to the particle filter localization~%float32 robotPoseConfidence~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose2D~%# Deprecated~%# Please use the full 3D pose.~%~%# In general our recommendation is to use a full 3D representation of everything and for 2D specific applications make the appropriate projections into the plane for their calculations but optimally will preserve the 3D information during processing.~%~%# If we have parallel copies of 2D datatypes every UI and other pipeline will end up needing to have dual interfaces to plot everything. And you will end up with not being able to use 3D tools for 2D use cases even if they're completely valid, as you'd have to reimplement it with different inputs and outputs. It's not particularly hard to plot the 2D pose or compute the yaw error for the Pose message and there are already tools and libraries that can do this for you.~%~%~%# This expresses a position and orientation on a 2D manifold.~%~%float64 x~%float64 y~%float64 theta~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MeanPoseConfStamped>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'robotPose))
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MeanPoseConfStamped>))
  "Converts a ROS message object to a list"
  (cl:list 'MeanPoseConfStamped
    (cl:cons ':header (header msg))
    (cl:cons ':robotPose (robotPose msg))
    (cl:cons ':robotPoseConfidence (robotPoseConfidence msg))
))
