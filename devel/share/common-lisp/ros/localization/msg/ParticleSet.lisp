; Auto-generated. Do not edit!


(cl:in-package localization-msg)


;//! \htmlinclude ParticleSet.msg.html

(cl:defclass <ParticleSet> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (particles
    :reader particles
    :initarg :particles
    :type (cl:vector localization-msg:Particle)
   :initform (cl:make-array 0 :element-type 'localization-msg:Particle :initial-element (cl:make-instance 'localization-msg:Particle))))
)

(cl:defclass ParticleSet (<ParticleSet>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ParticleSet>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ParticleSet)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name localization-msg:<ParticleSet> is deprecated: use localization-msg:ParticleSet instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <ParticleSet>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader localization-msg:header-val is deprecated.  Use localization-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'particles-val :lambda-list '(m))
(cl:defmethod particles-val ((m <ParticleSet>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader localization-msg:particles-val is deprecated.  Use localization-msg:particles instead.")
  (particles m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ParticleSet>) ostream)
  "Serializes a message object of type '<ParticleSet>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'particles))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'particles))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ParticleSet>) istream)
  "Deserializes a message object of type '<ParticleSet>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'particles) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'particles)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'localization-msg:Particle))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ParticleSet>)))
  "Returns string type for a message object of type '<ParticleSet>"
  "localization/ParticleSet")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ParticleSet)))
  "Returns string type for a message object of type 'ParticleSet"
  "localization/ParticleSet")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ParticleSet>)))
  "Returns md5sum for a message object of type '<ParticleSet>"
  "4d32c9e0abdc8871f449c29f3a280317")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ParticleSet)))
  "Returns md5sum for a message object of type 'ParticleSet"
  "4d32c9e0abdc8871f449c29f3a280317")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ParticleSet>)))
  "Returns full string definition for message of type '<ParticleSet>"
  (cl:format cl:nil "Header header~%Particle[] particles~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: localization/Particle~%geometry_msgs/Pose2D pose~%float32 weight~%~%================================================================================~%MSG: geometry_msgs/Pose2D~%# Deprecated~%# Please use the full 3D pose.~%~%# In general our recommendation is to use a full 3D representation of everything and for 2D specific applications make the appropriate projections into the plane for their calculations but optimally will preserve the 3D information during processing.~%~%# If we have parallel copies of 2D datatypes every UI and other pipeline will end up needing to have dual interfaces to plot everything. And you will end up with not being able to use 3D tools for 2D use cases even if they're completely valid, as you'd have to reimplement it with different inputs and outputs. It's not particularly hard to plot the 2D pose or compute the yaw error for the Pose message and there are already tools and libraries that can do this for you.~%~%~%# This expresses a position and orientation on a 2D manifold.~%~%float64 x~%float64 y~%float64 theta~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ParticleSet)))
  "Returns full string definition for message of type 'ParticleSet"
  (cl:format cl:nil "Header header~%Particle[] particles~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: localization/Particle~%geometry_msgs/Pose2D pose~%float32 weight~%~%================================================================================~%MSG: geometry_msgs/Pose2D~%# Deprecated~%# Please use the full 3D pose.~%~%# In general our recommendation is to use a full 3D representation of everything and for 2D specific applications make the appropriate projections into the plane for their calculations but optimally will preserve the 3D information during processing.~%~%# If we have parallel copies of 2D datatypes every UI and other pipeline will end up needing to have dual interfaces to plot everything. And you will end up with not being able to use 3D tools for 2D use cases even if they're completely valid, as you'd have to reimplement it with different inputs and outputs. It's not particularly hard to plot the 2D pose or compute the yaw error for the Pose message and there are already tools and libraries that can do this for you.~%~%~%# This expresses a position and orientation on a 2D manifold.~%~%float64 x~%float64 y~%float64 theta~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ParticleSet>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'particles) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ParticleSet>))
  "Converts a ROS message object to a list"
  (cl:list 'ParticleSet
    (cl:cons ':header (header msg))
    (cl:cons ':particles (particles msg))
))
