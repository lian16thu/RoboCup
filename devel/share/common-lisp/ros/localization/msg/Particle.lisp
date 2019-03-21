; Auto-generated. Do not edit!


(cl:in-package localization-msg)


;//! \htmlinclude Particle.msg.html

(cl:defclass <Particle> (roslisp-msg-protocol:ros-message)
  ((pose
    :reader pose
    :initarg :pose
    :type geometry_msgs-msg:Pose2D
    :initform (cl:make-instance 'geometry_msgs-msg:Pose2D))
   (weight
    :reader weight
    :initarg :weight
    :type cl:float
    :initform 0.0))
)

(cl:defclass Particle (<Particle>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Particle>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Particle)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name localization-msg:<Particle> is deprecated: use localization-msg:Particle instead.")))

(cl:ensure-generic-function 'pose-val :lambda-list '(m))
(cl:defmethod pose-val ((m <Particle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader localization-msg:pose-val is deprecated.  Use localization-msg:pose instead.")
  (pose m))

(cl:ensure-generic-function 'weight-val :lambda-list '(m))
(cl:defmethod weight-val ((m <Particle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader localization-msg:weight-val is deprecated.  Use localization-msg:weight instead.")
  (weight m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Particle>) ostream)
  "Serializes a message object of type '<Particle>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'pose) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'weight))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Particle>) istream)
  "Deserializes a message object of type '<Particle>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'pose) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'weight) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Particle>)))
  "Returns string type for a message object of type '<Particle>"
  "localization/Particle")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Particle)))
  "Returns string type for a message object of type 'Particle"
  "localization/Particle")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Particle>)))
  "Returns md5sum for a message object of type '<Particle>"
  "9d383f6b0a152c96c1ec3decf6abd3d3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Particle)))
  "Returns md5sum for a message object of type 'Particle"
  "9d383f6b0a152c96c1ec3decf6abd3d3")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Particle>)))
  "Returns full string definition for message of type '<Particle>"
  (cl:format cl:nil "geometry_msgs/Pose2D pose~%float32 weight~%~%================================================================================~%MSG: geometry_msgs/Pose2D~%# Deprecated~%# Please use the full 3D pose.~%~%# In general our recommendation is to use a full 3D representation of everything and for 2D specific applications make the appropriate projections into the plane for their calculations but optimally will preserve the 3D information during processing.~%~%# If we have parallel copies of 2D datatypes every UI and other pipeline will end up needing to have dual interfaces to plot everything. And you will end up with not being able to use 3D tools for 2D use cases even if they're completely valid, as you'd have to reimplement it with different inputs and outputs. It's not particularly hard to plot the 2D pose or compute the yaw error for the Pose message and there are already tools and libraries that can do this for you.~%~%~%# This expresses a position and orientation on a 2D manifold.~%~%float64 x~%float64 y~%float64 theta~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Particle)))
  "Returns full string definition for message of type 'Particle"
  (cl:format cl:nil "geometry_msgs/Pose2D pose~%float32 weight~%~%================================================================================~%MSG: geometry_msgs/Pose2D~%# Deprecated~%# Please use the full 3D pose.~%~%# In general our recommendation is to use a full 3D representation of everything and for 2D specific applications make the appropriate projections into the plane for their calculations but optimally will preserve the 3D information during processing.~%~%# If we have parallel copies of 2D datatypes every UI and other pipeline will end up needing to have dual interfaces to plot everything. And you will end up with not being able to use 3D tools for 2D use cases even if they're completely valid, as you'd have to reimplement it with different inputs and outputs. It's not particularly hard to plot the 2D pose or compute the yaw error for the Pose message and there are already tools and libraries that can do this for you.~%~%~%# This expresses a position and orientation on a 2D manifold.~%~%float64 x~%float64 y~%float64 theta~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Particle>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'pose))
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Particle>))
  "Converts a ROS message object to a list"
  (cl:list 'Particle
    (cl:cons ':pose (pose msg))
    (cl:cons ':weight (weight msg))
))
