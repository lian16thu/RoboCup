; Auto-generated. Do not edit!


(cl:in-package localization-msg)


;//! \htmlinclude GoalpostsDetected.msg.html

(cl:defclass <GoalpostsDetected> (roslisp-msg-protocol:ros-message)
  ((pose
    :reader pose
    :initarg :pose
    :type geometry_msgs-msg:Pose2D
    :initform (cl:make-instance 'geometry_msgs-msg:Pose2D))
   (type
    :reader type
    :initarg :type
    :type cl:fixnum
    :initform 0)
   (confidence
    :reader confidence
    :initarg :confidence
    :type cl:float
    :initform 0.0))
)

(cl:defclass GoalpostsDetected (<GoalpostsDetected>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GoalpostsDetected>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GoalpostsDetected)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name localization-msg:<GoalpostsDetected> is deprecated: use localization-msg:GoalpostsDetected instead.")))

(cl:ensure-generic-function 'pose-val :lambda-list '(m))
(cl:defmethod pose-val ((m <GoalpostsDetected>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader localization-msg:pose-val is deprecated.  Use localization-msg:pose instead.")
  (pose m))

(cl:ensure-generic-function 'type-val :lambda-list '(m))
(cl:defmethod type-val ((m <GoalpostsDetected>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader localization-msg:type-val is deprecated.  Use localization-msg:type instead.")
  (type m))

(cl:ensure-generic-function 'confidence-val :lambda-list '(m))
(cl:defmethod confidence-val ((m <GoalpostsDetected>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader localization-msg:confidence-val is deprecated.  Use localization-msg:confidence instead.")
  (confidence m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GoalpostsDetected>) ostream)
  "Serializes a message object of type '<GoalpostsDetected>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'pose) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'type)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'confidence))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GoalpostsDetected>) istream)
  "Deserializes a message object of type '<GoalpostsDetected>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'pose) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'type)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'confidence) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GoalpostsDetected>)))
  "Returns string type for a message object of type '<GoalpostsDetected>"
  "localization/GoalpostsDetected")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GoalpostsDetected)))
  "Returns string type for a message object of type 'GoalpostsDetected"
  "localization/GoalpostsDetected")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GoalpostsDetected>)))
  "Returns md5sum for a message object of type '<GoalpostsDetected>"
  "45117ab335e578bb32c6e4a315f61f4d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GoalpostsDetected)))
  "Returns md5sum for a message object of type 'GoalpostsDetected"
  "45117ab335e578bb32c6e4a315f61f4d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GoalpostsDetected>)))
  "Returns full string definition for message of type '<GoalpostsDetected>"
  (cl:format cl:nil "geometry_msgs/Pose2D pose        # Pose~%uint8 type                       # Type (see field_model::WorldObject::Type)~%float32 confidence               # confidence 0..1~%~%================================================================================~%MSG: geometry_msgs/Pose2D~%# Deprecated~%# Please use the full 3D pose.~%~%# In general our recommendation is to use a full 3D representation of everything and for 2D specific applications make the appropriate projections into the plane for their calculations but optimally will preserve the 3D information during processing.~%~%# If we have parallel copies of 2D datatypes every UI and other pipeline will end up needing to have dual interfaces to plot everything. And you will end up with not being able to use 3D tools for 2D use cases even if they're completely valid, as you'd have to reimplement it with different inputs and outputs. It's not particularly hard to plot the 2D pose or compute the yaw error for the Pose message and there are already tools and libraries that can do this for you.~%~%~%# This expresses a position and orientation on a 2D manifold.~%~%float64 x~%float64 y~%float64 theta~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GoalpostsDetected)))
  "Returns full string definition for message of type 'GoalpostsDetected"
  (cl:format cl:nil "geometry_msgs/Pose2D pose        # Pose~%uint8 type                       # Type (see field_model::WorldObject::Type)~%float32 confidence               # confidence 0..1~%~%================================================================================~%MSG: geometry_msgs/Pose2D~%# Deprecated~%# Please use the full 3D pose.~%~%# In general our recommendation is to use a full 3D representation of everything and for 2D specific applications make the appropriate projections into the plane for their calculations but optimally will preserve the 3D information during processing.~%~%# If we have parallel copies of 2D datatypes every UI and other pipeline will end up needing to have dual interfaces to plot everything. And you will end up with not being able to use 3D tools for 2D use cases even if they're completely valid, as you'd have to reimplement it with different inputs and outputs. It's not particularly hard to plot the 2D pose or compute the yaw error for the Pose message and there are already tools and libraries that can do this for you.~%~%~%# This expresses a position and orientation on a 2D manifold.~%~%float64 x~%float64 y~%float64 theta~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GoalpostsDetected>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'pose))
     1
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GoalpostsDetected>))
  "Converts a ROS message object to a list"
  (cl:list 'GoalpostsDetected
    (cl:cons ':pose (pose msg))
    (cl:cons ':type (type msg))
    (cl:cons ':confidence (confidence msg))
))
