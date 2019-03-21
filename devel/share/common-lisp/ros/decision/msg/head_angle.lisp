; Auto-generated. Do not edit!


(cl:in-package decision-msg)


;//! \htmlinclude head_angle.msg.html

(cl:defclass <head_angle> (roslisp-msg-protocol:ros-message)
  ((angle_head_pitch
    :reader angle_head_pitch
    :initarg :angle_head_pitch
    :type cl:float
    :initform 0.0)
   (angle_head_yaw
    :reader angle_head_yaw
    :initarg :angle_head_yaw
    :type cl:float
    :initform 0.0))
)

(cl:defclass head_angle (<head_angle>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <head_angle>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'head_angle)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name decision-msg:<head_angle> is deprecated: use decision-msg:head_angle instead.")))

(cl:ensure-generic-function 'angle_head_pitch-val :lambda-list '(m))
(cl:defmethod angle_head_pitch-val ((m <head_angle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader decision-msg:angle_head_pitch-val is deprecated.  Use decision-msg:angle_head_pitch instead.")
  (angle_head_pitch m))

(cl:ensure-generic-function 'angle_head_yaw-val :lambda-list '(m))
(cl:defmethod angle_head_yaw-val ((m <head_angle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader decision-msg:angle_head_yaw-val is deprecated.  Use decision-msg:angle_head_yaw instead.")
  (angle_head_yaw m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <head_angle>) ostream)
  "Serializes a message object of type '<head_angle>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'angle_head_pitch))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'angle_head_yaw))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <head_angle>) istream)
  "Deserializes a message object of type '<head_angle>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'angle_head_pitch) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'angle_head_yaw) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<head_angle>)))
  "Returns string type for a message object of type '<head_angle>"
  "decision/head_angle")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'head_angle)))
  "Returns string type for a message object of type 'head_angle"
  "decision/head_angle")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<head_angle>)))
  "Returns md5sum for a message object of type '<head_angle>"
  "13464c771052649b3693335e0dc785b9")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'head_angle)))
  "Returns md5sum for a message object of type 'head_angle"
  "13464c771052649b3693335e0dc785b9")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<head_angle>)))
  "Returns full string definition for message of type '<head_angle>"
  (cl:format cl:nil "float64 angle_head_pitch	#pitch~%float64 angle_head_yaw  	#yaw~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'head_angle)))
  "Returns full string definition for message of type 'head_angle"
  (cl:format cl:nil "float64 angle_head_pitch	#pitch~%float64 angle_head_yaw  	#yaw~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <head_angle>))
  (cl:+ 0
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <head_angle>))
  "Converts a ROS message object to a list"
  (cl:list 'head_angle
    (cl:cons ':angle_head_pitch (angle_head_pitch msg))
    (cl:cons ':angle_head_yaw (angle_head_yaw msg))
))
