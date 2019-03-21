; Auto-generated. Do not edit!


(cl:in-package head_motion-msg)


;//! \htmlinclude head_servo_angel.msg.html

(cl:defclass <head_servo_angel> (roslisp-msg-protocol:ros-message)
  ((pitch
    :reader pitch
    :initarg :pitch
    :type cl:float
    :initform 0.0)
   (yaw
    :reader yaw
    :initarg :yaw
    :type cl:float
    :initform 0.0))
)

(cl:defclass head_servo_angel (<head_servo_angel>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <head_servo_angel>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'head_servo_angel)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name head_motion-msg:<head_servo_angel> is deprecated: use head_motion-msg:head_servo_angel instead.")))

(cl:ensure-generic-function 'pitch-val :lambda-list '(m))
(cl:defmethod pitch-val ((m <head_servo_angel>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader head_motion-msg:pitch-val is deprecated.  Use head_motion-msg:pitch instead.")
  (pitch m))

(cl:ensure-generic-function 'yaw-val :lambda-list '(m))
(cl:defmethod yaw-val ((m <head_servo_angel>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader head_motion-msg:yaw-val is deprecated.  Use head_motion-msg:yaw instead.")
  (yaw m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <head_servo_angel>) ostream)
  "Serializes a message object of type '<head_servo_angel>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'pitch))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'yaw))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <head_servo_angel>) istream)
  "Deserializes a message object of type '<head_servo_angel>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'pitch) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'yaw) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<head_servo_angel>)))
  "Returns string type for a message object of type '<head_servo_angel>"
  "head_motion/head_servo_angel")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'head_servo_angel)))
  "Returns string type for a message object of type 'head_servo_angel"
  "head_motion/head_servo_angel")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<head_servo_angel>)))
  "Returns md5sum for a message object of type '<head_servo_angel>"
  "64da7800e4533f600c4d4b06d8aaa339")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'head_servo_angel)))
  "Returns md5sum for a message object of type 'head_servo_angel"
  "64da7800e4533f600c4d4b06d8aaa339")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<head_servo_angel>)))
  "Returns full string definition for message of type '<head_servo_angel>"
  (cl:format cl:nil "float64 pitch~%float64 yaw~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'head_servo_angel)))
  "Returns full string definition for message of type 'head_servo_angel"
  (cl:format cl:nil "float64 pitch~%float64 yaw~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <head_servo_angel>))
  (cl:+ 0
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <head_servo_angel>))
  "Converts a ROS message object to a list"
  (cl:list 'head_servo_angel
    (cl:cons ':pitch (pitch msg))
    (cl:cons ':yaw (yaw msg))
))
