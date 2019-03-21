; Auto-generated. Do not edit!


(cl:in-package head_motion-srv)


;//! \htmlinclude head_control-request.msg.html

(cl:defclass <head_control-request> (roslisp-msg-protocol:ros-message)
  ((pitch
    :reader pitch
    :initarg :pitch
    :type cl:float
    :initform 0.0)
   (yaw
    :reader yaw
    :initarg :yaw
    :type cl:float
    :initform 0.0)
   (PID
    :reader PID
    :initarg :PID
    :type cl:integer
    :initform 0))
)

(cl:defclass head_control-request (<head_control-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <head_control-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'head_control-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name head_motion-srv:<head_control-request> is deprecated: use head_motion-srv:head_control-request instead.")))

(cl:ensure-generic-function 'pitch-val :lambda-list '(m))
(cl:defmethod pitch-val ((m <head_control-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader head_motion-srv:pitch-val is deprecated.  Use head_motion-srv:pitch instead.")
  (pitch m))

(cl:ensure-generic-function 'yaw-val :lambda-list '(m))
(cl:defmethod yaw-val ((m <head_control-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader head_motion-srv:yaw-val is deprecated.  Use head_motion-srv:yaw instead.")
  (yaw m))

(cl:ensure-generic-function 'PID-val :lambda-list '(m))
(cl:defmethod PID-val ((m <head_control-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader head_motion-srv:PID-val is deprecated.  Use head_motion-srv:PID instead.")
  (PID m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <head_control-request>) ostream)
  "Serializes a message object of type '<head_control-request>"
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
  (cl:let* ((signed (cl:slot-value msg 'PID)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <head_control-request>) istream)
  "Deserializes a message object of type '<head_control-request>"
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
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'PID) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<head_control-request>)))
  "Returns string type for a service object of type '<head_control-request>"
  "head_motion/head_controlRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'head_control-request)))
  "Returns string type for a service object of type 'head_control-request"
  "head_motion/head_controlRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<head_control-request>)))
  "Returns md5sum for a message object of type '<head_control-request>"
  "e4aff7be97fd6298f4a9b30300ac7316")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'head_control-request)))
  "Returns md5sum for a message object of type 'head_control-request"
  "e4aff7be97fd6298f4a9b30300ac7316")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<head_control-request>)))
  "Returns full string definition for message of type '<head_control-request>"
  (cl:format cl:nil "float64 pitch~%float64 yaw~%int32 PID~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'head_control-request)))
  "Returns full string definition for message of type 'head_control-request"
  (cl:format cl:nil "float64 pitch~%float64 yaw~%int32 PID~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <head_control-request>))
  (cl:+ 0
     8
     8
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <head_control-request>))
  "Converts a ROS message object to a list"
  (cl:list 'head_control-request
    (cl:cons ':pitch (pitch msg))
    (cl:cons ':yaw (yaw msg))
    (cl:cons ':PID (PID msg))
))
;//! \htmlinclude head_control-response.msg.html

(cl:defclass <head_control-response> (roslisp-msg-protocol:ros-message)
  ((result
    :reader result
    :initarg :result
    :type cl:integer
    :initform 0))
)

(cl:defclass head_control-response (<head_control-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <head_control-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'head_control-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name head_motion-srv:<head_control-response> is deprecated: use head_motion-srv:head_control-response instead.")))

(cl:ensure-generic-function 'result-val :lambda-list '(m))
(cl:defmethod result-val ((m <head_control-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader head_motion-srv:result-val is deprecated.  Use head_motion-srv:result instead.")
  (result m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <head_control-response>) ostream)
  "Serializes a message object of type '<head_control-response>"
  (cl:let* ((signed (cl:slot-value msg 'result)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <head_control-response>) istream)
  "Deserializes a message object of type '<head_control-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'result) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<head_control-response>)))
  "Returns string type for a service object of type '<head_control-response>"
  "head_motion/head_controlResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'head_control-response)))
  "Returns string type for a service object of type 'head_control-response"
  "head_motion/head_controlResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<head_control-response>)))
  "Returns md5sum for a message object of type '<head_control-response>"
  "e4aff7be97fd6298f4a9b30300ac7316")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'head_control-response)))
  "Returns md5sum for a message object of type 'head_control-response"
  "e4aff7be97fd6298f4a9b30300ac7316")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<head_control-response>)))
  "Returns full string definition for message of type '<head_control-response>"
  (cl:format cl:nil "int32 result~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'head_control-response)))
  "Returns full string definition for message of type 'head_control-response"
  (cl:format cl:nil "int32 result~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <head_control-response>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <head_control-response>))
  "Converts a ROS message object to a list"
  (cl:list 'head_control-response
    (cl:cons ':result (result msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'head_control)))
  'head_control-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'head_control)))
  'head_control-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'head_control)))
  "Returns string type for a service object of type '<head_control>"
  "head_motion/head_control")