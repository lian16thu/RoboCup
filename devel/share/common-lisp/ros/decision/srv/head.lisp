; Auto-generated. Do not edit!


(cl:in-package decision-srv)


;//! \htmlinclude head-request.msg.html

(cl:defclass <head-request> (roslisp-msg-protocol:ros-message)
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

(cl:defclass head-request (<head-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <head-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'head-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name decision-srv:<head-request> is deprecated: use decision-srv:head-request instead.")))

(cl:ensure-generic-function 'pitch-val :lambda-list '(m))
(cl:defmethod pitch-val ((m <head-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader decision-srv:pitch-val is deprecated.  Use decision-srv:pitch instead.")
  (pitch m))

(cl:ensure-generic-function 'yaw-val :lambda-list '(m))
(cl:defmethod yaw-val ((m <head-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader decision-srv:yaw-val is deprecated.  Use decision-srv:yaw instead.")
  (yaw m))

(cl:ensure-generic-function 'PID-val :lambda-list '(m))
(cl:defmethod PID-val ((m <head-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader decision-srv:PID-val is deprecated.  Use decision-srv:PID instead.")
  (PID m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <head-request>) ostream)
  "Serializes a message object of type '<head-request>"
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <head-request>) istream)
  "Deserializes a message object of type '<head-request>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<head-request>)))
  "Returns string type for a service object of type '<head-request>"
  "decision/headRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'head-request)))
  "Returns string type for a service object of type 'head-request"
  "decision/headRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<head-request>)))
  "Returns md5sum for a message object of type '<head-request>"
  "e4aff7be97fd6298f4a9b30300ac7316")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'head-request)))
  "Returns md5sum for a message object of type 'head-request"
  "e4aff7be97fd6298f4a9b30300ac7316")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<head-request>)))
  "Returns full string definition for message of type '<head-request>"
  (cl:format cl:nil "float64 pitch~%float64 yaw~%int32 PID~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'head-request)))
  "Returns full string definition for message of type 'head-request"
  (cl:format cl:nil "float64 pitch~%float64 yaw~%int32 PID~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <head-request>))
  (cl:+ 0
     8
     8
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <head-request>))
  "Converts a ROS message object to a list"
  (cl:list 'head-request
    (cl:cons ':pitch (pitch msg))
    (cl:cons ':yaw (yaw msg))
    (cl:cons ':PID (PID msg))
))
;//! \htmlinclude head-response.msg.html

(cl:defclass <head-response> (roslisp-msg-protocol:ros-message)
  ((result
    :reader result
    :initarg :result
    :type cl:integer
    :initform 0))
)

(cl:defclass head-response (<head-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <head-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'head-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name decision-srv:<head-response> is deprecated: use decision-srv:head-response instead.")))

(cl:ensure-generic-function 'result-val :lambda-list '(m))
(cl:defmethod result-val ((m <head-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader decision-srv:result-val is deprecated.  Use decision-srv:result instead.")
  (result m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <head-response>) ostream)
  "Serializes a message object of type '<head-response>"
  (cl:let* ((signed (cl:slot-value msg 'result)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <head-response>) istream)
  "Deserializes a message object of type '<head-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'result) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<head-response>)))
  "Returns string type for a service object of type '<head-response>"
  "decision/headResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'head-response)))
  "Returns string type for a service object of type 'head-response"
  "decision/headResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<head-response>)))
  "Returns md5sum for a message object of type '<head-response>"
  "e4aff7be97fd6298f4a9b30300ac7316")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'head-response)))
  "Returns md5sum for a message object of type 'head-response"
  "e4aff7be97fd6298f4a9b30300ac7316")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<head-response>)))
  "Returns full string definition for message of type '<head-response>"
  (cl:format cl:nil "int32 result~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'head-response)))
  "Returns full string definition for message of type 'head-response"
  (cl:format cl:nil "int32 result~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <head-response>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <head-response>))
  "Converts a ROS message object to a list"
  (cl:list 'head-response
    (cl:cons ':result (result msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'head)))
  'head-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'head)))
  'head-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'head)))
  "Returns string type for a service object of type '<head>"
  "decision/head")