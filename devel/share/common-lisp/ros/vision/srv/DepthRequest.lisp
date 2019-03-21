; Auto-generated. Do not edit!


(cl:in-package vision-srv)


;//! \htmlinclude DepthRequest-request.msg.html

(cl:defclass <DepthRequest-request> (roslisp-msg-protocol:ros-message)
  ((u
    :reader u
    :initarg :u
    :type cl:integer
    :initform 0)
   (v
    :reader v
    :initarg :v
    :type cl:integer
    :initform 0))
)

(cl:defclass DepthRequest-request (<DepthRequest-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DepthRequest-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DepthRequest-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name vision-srv:<DepthRequest-request> is deprecated: use vision-srv:DepthRequest-request instead.")))

(cl:ensure-generic-function 'u-val :lambda-list '(m))
(cl:defmethod u-val ((m <DepthRequest-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vision-srv:u-val is deprecated.  Use vision-srv:u instead.")
  (u m))

(cl:ensure-generic-function 'v-val :lambda-list '(m))
(cl:defmethod v-val ((m <DepthRequest-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vision-srv:v-val is deprecated.  Use vision-srv:v instead.")
  (v m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DepthRequest-request>) ostream)
  "Serializes a message object of type '<DepthRequest-request>"
  (cl:let* ((signed (cl:slot-value msg 'u)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'v)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DepthRequest-request>) istream)
  "Deserializes a message object of type '<DepthRequest-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'u) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'v) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DepthRequest-request>)))
  "Returns string type for a service object of type '<DepthRequest-request>"
  "vision/DepthRequestRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DepthRequest-request)))
  "Returns string type for a service object of type 'DepthRequest-request"
  "vision/DepthRequestRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DepthRequest-request>)))
  "Returns md5sum for a message object of type '<DepthRequest-request>"
  "1036fa6d5d2b46f7aa7aaecd35620560")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DepthRequest-request)))
  "Returns md5sum for a message object of type 'DepthRequest-request"
  "1036fa6d5d2b46f7aa7aaecd35620560")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DepthRequest-request>)))
  "Returns full string definition for message of type '<DepthRequest-request>"
  (cl:format cl:nil "~%~%int32 u~%int32 v~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DepthRequest-request)))
  "Returns full string definition for message of type 'DepthRequest-request"
  (cl:format cl:nil "~%~%int32 u~%int32 v~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DepthRequest-request>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DepthRequest-request>))
  "Converts a ROS message object to a list"
  (cl:list 'DepthRequest-request
    (cl:cons ':u (u msg))
    (cl:cons ':v (v msg))
))
;//! \htmlinclude DepthRequest-response.msg.html

(cl:defclass <DepthRequest-response> (roslisp-msg-protocol:ros-message)
  ((depth
    :reader depth
    :initarg :depth
    :type cl:float
    :initform 0.0))
)

(cl:defclass DepthRequest-response (<DepthRequest-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DepthRequest-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DepthRequest-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name vision-srv:<DepthRequest-response> is deprecated: use vision-srv:DepthRequest-response instead.")))

(cl:ensure-generic-function 'depth-val :lambda-list '(m))
(cl:defmethod depth-val ((m <DepthRequest-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vision-srv:depth-val is deprecated.  Use vision-srv:depth instead.")
  (depth m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DepthRequest-response>) ostream)
  "Serializes a message object of type '<DepthRequest-response>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'depth))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DepthRequest-response>) istream)
  "Deserializes a message object of type '<DepthRequest-response>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'depth) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DepthRequest-response>)))
  "Returns string type for a service object of type '<DepthRequest-response>"
  "vision/DepthRequestResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DepthRequest-response)))
  "Returns string type for a service object of type 'DepthRequest-response"
  "vision/DepthRequestResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DepthRequest-response>)))
  "Returns md5sum for a message object of type '<DepthRequest-response>"
  "1036fa6d5d2b46f7aa7aaecd35620560")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DepthRequest-response)))
  "Returns md5sum for a message object of type 'DepthRequest-response"
  "1036fa6d5d2b46f7aa7aaecd35620560")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DepthRequest-response>)))
  "Returns full string definition for message of type '<DepthRequest-response>"
  (cl:format cl:nil "float64 depth~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DepthRequest-response)))
  "Returns full string definition for message of type 'DepthRequest-response"
  (cl:format cl:nil "float64 depth~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DepthRequest-response>))
  (cl:+ 0
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DepthRequest-response>))
  "Converts a ROS message object to a list"
  (cl:list 'DepthRequest-response
    (cl:cons ':depth (depth msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'DepthRequest)))
  'DepthRequest-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'DepthRequest)))
  'DepthRequest-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DepthRequest)))
  "Returns string type for a service object of type '<DepthRequest>"
  "vision/DepthRequest")