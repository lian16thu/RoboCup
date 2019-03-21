; Auto-generated. Do not edit!


(cl:in-package decision-srv)


;//! \htmlinclude walk-request.msg.html

(cl:defclass <walk-request> (roslisp-msg-protocol:ros-message)
  ((type
    :reader type
    :initarg :type
    :type cl:integer
    :initform 0)
   (speed_x
    :reader speed_x
    :initarg :speed_x
    :type cl:float
    :initform 0.0)
   (speed_y
    :reader speed_y
    :initarg :speed_y
    :type cl:float
    :initform 0.0)
   (rotation_speed
    :reader rotation_speed
    :initarg :rotation_speed
    :type cl:float
    :initform 0.0)
   (step
    :reader step
    :initarg :step
    :type cl:integer
    :initform 0))
)

(cl:defclass walk-request (<walk-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <walk-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'walk-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name decision-srv:<walk-request> is deprecated: use decision-srv:walk-request instead.")))

(cl:ensure-generic-function 'type-val :lambda-list '(m))
(cl:defmethod type-val ((m <walk-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader decision-srv:type-val is deprecated.  Use decision-srv:type instead.")
  (type m))

(cl:ensure-generic-function 'speed_x-val :lambda-list '(m))
(cl:defmethod speed_x-val ((m <walk-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader decision-srv:speed_x-val is deprecated.  Use decision-srv:speed_x instead.")
  (speed_x m))

(cl:ensure-generic-function 'speed_y-val :lambda-list '(m))
(cl:defmethod speed_y-val ((m <walk-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader decision-srv:speed_y-val is deprecated.  Use decision-srv:speed_y instead.")
  (speed_y m))

(cl:ensure-generic-function 'rotation_speed-val :lambda-list '(m))
(cl:defmethod rotation_speed-val ((m <walk-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader decision-srv:rotation_speed-val is deprecated.  Use decision-srv:rotation_speed instead.")
  (rotation_speed m))

(cl:ensure-generic-function 'step-val :lambda-list '(m))
(cl:defmethod step-val ((m <walk-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader decision-srv:step-val is deprecated.  Use decision-srv:step instead.")
  (step m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <walk-request>) ostream)
  "Serializes a message object of type '<walk-request>"
  (cl:let* ((signed (cl:slot-value msg 'type)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'speed_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'speed_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'rotation_speed))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let* ((signed (cl:slot-value msg 'step)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <walk-request>) istream)
  "Deserializes a message object of type '<walk-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'type) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'speed_x) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'speed_y) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'rotation_speed) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'step) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<walk-request>)))
  "Returns string type for a service object of type '<walk-request>"
  "decision/walkRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'walk-request)))
  "Returns string type for a service object of type 'walk-request"
  "decision/walkRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<walk-request>)))
  "Returns md5sum for a message object of type '<walk-request>"
  "abed49acefdf43a65d0e6ffd2bf04f9b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'walk-request)))
  "Returns md5sum for a message object of type 'walk-request"
  "abed49acefdf43a65d0e6ffd2bf04f9b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<walk-request>)))
  "Returns full string definition for message of type '<walk-request>"
  (cl:format cl:nil "int32 type~%float64 speed_x~%float64 speed_y~%float64 rotation_speed~%int32 step~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'walk-request)))
  "Returns full string definition for message of type 'walk-request"
  (cl:format cl:nil "int32 type~%float64 speed_x~%float64 speed_y~%float64 rotation_speed~%int32 step~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <walk-request>))
  (cl:+ 0
     4
     8
     8
     8
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <walk-request>))
  "Converts a ROS message object to a list"
  (cl:list 'walk-request
    (cl:cons ':type (type msg))
    (cl:cons ':speed_x (speed_x msg))
    (cl:cons ':speed_y (speed_y msg))
    (cl:cons ':rotation_speed (rotation_speed msg))
    (cl:cons ':step (step msg))
))
;//! \htmlinclude walk-response.msg.html

(cl:defclass <walk-response> (roslisp-msg-protocol:ros-message)
  ((results
    :reader results
    :initarg :results
    :type cl:integer
    :initform 0))
)

(cl:defclass walk-response (<walk-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <walk-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'walk-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name decision-srv:<walk-response> is deprecated: use decision-srv:walk-response instead.")))

(cl:ensure-generic-function 'results-val :lambda-list '(m))
(cl:defmethod results-val ((m <walk-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader decision-srv:results-val is deprecated.  Use decision-srv:results instead.")
  (results m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <walk-response>) ostream)
  "Serializes a message object of type '<walk-response>"
  (cl:let* ((signed (cl:slot-value msg 'results)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <walk-response>) istream)
  "Deserializes a message object of type '<walk-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'results) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<walk-response>)))
  "Returns string type for a service object of type '<walk-response>"
  "decision/walkResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'walk-response)))
  "Returns string type for a service object of type 'walk-response"
  "decision/walkResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<walk-response>)))
  "Returns md5sum for a message object of type '<walk-response>"
  "abed49acefdf43a65d0e6ffd2bf04f9b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'walk-response)))
  "Returns md5sum for a message object of type 'walk-response"
  "abed49acefdf43a65d0e6ffd2bf04f9b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<walk-response>)))
  "Returns full string definition for message of type '<walk-response>"
  (cl:format cl:nil "int32 results~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'walk-response)))
  "Returns full string definition for message of type 'walk-response"
  (cl:format cl:nil "int32 results~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <walk-response>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <walk-response>))
  "Converts a ROS message object to a list"
  (cl:list 'walk-response
    (cl:cons ':results (results msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'walk)))
  'walk-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'walk)))
  'walk-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'walk)))
  "Returns string type for a service object of type '<walk>"
  "decision/walk")