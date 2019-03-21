; Auto-generated. Do not edit!


(cl:in-package vision-msg)


;//! \htmlinclude Ball.msg.html

(cl:defclass <Ball> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (ball_detected
    :reader ball_detected
    :initarg :ball_detected
    :type cl:boolean
    :initform cl:nil)
   (ball_range
    :reader ball_range
    :initarg :ball_range
    :type cl:float
    :initform 0.0)
   (ball_bearing
    :reader ball_bearing
    :initarg :ball_bearing
    :type cl:float
    :initform 0.0)
   (ball_radius
    :reader ball_radius
    :initarg :ball_radius
    :type cl:float
    :initform 0.0)
   (ball_center_x
    :reader ball_center_x
    :initarg :ball_center_x
    :type cl:integer
    :initform 0)
   (ball_center_y
    :reader ball_center_y
    :initarg :ball_center_y
    :type cl:integer
    :initform 0)
   (kick_time
    :reader kick_time
    :initarg :kick_time
    :type cl:float
    :initform 0.0))
)

(cl:defclass Ball (<Ball>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Ball>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Ball)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name vision-msg:<Ball> is deprecated: use vision-msg:Ball instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <Ball>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vision-msg:header-val is deprecated.  Use vision-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'ball_detected-val :lambda-list '(m))
(cl:defmethod ball_detected-val ((m <Ball>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vision-msg:ball_detected-val is deprecated.  Use vision-msg:ball_detected instead.")
  (ball_detected m))

(cl:ensure-generic-function 'ball_range-val :lambda-list '(m))
(cl:defmethod ball_range-val ((m <Ball>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vision-msg:ball_range-val is deprecated.  Use vision-msg:ball_range instead.")
  (ball_range m))

(cl:ensure-generic-function 'ball_bearing-val :lambda-list '(m))
(cl:defmethod ball_bearing-val ((m <Ball>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vision-msg:ball_bearing-val is deprecated.  Use vision-msg:ball_bearing instead.")
  (ball_bearing m))

(cl:ensure-generic-function 'ball_radius-val :lambda-list '(m))
(cl:defmethod ball_radius-val ((m <Ball>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vision-msg:ball_radius-val is deprecated.  Use vision-msg:ball_radius instead.")
  (ball_radius m))

(cl:ensure-generic-function 'ball_center_x-val :lambda-list '(m))
(cl:defmethod ball_center_x-val ((m <Ball>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vision-msg:ball_center_x-val is deprecated.  Use vision-msg:ball_center_x instead.")
  (ball_center_x m))

(cl:ensure-generic-function 'ball_center_y-val :lambda-list '(m))
(cl:defmethod ball_center_y-val ((m <Ball>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vision-msg:ball_center_y-val is deprecated.  Use vision-msg:ball_center_y instead.")
  (ball_center_y m))

(cl:ensure-generic-function 'kick_time-val :lambda-list '(m))
(cl:defmethod kick_time-val ((m <Ball>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vision-msg:kick_time-val is deprecated.  Use vision-msg:kick_time instead.")
  (kick_time m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Ball>) ostream)
  "Serializes a message object of type '<Ball>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'ball_detected) 1 0)) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'ball_range))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'ball_bearing))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'ball_radius))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let* ((signed (cl:slot-value msg 'ball_center_x)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'ball_center_y)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'kick_time))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Ball>) istream)
  "Deserializes a message object of type '<Ball>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:slot-value msg 'ball_detected) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'ball_range) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'ball_bearing) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'ball_radius) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'ball_center_x) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'ball_center_y) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'kick_time) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Ball>)))
  "Returns string type for a message object of type '<Ball>"
  "vision/Ball")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Ball)))
  "Returns string type for a message object of type 'Ball"
  "vision/Ball")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Ball>)))
  "Returns md5sum for a message object of type '<Ball>"
  "07dd74c62bc00b3a12c5e1cd0e3a4159")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Ball)))
  "Returns md5sum for a message object of type 'Ball"
  "07dd74c62bc00b3a12c5e1cd0e3a4159")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Ball>)))
  "Returns full string definition for message of type '<Ball>"
  (cl:format cl:nil "Header header~%bool ball_detected~%float64 ball_range~%float64 ball_bearing~%float64 ball_radius~%int64 ball_center_x~%int64 ball_center_y~%float64 kick_time~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Ball)))
  "Returns full string definition for message of type 'Ball"
  (cl:format cl:nil "Header header~%bool ball_detected~%float64 ball_range~%float64 ball_bearing~%float64 ball_radius~%int64 ball_center_x~%int64 ball_center_y~%float64 kick_time~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Ball>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
     8
     8
     8
     8
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Ball>))
  "Converts a ROS message object to a list"
  (cl:list 'Ball
    (cl:cons ':header (header msg))
    (cl:cons ':ball_detected (ball_detected msg))
    (cl:cons ':ball_range (ball_range msg))
    (cl:cons ':ball_bearing (ball_bearing msg))
    (cl:cons ':ball_radius (ball_radius msg))
    (cl:cons ':ball_center_x (ball_center_x msg))
    (cl:cons ':ball_center_y (ball_center_y msg))
    (cl:cons ':kick_time (kick_time msg))
))
