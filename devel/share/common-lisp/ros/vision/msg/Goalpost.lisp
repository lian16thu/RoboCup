; Auto-generated. Do not edit!


(cl:in-package vision-msg)


;//! \htmlinclude Goalpost.msg.html

(cl:defclass <Goalpost> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (goalpost_detected
    :reader goalpost_detected
    :initarg :goalpost_detected
    :type cl:boolean
    :initform cl:nil)
   (goalpost_number
    :reader goalpost_number
    :initarg :goalpost_number
    :type cl:fixnum
    :initform 0)
   (goalpost_left_range
    :reader goalpost_left_range
    :initarg :goalpost_left_range
    :type cl:float
    :initform 0.0)
   (goalpost_left_bearing
    :reader goalpost_left_bearing
    :initarg :goalpost_left_bearing
    :type cl:float
    :initform 0.0)
   (goalpost_right_range
    :reader goalpost_right_range
    :initarg :goalpost_right_range
    :type cl:float
    :initform 0.0)
   (goalpost_right_bearing
    :reader goalpost_right_bearing
    :initarg :goalpost_right_bearing
    :type cl:float
    :initform 0.0)
   (goalpost_center_range
    :reader goalpost_center_range
    :initarg :goalpost_center_range
    :type cl:float
    :initform 0.0)
   (goalpost_center_bearing
    :reader goalpost_center_bearing
    :initarg :goalpost_center_bearing
    :type cl:float
    :initform 0.0))
)

(cl:defclass Goalpost (<Goalpost>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Goalpost>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Goalpost)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name vision-msg:<Goalpost> is deprecated: use vision-msg:Goalpost instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <Goalpost>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vision-msg:header-val is deprecated.  Use vision-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'goalpost_detected-val :lambda-list '(m))
(cl:defmethod goalpost_detected-val ((m <Goalpost>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vision-msg:goalpost_detected-val is deprecated.  Use vision-msg:goalpost_detected instead.")
  (goalpost_detected m))

(cl:ensure-generic-function 'goalpost_number-val :lambda-list '(m))
(cl:defmethod goalpost_number-val ((m <Goalpost>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vision-msg:goalpost_number-val is deprecated.  Use vision-msg:goalpost_number instead.")
  (goalpost_number m))

(cl:ensure-generic-function 'goalpost_left_range-val :lambda-list '(m))
(cl:defmethod goalpost_left_range-val ((m <Goalpost>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vision-msg:goalpost_left_range-val is deprecated.  Use vision-msg:goalpost_left_range instead.")
  (goalpost_left_range m))

(cl:ensure-generic-function 'goalpost_left_bearing-val :lambda-list '(m))
(cl:defmethod goalpost_left_bearing-val ((m <Goalpost>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vision-msg:goalpost_left_bearing-val is deprecated.  Use vision-msg:goalpost_left_bearing instead.")
  (goalpost_left_bearing m))

(cl:ensure-generic-function 'goalpost_right_range-val :lambda-list '(m))
(cl:defmethod goalpost_right_range-val ((m <Goalpost>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vision-msg:goalpost_right_range-val is deprecated.  Use vision-msg:goalpost_right_range instead.")
  (goalpost_right_range m))

(cl:ensure-generic-function 'goalpost_right_bearing-val :lambda-list '(m))
(cl:defmethod goalpost_right_bearing-val ((m <Goalpost>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vision-msg:goalpost_right_bearing-val is deprecated.  Use vision-msg:goalpost_right_bearing instead.")
  (goalpost_right_bearing m))

(cl:ensure-generic-function 'goalpost_center_range-val :lambda-list '(m))
(cl:defmethod goalpost_center_range-val ((m <Goalpost>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vision-msg:goalpost_center_range-val is deprecated.  Use vision-msg:goalpost_center_range instead.")
  (goalpost_center_range m))

(cl:ensure-generic-function 'goalpost_center_bearing-val :lambda-list '(m))
(cl:defmethod goalpost_center_bearing-val ((m <Goalpost>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vision-msg:goalpost_center_bearing-val is deprecated.  Use vision-msg:goalpost_center_bearing instead.")
  (goalpost_center_bearing m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Goalpost>) ostream)
  "Serializes a message object of type '<Goalpost>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'goalpost_detected) 1 0)) ostream)
  (cl:let* ((signed (cl:slot-value msg 'goalpost_number)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'goalpost_left_range))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'goalpost_left_bearing))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'goalpost_right_range))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'goalpost_right_bearing))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'goalpost_center_range))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'goalpost_center_bearing))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Goalpost>) istream)
  "Deserializes a message object of type '<Goalpost>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:slot-value msg 'goalpost_detected) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'goalpost_number) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'goalpost_left_range) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'goalpost_left_bearing) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'goalpost_right_range) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'goalpost_right_bearing) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'goalpost_center_range) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'goalpost_center_bearing) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Goalpost>)))
  "Returns string type for a message object of type '<Goalpost>"
  "vision/Goalpost")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Goalpost)))
  "Returns string type for a message object of type 'Goalpost"
  "vision/Goalpost")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Goalpost>)))
  "Returns md5sum for a message object of type '<Goalpost>"
  "c7e85911d22877f743fec7cea6c072ce")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Goalpost)))
  "Returns md5sum for a message object of type 'Goalpost"
  "c7e85911d22877f743fec7cea6c072ce")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Goalpost>)))
  "Returns full string definition for message of type '<Goalpost>"
  (cl:format cl:nil "Header header~%bool goalpost_detected~%int8 goalpost_number   # 3 if only see the middle bar of goalpost. then send out the left and right point as left goalpost and right goalpost~%float64 goalpost_left_range~%float64 goalpost_left_bearing~%float64 goalpost_right_range~%float64 goalpost_right_bearing~%float64 goalpost_center_range~%float64 goalpost_center_bearing~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Goalpost)))
  "Returns full string definition for message of type 'Goalpost"
  (cl:format cl:nil "Header header~%bool goalpost_detected~%int8 goalpost_number   # 3 if only see the middle bar of goalpost. then send out the left and right point as left goalpost and right goalpost~%float64 goalpost_left_range~%float64 goalpost_left_bearing~%float64 goalpost_right_range~%float64 goalpost_right_bearing~%float64 goalpost_center_range~%float64 goalpost_center_bearing~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Goalpost>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
     1
     8
     8
     8
     8
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Goalpost>))
  "Converts a ROS message object to a list"
  (cl:list 'Goalpost
    (cl:cons ':header (header msg))
    (cl:cons ':goalpost_detected (goalpost_detected msg))
    (cl:cons ':goalpost_number (goalpost_number msg))
    (cl:cons ':goalpost_left_range (goalpost_left_range msg))
    (cl:cons ':goalpost_left_bearing (goalpost_left_bearing msg))
    (cl:cons ':goalpost_right_range (goalpost_right_range msg))
    (cl:cons ':goalpost_right_bearing (goalpost_right_bearing msg))
    (cl:cons ':goalpost_center_range (goalpost_center_range msg))
    (cl:cons ':goalpost_center_bearing (goalpost_center_bearing msg))
))
