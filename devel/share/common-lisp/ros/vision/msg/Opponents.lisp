; Auto-generated. Do not edit!


(cl:in-package vision-msg)


;//! \htmlinclude Opponents.msg.html

(cl:defclass <Opponents> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (opponent_detected
    :reader opponent_detected
    :initarg :opponent_detected
    :type cl:boolean
    :initform cl:nil)
   (opponent_number
    :reader opponent_number
    :initarg :opponent_number
    :type cl:fixnum
    :initform 0)
   (opponent_range
    :reader opponent_range
    :initarg :opponent_range
    :type (cl:vector cl:float)
   :initform (cl:make-array 5 :element-type 'cl:float :initial-element 0.0))
   (opponent_bearing
    :reader opponent_bearing
    :initarg :opponent_bearing
    :type (cl:vector cl:float)
   :initform (cl:make-array 5 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass Opponents (<Opponents>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Opponents>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Opponents)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name vision-msg:<Opponents> is deprecated: use vision-msg:Opponents instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <Opponents>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vision-msg:header-val is deprecated.  Use vision-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'opponent_detected-val :lambda-list '(m))
(cl:defmethod opponent_detected-val ((m <Opponents>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vision-msg:opponent_detected-val is deprecated.  Use vision-msg:opponent_detected instead.")
  (opponent_detected m))

(cl:ensure-generic-function 'opponent_number-val :lambda-list '(m))
(cl:defmethod opponent_number-val ((m <Opponents>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vision-msg:opponent_number-val is deprecated.  Use vision-msg:opponent_number instead.")
  (opponent_number m))

(cl:ensure-generic-function 'opponent_range-val :lambda-list '(m))
(cl:defmethod opponent_range-val ((m <Opponents>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vision-msg:opponent_range-val is deprecated.  Use vision-msg:opponent_range instead.")
  (opponent_range m))

(cl:ensure-generic-function 'opponent_bearing-val :lambda-list '(m))
(cl:defmethod opponent_bearing-val ((m <Opponents>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vision-msg:opponent_bearing-val is deprecated.  Use vision-msg:opponent_bearing instead.")
  (opponent_bearing m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Opponents>) ostream)
  "Serializes a message object of type '<Opponents>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'opponent_detected) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'opponent_number)) ostream)
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'opponent_range))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'opponent_bearing))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Opponents>) istream)
  "Deserializes a message object of type '<Opponents>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:slot-value msg 'opponent_detected) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'opponent_number)) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'opponent_range) (cl:make-array 5))
  (cl:let ((vals (cl:slot-value msg 'opponent_range)))
    (cl:dotimes (i 5)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits)))))
  (cl:setf (cl:slot-value msg 'opponent_bearing) (cl:make-array 5))
  (cl:let ((vals (cl:slot-value msg 'opponent_bearing)))
    (cl:dotimes (i 5)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Opponents>)))
  "Returns string type for a message object of type '<Opponents>"
  "vision/Opponents")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Opponents)))
  "Returns string type for a message object of type 'Opponents"
  "vision/Opponents")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Opponents>)))
  "Returns md5sum for a message object of type '<Opponents>"
  "ef41a5e204617f0c2b713c70ec7cf155")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Opponents)))
  "Returns md5sum for a message object of type 'Opponents"
  "ef41a5e204617f0c2b713c70ec7cf155")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Opponents>)))
  "Returns full string definition for message of type '<Opponents>"
  (cl:format cl:nil "Header header~%bool opponent_detected~%uint8 opponent_number~%float64[5] opponent_range~%float64[5] opponent_bearing~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Opponents)))
  "Returns full string definition for message of type 'Opponents"
  (cl:format cl:nil "Header header~%bool opponent_detected~%uint8 opponent_number~%float64[5] opponent_range~%float64[5] opponent_bearing~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Opponents>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
     1
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'opponent_range) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'opponent_bearing) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Opponents>))
  "Converts a ROS message object to a list"
  (cl:list 'Opponents
    (cl:cons ':header (header msg))
    (cl:cons ':opponent_detected (opponent_detected msg))
    (cl:cons ':opponent_number (opponent_number msg))
    (cl:cons ':opponent_range (opponent_range msg))
    (cl:cons ':opponent_bearing (opponent_bearing msg))
))
