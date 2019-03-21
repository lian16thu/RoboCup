; Auto-generated. Do not edit!


(cl:in-package vision-msg)


;//! \htmlinclude Lines.msg.html

(cl:defclass <Lines> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (lines_number
    :reader lines_number
    :initarg :lines_number
    :type cl:fixnum
    :initform 0)
   (line_start_x
    :reader line_start_x
    :initarg :line_start_x
    :type (cl:vector cl:float)
   :initform (cl:make-array 20 :element-type 'cl:float :initial-element 0.0))
   (line_start_y
    :reader line_start_y
    :initarg :line_start_y
    :type (cl:vector cl:float)
   :initform (cl:make-array 20 :element-type 'cl:float :initial-element 0.0))
   (line_end_x
    :reader line_end_x
    :initarg :line_end_x
    :type (cl:vector cl:float)
   :initform (cl:make-array 20 :element-type 'cl:float :initial-element 0.0))
   (line_end_y
    :reader line_end_y
    :initarg :line_end_y
    :type (cl:vector cl:float)
   :initform (cl:make-array 20 :element-type 'cl:float :initial-element 0.0))
   (line_confidence
    :reader line_confidence
    :initarg :line_confidence
    :type (cl:vector cl:float)
   :initform (cl:make-array 20 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass Lines (<Lines>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Lines>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Lines)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name vision-msg:<Lines> is deprecated: use vision-msg:Lines instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <Lines>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vision-msg:header-val is deprecated.  Use vision-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'lines_number-val :lambda-list '(m))
(cl:defmethod lines_number-val ((m <Lines>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vision-msg:lines_number-val is deprecated.  Use vision-msg:lines_number instead.")
  (lines_number m))

(cl:ensure-generic-function 'line_start_x-val :lambda-list '(m))
(cl:defmethod line_start_x-val ((m <Lines>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vision-msg:line_start_x-val is deprecated.  Use vision-msg:line_start_x instead.")
  (line_start_x m))

(cl:ensure-generic-function 'line_start_y-val :lambda-list '(m))
(cl:defmethod line_start_y-val ((m <Lines>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vision-msg:line_start_y-val is deprecated.  Use vision-msg:line_start_y instead.")
  (line_start_y m))

(cl:ensure-generic-function 'line_end_x-val :lambda-list '(m))
(cl:defmethod line_end_x-val ((m <Lines>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vision-msg:line_end_x-val is deprecated.  Use vision-msg:line_end_x instead.")
  (line_end_x m))

(cl:ensure-generic-function 'line_end_y-val :lambda-list '(m))
(cl:defmethod line_end_y-val ((m <Lines>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vision-msg:line_end_y-val is deprecated.  Use vision-msg:line_end_y instead.")
  (line_end_y m))

(cl:ensure-generic-function 'line_confidence-val :lambda-list '(m))
(cl:defmethod line_confidence-val ((m <Lines>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vision-msg:line_confidence-val is deprecated.  Use vision-msg:line_confidence instead.")
  (line_confidence m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Lines>) ostream)
  "Serializes a message object of type '<Lines>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'lines_number)) ostream)
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'line_start_x))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'line_start_y))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'line_end_x))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'line_end_y))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'line_confidence))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Lines>) istream)
  "Deserializes a message object of type '<Lines>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'lines_number)) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'line_start_x) (cl:make-array 20))
  (cl:let ((vals (cl:slot-value msg 'line_start_x)))
    (cl:dotimes (i 20)
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
  (cl:setf (cl:slot-value msg 'line_start_y) (cl:make-array 20))
  (cl:let ((vals (cl:slot-value msg 'line_start_y)))
    (cl:dotimes (i 20)
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
  (cl:setf (cl:slot-value msg 'line_end_x) (cl:make-array 20))
  (cl:let ((vals (cl:slot-value msg 'line_end_x)))
    (cl:dotimes (i 20)
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
  (cl:setf (cl:slot-value msg 'line_end_y) (cl:make-array 20))
  (cl:let ((vals (cl:slot-value msg 'line_end_y)))
    (cl:dotimes (i 20)
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
  (cl:setf (cl:slot-value msg 'line_confidence) (cl:make-array 20))
  (cl:let ((vals (cl:slot-value msg 'line_confidence)))
    (cl:dotimes (i 20)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Lines>)))
  "Returns string type for a message object of type '<Lines>"
  "vision/Lines")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Lines)))
  "Returns string type for a message object of type 'Lines"
  "vision/Lines")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Lines>)))
  "Returns md5sum for a message object of type '<Lines>"
  "26fb3619336498992b94003d734b3dd9")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Lines)))
  "Returns md5sum for a message object of type 'Lines"
  "26fb3619336498992b94003d734b3dd9")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Lines>)))
  "Returns full string definition for message of type '<Lines>"
  (cl:format cl:nil "Header header~%uint8 lines_number  ~%float64[20] line_start_x                 ~%float64[20] line_start_y ~%float64[20] line_end_x                 ~%float64[20] line_end_y               ~%float32[20] line_confidence               # confidence 0..1~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Lines)))
  "Returns full string definition for message of type 'Lines"
  (cl:format cl:nil "Header header~%uint8 lines_number  ~%float64[20] line_start_x                 ~%float64[20] line_start_y ~%float64[20] line_end_x                 ~%float64[20] line_end_y               ~%float32[20] line_confidence               # confidence 0..1~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Lines>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'line_start_x) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'line_start_y) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'line_end_x) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'line_end_y) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'line_confidence) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Lines>))
  "Converts a ROS message object to a list"
  (cl:list 'Lines
    (cl:cons ':header (header msg))
    (cl:cons ':lines_number (lines_number msg))
    (cl:cons ':line_start_x (line_start_x msg))
    (cl:cons ':line_start_y (line_start_y msg))
    (cl:cons ':line_end_x (line_end_x msg))
    (cl:cons ':line_end_y (line_end_y msg))
    (cl:cons ':line_confidence (line_confidence msg))
))
