; Auto-generated. Do not edit!


(cl:in-package decision-msg)


;//! \htmlinclude SerialReceived.msg.html

(cl:defclass <SerialReceived> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (received_data
    :reader received_data
    :initarg :received_data
    :type (cl:vector cl:float)
   :initform (cl:make-array 20 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass SerialReceived (<SerialReceived>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SerialReceived>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SerialReceived)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name decision-msg:<SerialReceived> is deprecated: use decision-msg:SerialReceived instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <SerialReceived>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader decision-msg:header-val is deprecated.  Use decision-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'received_data-val :lambda-list '(m))
(cl:defmethod received_data-val ((m <SerialReceived>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader decision-msg:received_data-val is deprecated.  Use decision-msg:received_data instead.")
  (received_data m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SerialReceived>) ostream)
  "Serializes a message object of type '<SerialReceived>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'received_data))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SerialReceived>) istream)
  "Deserializes a message object of type '<SerialReceived>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:setf (cl:slot-value msg 'received_data) (cl:make-array 20))
  (cl:let ((vals (cl:slot-value msg 'received_data)))
    (cl:dotimes (i 20)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SerialReceived>)))
  "Returns string type for a message object of type '<SerialReceived>"
  "decision/SerialReceived")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SerialReceived)))
  "Returns string type for a message object of type 'SerialReceived"
  "decision/SerialReceived")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SerialReceived>)))
  "Returns md5sum for a message object of type '<SerialReceived>"
  "dbd49299a567b5d4ac76462b0080eaca")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SerialReceived)))
  "Returns md5sum for a message object of type 'SerialReceived"
  "dbd49299a567b5d4ac76462b0080eaca")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SerialReceived>)))
  "Returns full string definition for message of type '<SerialReceived>"
  (cl:format cl:nil "std_msgs/Header header~%float32[20] received_data~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SerialReceived)))
  "Returns full string definition for message of type 'SerialReceived"
  (cl:format cl:nil "std_msgs/Header header~%float32[20] received_data~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SerialReceived>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'received_data) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SerialReceived>))
  "Converts a ROS message object to a list"
  (cl:list 'SerialReceived
    (cl:cons ':header (header msg))
    (cl:cons ':received_data (received_data msg))
))
