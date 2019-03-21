; Auto-generated. Do not edit!


(cl:in-package localization-msg)


;//! \htmlinclude WorldObjects.msg.html

(cl:defclass <WorldObjects> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (objects
    :reader objects
    :initarg :objects
    :type (cl:vector localization-msg:ObjectsDetected)
   :initform (cl:make-array 0 :element-type 'localization-msg:ObjectsDetected :initial-element (cl:make-instance 'localization-msg:ObjectsDetected)))
   (goalposts
    :reader goalposts
    :initarg :goalposts
    :type (cl:vector localization-msg:GoalpostsDetected)
   :initform (cl:make-array 0 :element-type 'localization-msg:GoalpostsDetected :initial-element (cl:make-instance 'localization-msg:GoalpostsDetected)))
   (lines
    :reader lines
    :initarg :lines
    :type (cl:vector localization-msg:LinesDetected)
   :initform (cl:make-array 0 :element-type 'localization-msg:LinesDetected :initial-element (cl:make-instance 'localization-msg:LinesDetected)))
   (obstacles
    :reader obstacles
    :initarg :obstacles
    :type (cl:vector localization-msg:ObstaclesDetected)
   :initform (cl:make-array 0 :element-type 'localization-msg:ObstaclesDetected :initial-element (cl:make-instance 'localization-msg:ObstaclesDetected))))
)

(cl:defclass WorldObjects (<WorldObjects>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <WorldObjects>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'WorldObjects)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name localization-msg:<WorldObjects> is deprecated: use localization-msg:WorldObjects instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <WorldObjects>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader localization-msg:header-val is deprecated.  Use localization-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'objects-val :lambda-list '(m))
(cl:defmethod objects-val ((m <WorldObjects>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader localization-msg:objects-val is deprecated.  Use localization-msg:objects instead.")
  (objects m))

(cl:ensure-generic-function 'goalposts-val :lambda-list '(m))
(cl:defmethod goalposts-val ((m <WorldObjects>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader localization-msg:goalposts-val is deprecated.  Use localization-msg:goalposts instead.")
  (goalposts m))

(cl:ensure-generic-function 'lines-val :lambda-list '(m))
(cl:defmethod lines-val ((m <WorldObjects>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader localization-msg:lines-val is deprecated.  Use localization-msg:lines instead.")
  (lines m))

(cl:ensure-generic-function 'obstacles-val :lambda-list '(m))
(cl:defmethod obstacles-val ((m <WorldObjects>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader localization-msg:obstacles-val is deprecated.  Use localization-msg:obstacles instead.")
  (obstacles m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <WorldObjects>) ostream)
  "Serializes a message object of type '<WorldObjects>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'objects))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'objects))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'goalposts))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'goalposts))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'lines))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'lines))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'obstacles))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'obstacles))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <WorldObjects>) istream)
  "Deserializes a message object of type '<WorldObjects>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'objects) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'objects)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'localization-msg:ObjectsDetected))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'goalposts) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'goalposts)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'localization-msg:GoalpostsDetected))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'lines) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'lines)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'localization-msg:LinesDetected))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'obstacles) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'obstacles)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'localization-msg:ObstaclesDetected))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<WorldObjects>)))
  "Returns string type for a message object of type '<WorldObjects>"
  "localization/WorldObjects")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'WorldObjects)))
  "Returns string type for a message object of type 'WorldObjects"
  "localization/WorldObjects")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<WorldObjects>)))
  "Returns md5sum for a message object of type '<WorldObjects>"
  "b3b402e0a736a3c111358cf72848e29c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'WorldObjects)))
  "Returns md5sum for a message object of type 'WorldObjects"
  "b3b402e0a736a3c111358cf72848e29c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<WorldObjects>)))
  "Returns full string definition for message of type '<WorldObjects>"
  (cl:format cl:nil "Header header~%#add more for the lines and such, like in detections~%ObjectsDetected[] objects~%GoalpostsDetected[] goalposts~%LinesDetected[] lines~%ObstaclesDetected[] obstacles~%~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: localization/ObjectsDetected~%geometry_msgs/Pose2D pose        # Pose~%uint8 type                       # Type (see field_model::WorldObject::Type)~%float32 confidence               # confidence 0..1~%~%================================================================================~%MSG: geometry_msgs/Pose2D~%# Deprecated~%# Please use the full 3D pose.~%~%# In general our recommendation is to use a full 3D representation of everything and for 2D specific applications make the appropriate projections into the plane for their calculations but optimally will preserve the 3D information during processing.~%~%# If we have parallel copies of 2D datatypes every UI and other pipeline will end up needing to have dual interfaces to plot everything. And you will end up with not being able to use 3D tools for 2D use cases even if they're completely valid, as you'd have to reimplement it with different inputs and outputs. It's not particularly hard to plot the 2D pose or compute the yaw error for the Pose message and there are already tools and libraries that can do this for you.~%~%~%# This expresses a position and orientation on a 2D manifold.~%~%float64 x~%float64 y~%float64 theta~%~%================================================================================~%MSG: localization/GoalpostsDetected~%geometry_msgs/Pose2D pose        # Pose~%uint8 type                       # Type (see field_model::WorldObject::Type)~%float32 confidence               # confidence 0..1~%~%================================================================================~%MSG: localization/LinesDetected~%float32 x1~%float32 y1~%float32 x2~%float32 y2~%~%================================================================================~%MSG: localization/ObstaclesDetected~%geometry_msgs/Pose2D pose        # Pose~%uint8 type                       # Type 0 for opponent, 1 for other obstacle~%float32 confidence               # confidence 0..1~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'WorldObjects)))
  "Returns full string definition for message of type 'WorldObjects"
  (cl:format cl:nil "Header header~%#add more for the lines and such, like in detections~%ObjectsDetected[] objects~%GoalpostsDetected[] goalposts~%LinesDetected[] lines~%ObstaclesDetected[] obstacles~%~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: localization/ObjectsDetected~%geometry_msgs/Pose2D pose        # Pose~%uint8 type                       # Type (see field_model::WorldObject::Type)~%float32 confidence               # confidence 0..1~%~%================================================================================~%MSG: geometry_msgs/Pose2D~%# Deprecated~%# Please use the full 3D pose.~%~%# In general our recommendation is to use a full 3D representation of everything and for 2D specific applications make the appropriate projections into the plane for their calculations but optimally will preserve the 3D information during processing.~%~%# If we have parallel copies of 2D datatypes every UI and other pipeline will end up needing to have dual interfaces to plot everything. And you will end up with not being able to use 3D tools for 2D use cases even if they're completely valid, as you'd have to reimplement it with different inputs and outputs. It's not particularly hard to plot the 2D pose or compute the yaw error for the Pose message and there are already tools and libraries that can do this for you.~%~%~%# This expresses a position and orientation on a 2D manifold.~%~%float64 x~%float64 y~%float64 theta~%~%================================================================================~%MSG: localization/GoalpostsDetected~%geometry_msgs/Pose2D pose        # Pose~%uint8 type                       # Type (see field_model::WorldObject::Type)~%float32 confidence               # confidence 0..1~%~%================================================================================~%MSG: localization/LinesDetected~%float32 x1~%float32 y1~%float32 x2~%float32 y2~%~%================================================================================~%MSG: localization/ObstaclesDetected~%geometry_msgs/Pose2D pose        # Pose~%uint8 type                       # Type 0 for opponent, 1 for other obstacle~%float32 confidence               # confidence 0..1~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <WorldObjects>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'objects) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'goalposts) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'lines) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'obstacles) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <WorldObjects>))
  "Converts a ROS message object to a list"
  (cl:list 'WorldObjects
    (cl:cons ':header (header msg))
    (cl:cons ':objects (objects msg))
    (cl:cons ':goalposts (goalposts msg))
    (cl:cons ':lines (lines msg))
    (cl:cons ':obstacles (obstacles msg))
))
