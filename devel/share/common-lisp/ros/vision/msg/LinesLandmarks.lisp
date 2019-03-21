; Auto-generated. Do not edit!


(cl:in-package vision-msg)


;//! \htmlinclude LinesLandmarks.msg.html

(cl:defclass <LinesLandmarks> (roslisp-msg-protocol:ros-message)
  ((lines
    :reader lines
    :initarg :lines
    :type (cl:vector vision-msg:Line)
   :initform (cl:make-array 0 :element-type 'vision-msg:Line :initial-element (cl:make-instance 'vision-msg:Line)))
   (landmarks
    :reader landmarks
    :initarg :landmarks
    :type (cl:vector vision-msg:Landmark)
   :initform (cl:make-array 0 :element-type 'vision-msg:Landmark :initial-element (cl:make-instance 'vision-msg:Landmark))))
)

(cl:defclass LinesLandmarks (<LinesLandmarks>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <LinesLandmarks>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'LinesLandmarks)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name vision-msg:<LinesLandmarks> is deprecated: use vision-msg:LinesLandmarks instead.")))

(cl:ensure-generic-function 'lines-val :lambda-list '(m))
(cl:defmethod lines-val ((m <LinesLandmarks>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vision-msg:lines-val is deprecated.  Use vision-msg:lines instead.")
  (lines m))

(cl:ensure-generic-function 'landmarks-val :lambda-list '(m))
(cl:defmethod landmarks-val ((m <LinesLandmarks>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vision-msg:landmarks-val is deprecated.  Use vision-msg:landmarks instead.")
  (landmarks m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <LinesLandmarks>) ostream)
  "Serializes a message object of type '<LinesLandmarks>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'lines))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'lines))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'landmarks))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'landmarks))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <LinesLandmarks>) istream)
  "Deserializes a message object of type '<LinesLandmarks>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'lines) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'lines)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'vision-msg:Line))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'landmarks) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'landmarks)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'vision-msg:Landmark))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<LinesLandmarks>)))
  "Returns string type for a message object of type '<LinesLandmarks>"
  "vision/LinesLandmarks")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'LinesLandmarks)))
  "Returns string type for a message object of type 'LinesLandmarks"
  "vision/LinesLandmarks")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<LinesLandmarks>)))
  "Returns md5sum for a message object of type '<LinesLandmarks>"
  "fb66f810489cc88ff8dd15c871c9425d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'LinesLandmarks)))
  "Returns md5sum for a message object of type 'LinesLandmarks"
  "fb66f810489cc88ff8dd15c871c9425d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<LinesLandmarks>)))
  "Returns full string definition for message of type '<LinesLandmarks>"
  (cl:format cl:nil "Line[] lines~%Landmark[] landmarks~%~%================================================================================~%MSG: vision/Line~%float32 x1 #line start~%float32 y1~%float32 x2 #line end~%float32 y2~%~%================================================================================~%MSG: vision/Landmark~%geometry_msgs/Pose2D pose        # Pose~%uint8 type                       # Type (see localization::field_model::WorldObject::Type)~%float32 confidence               # confidence 0..1~%~%================================================================================~%MSG: geometry_msgs/Pose2D~%# Deprecated~%# Please use the full 3D pose.~%~%# In general our recommendation is to use a full 3D representation of everything and for 2D specific applications make the appropriate projections into the plane for their calculations but optimally will preserve the 3D information during processing.~%~%# If we have parallel copies of 2D datatypes every UI and other pipeline will end up needing to have dual interfaces to plot everything. And you will end up with not being able to use 3D tools for 2D use cases even if they're completely valid, as you'd have to reimplement it with different inputs and outputs. It's not particularly hard to plot the 2D pose or compute the yaw error for the Pose message and there are already tools and libraries that can do this for you.~%~%~%# This expresses a position and orientation on a 2D manifold.~%~%float64 x~%float64 y~%float64 theta~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'LinesLandmarks)))
  "Returns full string definition for message of type 'LinesLandmarks"
  (cl:format cl:nil "Line[] lines~%Landmark[] landmarks~%~%================================================================================~%MSG: vision/Line~%float32 x1 #line start~%float32 y1~%float32 x2 #line end~%float32 y2~%~%================================================================================~%MSG: vision/Landmark~%geometry_msgs/Pose2D pose        # Pose~%uint8 type                       # Type (see localization::field_model::WorldObject::Type)~%float32 confidence               # confidence 0..1~%~%================================================================================~%MSG: geometry_msgs/Pose2D~%# Deprecated~%# Please use the full 3D pose.~%~%# In general our recommendation is to use a full 3D representation of everything and for 2D specific applications make the appropriate projections into the plane for their calculations but optimally will preserve the 3D information during processing.~%~%# If we have parallel copies of 2D datatypes every UI and other pipeline will end up needing to have dual interfaces to plot everything. And you will end up with not being able to use 3D tools for 2D use cases even if they're completely valid, as you'd have to reimplement it with different inputs and outputs. It's not particularly hard to plot the 2D pose or compute the yaw error for the Pose message and there are already tools and libraries that can do this for you.~%~%~%# This expresses a position and orientation on a 2D manifold.~%~%float64 x~%float64 y~%float64 theta~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <LinesLandmarks>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'lines) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'landmarks) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <LinesLandmarks>))
  "Converts a ROS message object to a list"
  (cl:list 'LinesLandmarks
    (cl:cons ':lines (lines msg))
    (cl:cons ':landmarks (landmarks msg))
))
