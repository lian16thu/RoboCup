; Auto-generated. Do not edit!


(cl:in-package decision-msg)


;//! \htmlinclude gyro_euler.msg.html

(cl:defclass <gyro_euler> (roslisp-msg-protocol:ros-message)
  ((euler_angle
    :reader euler_angle
    :initarg :euler_angle
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3)))
)

(cl:defclass gyro_euler (<gyro_euler>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <gyro_euler>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'gyro_euler)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name decision-msg:<gyro_euler> is deprecated: use decision-msg:gyro_euler instead.")))

(cl:ensure-generic-function 'euler_angle-val :lambda-list '(m))
(cl:defmethod euler_angle-val ((m <gyro_euler>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader decision-msg:euler_angle-val is deprecated.  Use decision-msg:euler_angle instead.")
  (euler_angle m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <gyro_euler>) ostream)
  "Serializes a message object of type '<gyro_euler>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'euler_angle) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <gyro_euler>) istream)
  "Deserializes a message object of type '<gyro_euler>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'euler_angle) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<gyro_euler>)))
  "Returns string type for a message object of type '<gyro_euler>"
  "decision/gyro_euler")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'gyro_euler)))
  "Returns string type for a message object of type 'gyro_euler"
  "decision/gyro_euler")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<gyro_euler>)))
  "Returns md5sum for a message object of type '<gyro_euler>"
  "a0ed725f607be591af507cbf7836e5ed")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'gyro_euler)))
  "Returns md5sum for a message object of type 'gyro_euler"
  "a0ed725f607be591af507cbf7836e5ed")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<gyro_euler>)))
  "Returns full string definition for message of type '<gyro_euler>"
  (cl:format cl:nil "geometry_msgs/Vector3 euler_angle~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'gyro_euler)))
  "Returns full string definition for message of type 'gyro_euler"
  (cl:format cl:nil "geometry_msgs/Vector3 euler_angle~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <gyro_euler>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'euler_angle))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <gyro_euler>))
  "Converts a ROS message object to a list"
  (cl:list 'gyro_euler
    (cl:cons ':euler_angle (euler_angle msg))
))
