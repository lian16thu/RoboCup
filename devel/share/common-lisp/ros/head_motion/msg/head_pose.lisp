; Auto-generated. Do not edit!


(cl:in-package head_motion-msg)


;//! \htmlinclude head_pose.msg.html

(cl:defclass <head_pose> (roslisp-msg-protocol:ros-message)
  ((yaw
    :reader yaw
    :initarg :yaw
    :type cl:integer
    :initform 0)
   (pitch
    :reader pitch
    :initarg :pitch
    :type cl:integer
    :initform 0))
)

(cl:defclass head_pose (<head_pose>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <head_pose>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'head_pose)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name head_motion-msg:<head_pose> is deprecated: use head_motion-msg:head_pose instead.")))

(cl:ensure-generic-function 'yaw-val :lambda-list '(m))
(cl:defmethod yaw-val ((m <head_pose>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader head_motion-msg:yaw-val is deprecated.  Use head_motion-msg:yaw instead.")
  (yaw m))

(cl:ensure-generic-function 'pitch-val :lambda-list '(m))
(cl:defmethod pitch-val ((m <head_pose>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader head_motion-msg:pitch-val is deprecated.  Use head_motion-msg:pitch instead.")
  (pitch m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <head_pose>) ostream)
  "Serializes a message object of type '<head_pose>"
  (cl:let* ((signed (cl:slot-value msg 'yaw)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'pitch)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <head_pose>) istream)
  "Deserializes a message object of type '<head_pose>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'yaw) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'pitch) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<head_pose>)))
  "Returns string type for a message object of type '<head_pose>"
  "head_motion/head_pose")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'head_pose)))
  "Returns string type for a message object of type 'head_pose"
  "head_motion/head_pose")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<head_pose>)))
  "Returns md5sum for a message object of type '<head_pose>"
  "a639ba30f63c6c2e9b511b1c6df08e3a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'head_pose)))
  "Returns md5sum for a message object of type 'head_pose"
  "a639ba30f63c6c2e9b511b1c6df08e3a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<head_pose>)))
  "Returns full string definition for message of type '<head_pose>"
  (cl:format cl:nil "int64 yaw~%int64 pitch~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'head_pose)))
  "Returns full string definition for message of type 'head_pose"
  (cl:format cl:nil "int64 yaw~%int64 pitch~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <head_pose>))
  (cl:+ 0
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <head_pose>))
  "Converts a ROS message object to a list"
  (cl:list 'head_pose
    (cl:cons ':yaw (yaw msg))
    (cl:cons ':pitch (pitch msg))
))
