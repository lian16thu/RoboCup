; Auto-generated. Do not edit!


(cl:in-package localization-msg)


;//! \htmlinclude LinesDetected.msg.html

(cl:defclass <LinesDetected> (roslisp-msg-protocol:ros-message)
  ((x1
    :reader x1
    :initarg :x1
    :type cl:float
    :initform 0.0)
   (y1
    :reader y1
    :initarg :y1
    :type cl:float
    :initform 0.0)
   (x2
    :reader x2
    :initarg :x2
    :type cl:float
    :initform 0.0)
   (y2
    :reader y2
    :initarg :y2
    :type cl:float
    :initform 0.0))
)

(cl:defclass LinesDetected (<LinesDetected>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <LinesDetected>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'LinesDetected)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name localization-msg:<LinesDetected> is deprecated: use localization-msg:LinesDetected instead.")))

(cl:ensure-generic-function 'x1-val :lambda-list '(m))
(cl:defmethod x1-val ((m <LinesDetected>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader localization-msg:x1-val is deprecated.  Use localization-msg:x1 instead.")
  (x1 m))

(cl:ensure-generic-function 'y1-val :lambda-list '(m))
(cl:defmethod y1-val ((m <LinesDetected>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader localization-msg:y1-val is deprecated.  Use localization-msg:y1 instead.")
  (y1 m))

(cl:ensure-generic-function 'x2-val :lambda-list '(m))
(cl:defmethod x2-val ((m <LinesDetected>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader localization-msg:x2-val is deprecated.  Use localization-msg:x2 instead.")
  (x2 m))

(cl:ensure-generic-function 'y2-val :lambda-list '(m))
(cl:defmethod y2-val ((m <LinesDetected>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader localization-msg:y2-val is deprecated.  Use localization-msg:y2 instead.")
  (y2 m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <LinesDetected>) ostream)
  "Serializes a message object of type '<LinesDetected>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'x1))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'y1))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'x2))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'y2))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <LinesDetected>) istream)
  "Deserializes a message object of type '<LinesDetected>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'x1) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'y1) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'x2) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'y2) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<LinesDetected>)))
  "Returns string type for a message object of type '<LinesDetected>"
  "localization/LinesDetected")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'LinesDetected)))
  "Returns string type for a message object of type 'LinesDetected"
  "localization/LinesDetected")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<LinesDetected>)))
  "Returns md5sum for a message object of type '<LinesDetected>"
  "1d74979d8119401281d48677f845994f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'LinesDetected)))
  "Returns md5sum for a message object of type 'LinesDetected"
  "1d74979d8119401281d48677f845994f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<LinesDetected>)))
  "Returns full string definition for message of type '<LinesDetected>"
  (cl:format cl:nil "float32 x1~%float32 y1~%float32 x2~%float32 y2~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'LinesDetected)))
  "Returns full string definition for message of type 'LinesDetected"
  (cl:format cl:nil "float32 x1~%float32 y1~%float32 x2~%float32 y2~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <LinesDetected>))
  (cl:+ 0
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <LinesDetected>))
  "Converts a ROS message object to a list"
  (cl:list 'LinesDetected
    (cl:cons ':x1 (x1 msg))
    (cl:cons ':y1 (y1 msg))
    (cl:cons ':x2 (x2 msg))
    (cl:cons ':y2 (y2 msg))
))
