; Auto-generated. Do not edit!


(cl:in-package decision-msg)


;//! \htmlinclude GoalData.msg.html

(cl:defclass <GoalData> (roslisp-msg-protocol:ros-message)
  ((goal
    :reader goal
    :initarg :goal
    :type cl:fixnum
    :initform 0)
   (leftx
    :reader leftx
    :initarg :leftx
    :type cl:float
    :initform 0.0)
   (lefty
    :reader lefty
    :initarg :lefty
    :type cl:float
    :initform 0.0)
   (rightx
    :reader rightx
    :initarg :rightx
    :type cl:float
    :initform 0.0)
   (righty
    :reader righty
    :initarg :righty
    :type cl:float
    :initform 0.0))
)

(cl:defclass GoalData (<GoalData>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GoalData>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GoalData)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name decision-msg:<GoalData> is deprecated: use decision-msg:GoalData instead.")))

(cl:ensure-generic-function 'goal-val :lambda-list '(m))
(cl:defmethod goal-val ((m <GoalData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader decision-msg:goal-val is deprecated.  Use decision-msg:goal instead.")
  (goal m))

(cl:ensure-generic-function 'leftx-val :lambda-list '(m))
(cl:defmethod leftx-val ((m <GoalData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader decision-msg:leftx-val is deprecated.  Use decision-msg:leftx instead.")
  (leftx m))

(cl:ensure-generic-function 'lefty-val :lambda-list '(m))
(cl:defmethod lefty-val ((m <GoalData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader decision-msg:lefty-val is deprecated.  Use decision-msg:lefty instead.")
  (lefty m))

(cl:ensure-generic-function 'rightx-val :lambda-list '(m))
(cl:defmethod rightx-val ((m <GoalData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader decision-msg:rightx-val is deprecated.  Use decision-msg:rightx instead.")
  (rightx m))

(cl:ensure-generic-function 'righty-val :lambda-list '(m))
(cl:defmethod righty-val ((m <GoalData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader decision-msg:righty-val is deprecated.  Use decision-msg:righty instead.")
  (righty m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GoalData>) ostream)
  "Serializes a message object of type '<GoalData>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'goal)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'leftx))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'lefty))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'rightx))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'righty))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GoalData>) istream)
  "Deserializes a message object of type '<GoalData>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'goal)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'leftx) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'lefty) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'rightx) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'righty) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GoalData>)))
  "Returns string type for a message object of type '<GoalData>"
  "decision/GoalData")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GoalData)))
  "Returns string type for a message object of type 'GoalData"
  "decision/GoalData")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GoalData>)))
  "Returns md5sum for a message object of type '<GoalData>"
  "657b4b89e12eaea48f7336974eb25b11")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GoalData)))
  "Returns md5sum for a message object of type 'GoalData"
  "657b4b89e12eaea48f7336974eb25b11")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GoalData>)))
  "Returns full string definition for message of type '<GoalData>"
  (cl:format cl:nil "uint8 goal~%float32 leftx~%float32 lefty~%float32 rightx~%float32 righty~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GoalData)))
  "Returns full string definition for message of type 'GoalData"
  (cl:format cl:nil "uint8 goal~%float32 leftx~%float32 lefty~%float32 rightx~%float32 righty~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GoalData>))
  (cl:+ 0
     1
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GoalData>))
  "Converts a ROS message object to a list"
  (cl:list 'GoalData
    (cl:cons ':goal (goal msg))
    (cl:cons ':leftx (leftx msg))
    (cl:cons ':lefty (lefty msg))
    (cl:cons ':rightx (rightx msg))
    (cl:cons ':righty (righty msg))
))
