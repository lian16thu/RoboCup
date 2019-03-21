; Auto-generated. Do not edit!


(cl:in-package decision-msg)


;//! \htmlinclude gameControl.msg.html

(cl:defclass <gameControl> (roslisp-msg-protocol:ros-message)
  ((state
    :reader state
    :initarg :state
    :type cl:integer
    :initform 0)
   (secsRemaining
    :reader secsRemaining
    :initarg :secsRemaining
    :type cl:integer
    :initform 0))
)

(cl:defclass gameControl (<gameControl>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <gameControl>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'gameControl)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name decision-msg:<gameControl> is deprecated: use decision-msg:gameControl instead.")))

(cl:ensure-generic-function 'state-val :lambda-list '(m))
(cl:defmethod state-val ((m <gameControl>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader decision-msg:state-val is deprecated.  Use decision-msg:state instead.")
  (state m))

(cl:ensure-generic-function 'secsRemaining-val :lambda-list '(m))
(cl:defmethod secsRemaining-val ((m <gameControl>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader decision-msg:secsRemaining-val is deprecated.  Use decision-msg:secsRemaining instead.")
  (secsRemaining m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <gameControl>) ostream)
  "Serializes a message object of type '<gameControl>"
  (cl:let* ((signed (cl:slot-value msg 'state)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'secsRemaining)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <gameControl>) istream)
  "Deserializes a message object of type '<gameControl>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'state) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'secsRemaining) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<gameControl>)))
  "Returns string type for a message object of type '<gameControl>"
  "decision/gameControl")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'gameControl)))
  "Returns string type for a message object of type 'gameControl"
  "decision/gameControl")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<gameControl>)))
  "Returns md5sum for a message object of type '<gameControl>"
  "4f8fd4e28d3da76570da7ab09212fb90")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'gameControl)))
  "Returns md5sum for a message object of type 'gameControl"
  "4f8fd4e28d3da76570da7ab09212fb90")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<gameControl>)))
  "Returns full string definition for message of type '<gameControl>"
  (cl:format cl:nil "int32 state~%int32 secsRemaining~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'gameControl)))
  "Returns full string definition for message of type 'gameControl"
  (cl:format cl:nil "int32 state~%int32 secsRemaining~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <gameControl>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <gameControl>))
  "Converts a ROS message object to a list"
  (cl:list 'gameControl
    (cl:cons ':state (state msg))
    (cl:cons ':secsRemaining (secsRemaining msg))
))
