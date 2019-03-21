; Auto-generated. Do not edit!


(cl:in-package decision-msg)


;//! \htmlinclude Ball.msg.html

(cl:defclass <Ball> (roslisp-msg-protocol:ros-message)
  ((bBallWasSeen
    :reader bBallWasSeen
    :initarg :bBallWasSeen
    :type cl:fixnum
    :initform 0)
   (iCenterInImageX
    :reader iCenterInImageX
    :initarg :iCenterInImageX
    :type cl:fixnum
    :initform 0)
   (iCenterInImageY
    :reader iCenterInImageY
    :initarg :iCenterInImageY
    :type cl:fixnum
    :initform 0)
   (iRadiusInImage
    :reader iRadiusInImage
    :initarg :iRadiusInImage
    :type cl:fixnum
    :initform 0)
   (fDistance
    :reader fDistance
    :initarg :fDistance
    :type cl:float
    :initform 0.0)
   (fAngle
    :reader fAngle
    :initarg :fAngle
    :type cl:float
    :initform 0.0))
)

(cl:defclass Ball (<Ball>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Ball>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Ball)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name decision-msg:<Ball> is deprecated: use decision-msg:Ball instead.")))

(cl:ensure-generic-function 'bBallWasSeen-val :lambda-list '(m))
(cl:defmethod bBallWasSeen-val ((m <Ball>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader decision-msg:bBallWasSeen-val is deprecated.  Use decision-msg:bBallWasSeen instead.")
  (bBallWasSeen m))

(cl:ensure-generic-function 'iCenterInImageX-val :lambda-list '(m))
(cl:defmethod iCenterInImageX-val ((m <Ball>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader decision-msg:iCenterInImageX-val is deprecated.  Use decision-msg:iCenterInImageX instead.")
  (iCenterInImageX m))

(cl:ensure-generic-function 'iCenterInImageY-val :lambda-list '(m))
(cl:defmethod iCenterInImageY-val ((m <Ball>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader decision-msg:iCenterInImageY-val is deprecated.  Use decision-msg:iCenterInImageY instead.")
  (iCenterInImageY m))

(cl:ensure-generic-function 'iRadiusInImage-val :lambda-list '(m))
(cl:defmethod iRadiusInImage-val ((m <Ball>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader decision-msg:iRadiusInImage-val is deprecated.  Use decision-msg:iRadiusInImage instead.")
  (iRadiusInImage m))

(cl:ensure-generic-function 'fDistance-val :lambda-list '(m))
(cl:defmethod fDistance-val ((m <Ball>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader decision-msg:fDistance-val is deprecated.  Use decision-msg:fDistance instead.")
  (fDistance m))

(cl:ensure-generic-function 'fAngle-val :lambda-list '(m))
(cl:defmethod fAngle-val ((m <Ball>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader decision-msg:fAngle-val is deprecated.  Use decision-msg:fAngle instead.")
  (fAngle m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Ball>) ostream)
  "Serializes a message object of type '<Ball>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'bBallWasSeen)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'iCenterInImageX)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'iCenterInImageX)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'iCenterInImageY)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'iCenterInImageY)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'iRadiusInImage)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'iRadiusInImage)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'fDistance))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'fAngle))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Ball>) istream)
  "Deserializes a message object of type '<Ball>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'bBallWasSeen)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'iCenterInImageX)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'iCenterInImageX)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'iCenterInImageY)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'iCenterInImageY)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'iRadiusInImage)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'iRadiusInImage)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'fDistance) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'fAngle) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Ball>)))
  "Returns string type for a message object of type '<Ball>"
  "decision/Ball")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Ball)))
  "Returns string type for a message object of type 'Ball"
  "decision/Ball")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Ball>)))
  "Returns md5sum for a message object of type '<Ball>"
  "b1776e77d61a18b8d498a831f6c9807a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Ball)))
  "Returns md5sum for a message object of type 'Ball"
  "b1776e77d61a18b8d498a831f6c9807a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Ball>)))
  "Returns full string definition for message of type '<Ball>"
  (cl:format cl:nil "uint8 bBallWasSeen~%uint16 iCenterInImageX~%uint16 iCenterInImageY~%uint16 iRadiusInImage~%float32 fDistance~%float32 fAngle~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Ball)))
  "Returns full string definition for message of type 'Ball"
  (cl:format cl:nil "uint8 bBallWasSeen~%uint16 iCenterInImageX~%uint16 iCenterInImageY~%uint16 iRadiusInImage~%float32 fDistance~%float32 fAngle~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Ball>))
  (cl:+ 0
     1
     2
     2
     2
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Ball>))
  "Converts a ROS message object to a list"
  (cl:list 'Ball
    (cl:cons ':bBallWasSeen (bBallWasSeen msg))
    (cl:cons ':iCenterInImageX (iCenterInImageX msg))
    (cl:cons ':iCenterInImageY (iCenterInImageY msg))
    (cl:cons ':iRadiusInImage (iRadiusInImage msg))
    (cl:cons ':fDistance (fDistance msg))
    (cl:cons ':fAngle (fAngle msg))
))
