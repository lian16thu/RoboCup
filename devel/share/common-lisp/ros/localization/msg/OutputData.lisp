; Auto-generated. Do not edit!


(cl:in-package localization-msg)


;//! \htmlinclude OutputData.msg.html

(cl:defclass <OutputData> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (robotPose
    :reader robotPose
    :initarg :robotPose
    :type geometry_msgs-msg:Pose2D
    :initform (cl:make-instance 'geometry_msgs-msg:Pose2D))
   (robotHeadPose
    :reader robotHeadPose
    :initarg :robotHeadPose
    :type geometry_msgs-msg:Pose2D
    :initform (cl:make-instance 'geometry_msgs-msg:Pose2D))
   (robotPoseConfidence
    :reader robotPoseConfidence
    :initarg :robotPoseConfidence
    :type cl:float
    :initform 0.0)
   (bBallWasSeen
    :reader bBallWasSeen
    :initarg :bBallWasSeen
    :type cl:boolean
    :initform cl:nil)
   (ballCenterInImage
    :reader ballCenterInImage
    :initarg :ballCenterInImage
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point))
   (ballDistance
    :reader ballDistance
    :initarg :ballDistance
    :type cl:float
    :initform 0.0)
   (ballAngle
    :reader ballAngle
    :initarg :ballAngle
    :type cl:float
    :initform 0.0)
   (ballCenterOnField
    :reader ballCenterOnField
    :initarg :ballCenterOnField
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point))
   (bOpponentWasSeen
    :reader bOpponentWasSeen
    :initarg :bOpponentWasSeen
    :type cl:boolean
    :initform cl:nil)
   (opponentLeftEndInImage
    :reader opponentLeftEndInImage
    :initarg :opponentLeftEndInImage
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point))
   (opponentRightEndInImage
    :reader opponentRightEndInImage
    :initarg :opponentRightEndInImage
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point))
   (opponentDistance
    :reader opponentDistance
    :initarg :opponentDistance
    :type cl:float
    :initform 0.0)
   (opponentAngle
    :reader opponentAngle
    :initarg :opponentAngle
    :type cl:float
    :initform 0.0)
   (opponentCenterOnField
    :reader opponentCenterOnField
    :initarg :opponentCenterOnField
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point))
   (opponentRadiusOnField
    :reader opponentRadiusOnField
    :initarg :opponentRadiusOnField
    :type cl:float
    :initform 0.0)
   (bObstacleWasSeen
    :reader bObstacleWasSeen
    :initarg :bObstacleWasSeen
    :type cl:boolean
    :initform cl:nil)
   (iObstacleNumber
    :reader iObstacleNumber
    :initarg :iObstacleNumber
    :type cl:integer
    :initform 0)
   (obstacleLeftEndInImage
    :reader obstacleLeftEndInImage
    :initarg :obstacleLeftEndInImage
    :type (cl:vector geometry_msgs-msg:Point)
   :initform (cl:make-array 5 :element-type 'geometry_msgs-msg:Point :initial-element (cl:make-instance 'geometry_msgs-msg:Point)))
   (obstacleRightEndInImage
    :reader obstacleRightEndInImage
    :initarg :obstacleRightEndInImage
    :type (cl:vector geometry_msgs-msg:Point)
   :initform (cl:make-array 5 :element-type 'geometry_msgs-msg:Point :initial-element (cl:make-instance 'geometry_msgs-msg:Point)))
   (obstacleDistance
    :reader obstacleDistance
    :initarg :obstacleDistance
    :type (cl:vector cl:float)
   :initform (cl:make-array 5 :element-type 'cl:float :initial-element 0.0))
   (obstacleAngle
    :reader obstacleAngle
    :initarg :obstacleAngle
    :type (cl:vector cl:float)
   :initform (cl:make-array 5 :element-type 'cl:float :initial-element 0.0))
   (obstacleCenterOnField
    :reader obstacleCenterOnField
    :initarg :obstacleCenterOnField
    :type (cl:vector geometry_msgs-msg:Point)
   :initform (cl:make-array 5 :element-type 'geometry_msgs-msg:Point :initial-element (cl:make-instance 'geometry_msgs-msg:Point)))
   (obstacleRadiusOnField
    :reader obstacleRadiusOnField
    :initarg :obstacleRadiusOnField
    :type (cl:vector cl:float)
   :initform (cl:make-array 5 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass OutputData (<OutputData>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <OutputData>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'OutputData)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name localization-msg:<OutputData> is deprecated: use localization-msg:OutputData instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <OutputData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader localization-msg:header-val is deprecated.  Use localization-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'robotPose-val :lambda-list '(m))
(cl:defmethod robotPose-val ((m <OutputData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader localization-msg:robotPose-val is deprecated.  Use localization-msg:robotPose instead.")
  (robotPose m))

(cl:ensure-generic-function 'robotHeadPose-val :lambda-list '(m))
(cl:defmethod robotHeadPose-val ((m <OutputData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader localization-msg:robotHeadPose-val is deprecated.  Use localization-msg:robotHeadPose instead.")
  (robotHeadPose m))

(cl:ensure-generic-function 'robotPoseConfidence-val :lambda-list '(m))
(cl:defmethod robotPoseConfidence-val ((m <OutputData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader localization-msg:robotPoseConfidence-val is deprecated.  Use localization-msg:robotPoseConfidence instead.")
  (robotPoseConfidence m))

(cl:ensure-generic-function 'bBallWasSeen-val :lambda-list '(m))
(cl:defmethod bBallWasSeen-val ((m <OutputData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader localization-msg:bBallWasSeen-val is deprecated.  Use localization-msg:bBallWasSeen instead.")
  (bBallWasSeen m))

(cl:ensure-generic-function 'ballCenterInImage-val :lambda-list '(m))
(cl:defmethod ballCenterInImage-val ((m <OutputData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader localization-msg:ballCenterInImage-val is deprecated.  Use localization-msg:ballCenterInImage instead.")
  (ballCenterInImage m))

(cl:ensure-generic-function 'ballDistance-val :lambda-list '(m))
(cl:defmethod ballDistance-val ((m <OutputData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader localization-msg:ballDistance-val is deprecated.  Use localization-msg:ballDistance instead.")
  (ballDistance m))

(cl:ensure-generic-function 'ballAngle-val :lambda-list '(m))
(cl:defmethod ballAngle-val ((m <OutputData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader localization-msg:ballAngle-val is deprecated.  Use localization-msg:ballAngle instead.")
  (ballAngle m))

(cl:ensure-generic-function 'ballCenterOnField-val :lambda-list '(m))
(cl:defmethod ballCenterOnField-val ((m <OutputData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader localization-msg:ballCenterOnField-val is deprecated.  Use localization-msg:ballCenterOnField instead.")
  (ballCenterOnField m))

(cl:ensure-generic-function 'bOpponentWasSeen-val :lambda-list '(m))
(cl:defmethod bOpponentWasSeen-val ((m <OutputData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader localization-msg:bOpponentWasSeen-val is deprecated.  Use localization-msg:bOpponentWasSeen instead.")
  (bOpponentWasSeen m))

(cl:ensure-generic-function 'opponentLeftEndInImage-val :lambda-list '(m))
(cl:defmethod opponentLeftEndInImage-val ((m <OutputData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader localization-msg:opponentLeftEndInImage-val is deprecated.  Use localization-msg:opponentLeftEndInImage instead.")
  (opponentLeftEndInImage m))

(cl:ensure-generic-function 'opponentRightEndInImage-val :lambda-list '(m))
(cl:defmethod opponentRightEndInImage-val ((m <OutputData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader localization-msg:opponentRightEndInImage-val is deprecated.  Use localization-msg:opponentRightEndInImage instead.")
  (opponentRightEndInImage m))

(cl:ensure-generic-function 'opponentDistance-val :lambda-list '(m))
(cl:defmethod opponentDistance-val ((m <OutputData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader localization-msg:opponentDistance-val is deprecated.  Use localization-msg:opponentDistance instead.")
  (opponentDistance m))

(cl:ensure-generic-function 'opponentAngle-val :lambda-list '(m))
(cl:defmethod opponentAngle-val ((m <OutputData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader localization-msg:opponentAngle-val is deprecated.  Use localization-msg:opponentAngle instead.")
  (opponentAngle m))

(cl:ensure-generic-function 'opponentCenterOnField-val :lambda-list '(m))
(cl:defmethod opponentCenterOnField-val ((m <OutputData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader localization-msg:opponentCenterOnField-val is deprecated.  Use localization-msg:opponentCenterOnField instead.")
  (opponentCenterOnField m))

(cl:ensure-generic-function 'opponentRadiusOnField-val :lambda-list '(m))
(cl:defmethod opponentRadiusOnField-val ((m <OutputData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader localization-msg:opponentRadiusOnField-val is deprecated.  Use localization-msg:opponentRadiusOnField instead.")
  (opponentRadiusOnField m))

(cl:ensure-generic-function 'bObstacleWasSeen-val :lambda-list '(m))
(cl:defmethod bObstacleWasSeen-val ((m <OutputData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader localization-msg:bObstacleWasSeen-val is deprecated.  Use localization-msg:bObstacleWasSeen instead.")
  (bObstacleWasSeen m))

(cl:ensure-generic-function 'iObstacleNumber-val :lambda-list '(m))
(cl:defmethod iObstacleNumber-val ((m <OutputData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader localization-msg:iObstacleNumber-val is deprecated.  Use localization-msg:iObstacleNumber instead.")
  (iObstacleNumber m))

(cl:ensure-generic-function 'obstacleLeftEndInImage-val :lambda-list '(m))
(cl:defmethod obstacleLeftEndInImage-val ((m <OutputData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader localization-msg:obstacleLeftEndInImage-val is deprecated.  Use localization-msg:obstacleLeftEndInImage instead.")
  (obstacleLeftEndInImage m))

(cl:ensure-generic-function 'obstacleRightEndInImage-val :lambda-list '(m))
(cl:defmethod obstacleRightEndInImage-val ((m <OutputData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader localization-msg:obstacleRightEndInImage-val is deprecated.  Use localization-msg:obstacleRightEndInImage instead.")
  (obstacleRightEndInImage m))

(cl:ensure-generic-function 'obstacleDistance-val :lambda-list '(m))
(cl:defmethod obstacleDistance-val ((m <OutputData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader localization-msg:obstacleDistance-val is deprecated.  Use localization-msg:obstacleDistance instead.")
  (obstacleDistance m))

(cl:ensure-generic-function 'obstacleAngle-val :lambda-list '(m))
(cl:defmethod obstacleAngle-val ((m <OutputData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader localization-msg:obstacleAngle-val is deprecated.  Use localization-msg:obstacleAngle instead.")
  (obstacleAngle m))

(cl:ensure-generic-function 'obstacleCenterOnField-val :lambda-list '(m))
(cl:defmethod obstacleCenterOnField-val ((m <OutputData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader localization-msg:obstacleCenterOnField-val is deprecated.  Use localization-msg:obstacleCenterOnField instead.")
  (obstacleCenterOnField m))

(cl:ensure-generic-function 'obstacleRadiusOnField-val :lambda-list '(m))
(cl:defmethod obstacleRadiusOnField-val ((m <OutputData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader localization-msg:obstacleRadiusOnField-val is deprecated.  Use localization-msg:obstacleRadiusOnField instead.")
  (obstacleRadiusOnField m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <OutputData>) ostream)
  "Serializes a message object of type '<OutputData>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'robotPose) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'robotHeadPose) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'robotPoseConfidence))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'bBallWasSeen) 1 0)) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'ballCenterInImage) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'ballDistance))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'ballAngle))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'ballCenterOnField) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'bOpponentWasSeen) 1 0)) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'opponentLeftEndInImage) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'opponentRightEndInImage) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'opponentDistance))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'opponentAngle))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'opponentCenterOnField) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'opponentRadiusOnField))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'bObstacleWasSeen) 1 0)) ostream)
  (cl:let* ((signed (cl:slot-value msg 'iObstacleNumber)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'obstacleLeftEndInImage))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'obstacleRightEndInImage))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'obstacleDistance))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'obstacleAngle))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'obstacleCenterOnField))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'obstacleRadiusOnField))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <OutputData>) istream)
  "Deserializes a message object of type '<OutputData>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'robotPose) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'robotHeadPose) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'robotPoseConfidence) (roslisp-utils:decode-single-float-bits bits)))
    (cl:setf (cl:slot-value msg 'bBallWasSeen) (cl:not (cl:zerop (cl:read-byte istream))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'ballCenterInImage) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'ballDistance) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'ballAngle) (roslisp-utils:decode-single-float-bits bits)))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'ballCenterOnField) istream)
    (cl:setf (cl:slot-value msg 'bOpponentWasSeen) (cl:not (cl:zerop (cl:read-byte istream))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'opponentLeftEndInImage) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'opponentRightEndInImage) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'opponentDistance) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'opponentAngle) (roslisp-utils:decode-single-float-bits bits)))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'opponentCenterOnField) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'opponentRadiusOnField) (roslisp-utils:decode-single-float-bits bits)))
    (cl:setf (cl:slot-value msg 'bObstacleWasSeen) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'iObstacleNumber) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  (cl:setf (cl:slot-value msg 'obstacleLeftEndInImage) (cl:make-array 5))
  (cl:let ((vals (cl:slot-value msg 'obstacleLeftEndInImage)))
    (cl:dotimes (i 5)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:Point))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream)))
  (cl:setf (cl:slot-value msg 'obstacleRightEndInImage) (cl:make-array 5))
  (cl:let ((vals (cl:slot-value msg 'obstacleRightEndInImage)))
    (cl:dotimes (i 5)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:Point))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream)))
  (cl:setf (cl:slot-value msg 'obstacleDistance) (cl:make-array 5))
  (cl:let ((vals (cl:slot-value msg 'obstacleDistance)))
    (cl:dotimes (i 5)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits)))))
  (cl:setf (cl:slot-value msg 'obstacleAngle) (cl:make-array 5))
  (cl:let ((vals (cl:slot-value msg 'obstacleAngle)))
    (cl:dotimes (i 5)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits)))))
  (cl:setf (cl:slot-value msg 'obstacleCenterOnField) (cl:make-array 5))
  (cl:let ((vals (cl:slot-value msg 'obstacleCenterOnField)))
    (cl:dotimes (i 5)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:Point))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream)))
  (cl:setf (cl:slot-value msg 'obstacleRadiusOnField) (cl:make-array 5))
  (cl:let ((vals (cl:slot-value msg 'obstacleRadiusOnField)))
    (cl:dotimes (i 5)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<OutputData>)))
  "Returns string type for a message object of type '<OutputData>"
  "localization/OutputData")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'OutputData)))
  "Returns string type for a message object of type 'OutputData"
  "localization/OutputData")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<OutputData>)))
  "Returns md5sum for a message object of type '<OutputData>"
  "0d5bb91e3132d7eb05a2ae36456b86d6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'OutputData)))
  "Returns md5sum for a message object of type 'OutputData"
  "0d5bb91e3132d7eb05a2ae36456b86d6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<OutputData>)))
  "Returns full string definition for message of type '<OutputData>"
  (cl:format cl:nil "std_msgs/Header header 				# for time stamp~%geometry_msgs/Pose2D robotPose        		# Pose of the robot according to the particle filter localization~%geometry_msgs/Pose2D robotHeadPose              # Pose of the robot head according to the particle filter localization~%float32 robotPoseConfidence               	# confidence 0..1 for the robot_pose~%bool bBallWasSeen                               # boolean variable for ball detection~%geometry_msgs/Point ballCenterInImage		# coordinates of ball center in the image~%float32 ballDistance				# ball distance from robot as seen in the image~%float32 ballAngle				# ball angle from center as seen in the image, [-90,90]~%geometry_msgs/Point ballCenterOnField		# coordinates of ball center on field relative to robot localization~%bool bOpponentWasSeen                           # boolean variable for opponent detection~%geometry_msgs/Point opponentLeftEndInImage	# coordinates of opponent left end point in the image~%geometry_msgs/Point opponentRightEndInImage	# coordinates of opponent right end point in the image~%float32 opponentDistance			# opponent distance from robot as seen in the image~%float32 opponentAngle				# opponent angle from center as seen in the image, [-90,90]~%geometry_msgs/Point opponentCenterOnField	# coordinates of opponent center on field relative to robot localization~%float32 opponentRadiusOnField			# estimated radius of opponent on field~%bool bObstacleWasSeen                           # boolean variable for obstacle detection~%int32 iObstacleNumber                           # index of obstacle~%geometry_msgs/Point[5] obstacleLeftEndInImage	# coordinates of obstacle left end point in the image~%geometry_msgs/Point[5] obstacleRightEndInImage	# coordinates of obstacle right end point in the image~%float32[5] obstacleDistance                     # obstacle distance from robot as seen in the image~%float32[5] obstacleAngle			# obstacle angle from center as seen in the image, [-90,90]~%geometry_msgs/Point[5] obstacleCenterOnField	# coordinates of obstacle center on field relative to robot localization~%float32[5] obstacleRadiusOnField		# estimated radius of obstacle on field~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose2D~%# Deprecated~%# Please use the full 3D pose.~%~%# In general our recommendation is to use a full 3D representation of everything and for 2D specific applications make the appropriate projections into the plane for their calculations but optimally will preserve the 3D information during processing.~%~%# If we have parallel copies of 2D datatypes every UI and other pipeline will end up needing to have dual interfaces to plot everything. And you will end up with not being able to use 3D tools for 2D use cases even if they're completely valid, as you'd have to reimplement it with different inputs and outputs. It's not particularly hard to plot the 2D pose or compute the yaw error for the Pose message and there are already tools and libraries that can do this for you.~%~%~%# This expresses a position and orientation on a 2D manifold.~%~%float64 x~%float64 y~%float64 theta~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'OutputData)))
  "Returns full string definition for message of type 'OutputData"
  (cl:format cl:nil "std_msgs/Header header 				# for time stamp~%geometry_msgs/Pose2D robotPose        		# Pose of the robot according to the particle filter localization~%geometry_msgs/Pose2D robotHeadPose              # Pose of the robot head according to the particle filter localization~%float32 robotPoseConfidence               	# confidence 0..1 for the robot_pose~%bool bBallWasSeen                               # boolean variable for ball detection~%geometry_msgs/Point ballCenterInImage		# coordinates of ball center in the image~%float32 ballDistance				# ball distance from robot as seen in the image~%float32 ballAngle				# ball angle from center as seen in the image, [-90,90]~%geometry_msgs/Point ballCenterOnField		# coordinates of ball center on field relative to robot localization~%bool bOpponentWasSeen                           # boolean variable for opponent detection~%geometry_msgs/Point opponentLeftEndInImage	# coordinates of opponent left end point in the image~%geometry_msgs/Point opponentRightEndInImage	# coordinates of opponent right end point in the image~%float32 opponentDistance			# opponent distance from robot as seen in the image~%float32 opponentAngle				# opponent angle from center as seen in the image, [-90,90]~%geometry_msgs/Point opponentCenterOnField	# coordinates of opponent center on field relative to robot localization~%float32 opponentRadiusOnField			# estimated radius of opponent on field~%bool bObstacleWasSeen                           # boolean variable for obstacle detection~%int32 iObstacleNumber                           # index of obstacle~%geometry_msgs/Point[5] obstacleLeftEndInImage	# coordinates of obstacle left end point in the image~%geometry_msgs/Point[5] obstacleRightEndInImage	# coordinates of obstacle right end point in the image~%float32[5] obstacleDistance                     # obstacle distance from robot as seen in the image~%float32[5] obstacleAngle			# obstacle angle from center as seen in the image, [-90,90]~%geometry_msgs/Point[5] obstacleCenterOnField	# coordinates of obstacle center on field relative to robot localization~%float32[5] obstacleRadiusOnField		# estimated radius of obstacle on field~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose2D~%# Deprecated~%# Please use the full 3D pose.~%~%# In general our recommendation is to use a full 3D representation of everything and for 2D specific applications make the appropriate projections into the plane for their calculations but optimally will preserve the 3D information during processing.~%~%# If we have parallel copies of 2D datatypes every UI and other pipeline will end up needing to have dual interfaces to plot everything. And you will end up with not being able to use 3D tools for 2D use cases even if they're completely valid, as you'd have to reimplement it with different inputs and outputs. It's not particularly hard to plot the 2D pose or compute the yaw error for the Pose message and there are already tools and libraries that can do this for you.~%~%~%# This expresses a position and orientation on a 2D manifold.~%~%float64 x~%float64 y~%float64 theta~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <OutputData>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'robotPose))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'robotHeadPose))
     4
     1
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'ballCenterInImage))
     4
     4
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'ballCenterOnField))
     1
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'opponentLeftEndInImage))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'opponentRightEndInImage))
     4
     4
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'opponentCenterOnField))
     4
     1
     4
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'obstacleLeftEndInImage) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'obstacleRightEndInImage) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'obstacleDistance) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'obstacleAngle) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'obstacleCenterOnField) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'obstacleRadiusOnField) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <OutputData>))
  "Converts a ROS message object to a list"
  (cl:list 'OutputData
    (cl:cons ':header (header msg))
    (cl:cons ':robotPose (robotPose msg))
    (cl:cons ':robotHeadPose (robotHeadPose msg))
    (cl:cons ':robotPoseConfidence (robotPoseConfidence msg))
    (cl:cons ':bBallWasSeen (bBallWasSeen msg))
    (cl:cons ':ballCenterInImage (ballCenterInImage msg))
    (cl:cons ':ballDistance (ballDistance msg))
    (cl:cons ':ballAngle (ballAngle msg))
    (cl:cons ':ballCenterOnField (ballCenterOnField msg))
    (cl:cons ':bOpponentWasSeen (bOpponentWasSeen msg))
    (cl:cons ':opponentLeftEndInImage (opponentLeftEndInImage msg))
    (cl:cons ':opponentRightEndInImage (opponentRightEndInImage msg))
    (cl:cons ':opponentDistance (opponentDistance msg))
    (cl:cons ':opponentAngle (opponentAngle msg))
    (cl:cons ':opponentCenterOnField (opponentCenterOnField msg))
    (cl:cons ':opponentRadiusOnField (opponentRadiusOnField msg))
    (cl:cons ':bObstacleWasSeen (bObstacleWasSeen msg))
    (cl:cons ':iObstacleNumber (iObstacleNumber msg))
    (cl:cons ':obstacleLeftEndInImage (obstacleLeftEndInImage msg))
    (cl:cons ':obstacleRightEndInImage (obstacleRightEndInImage msg))
    (cl:cons ':obstacleDistance (obstacleDistance msg))
    (cl:cons ':obstacleAngle (obstacleAngle msg))
    (cl:cons ':obstacleCenterOnField (obstacleCenterOnField msg))
    (cl:cons ':obstacleRadiusOnField (obstacleRadiusOnField msg))
))
