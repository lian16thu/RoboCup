; Auto-generated. Do not edit!


(cl:in-package vision-msg)


;//! \htmlinclude Obstacle.msg.html

(cl:defclass <Obstacle> (roslisp-msg-protocol:ros-message)
  ((bObstacleWasSeen
    :reader bObstacleWasSeen
    :initarg :bObstacleWasSeen
    :type cl:fixnum
    :initform 0)
   (iLeftEdgeInImageX
    :reader iLeftEdgeInImageX
    :initarg :iLeftEdgeInImageX
    :type cl:fixnum
    :initform 0)
   (iLeftEdgeInImageY
    :reader iLeftEdgeInImageY
    :initarg :iLeftEdgeInImageY
    :type cl:fixnum
    :initform 0)
   (iRightEdgeInImageX
    :reader iRightEdgeInImageX
    :initarg :iRightEdgeInImageX
    :type cl:fixnum
    :initform 0)
   (iRightEdgeInImageY
    :reader iRightEdgeInImageY
    :initarg :iRightEdgeInImageY
    :type cl:fixnum
    :initform 0)
   (iHeightInImage
    :reader iHeightInImage
    :initarg :iHeightInImage
    :type cl:fixnum
    :initform 0)
   (inumber
    :reader inumber
    :initarg :inumber
    :type cl:fixnum
    :initform 0)
   (iOthersLeftEdgeInImageX
    :reader iOthersLeftEdgeInImageX
    :initarg :iOthersLeftEdgeInImageX
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 0 :element-type 'cl:fixnum :initial-element 0))
   (iOthersRightEdgeInImageX
    :reader iOthersRightEdgeInImageX
    :initarg :iOthersRightEdgeInImageX
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 0 :element-type 'cl:fixnum :initial-element 0))
   (iOthersInImageY
    :reader iOthersInImageY
    :initarg :iOthersInImageY
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 0 :element-type 'cl:fixnum :initial-element 0))
   (iOthersHeightInImage
    :reader iOthersHeightInImage
    :initarg :iOthersHeightInImage
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 0 :element-type 'cl:fixnum :initial-element 0)))
)

(cl:defclass Obstacle (<Obstacle>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Obstacle>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Obstacle)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name vision-msg:<Obstacle> is deprecated: use vision-msg:Obstacle instead.")))

(cl:ensure-generic-function 'bObstacleWasSeen-val :lambda-list '(m))
(cl:defmethod bObstacleWasSeen-val ((m <Obstacle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vision-msg:bObstacleWasSeen-val is deprecated.  Use vision-msg:bObstacleWasSeen instead.")
  (bObstacleWasSeen m))

(cl:ensure-generic-function 'iLeftEdgeInImageX-val :lambda-list '(m))
(cl:defmethod iLeftEdgeInImageX-val ((m <Obstacle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vision-msg:iLeftEdgeInImageX-val is deprecated.  Use vision-msg:iLeftEdgeInImageX instead.")
  (iLeftEdgeInImageX m))

(cl:ensure-generic-function 'iLeftEdgeInImageY-val :lambda-list '(m))
(cl:defmethod iLeftEdgeInImageY-val ((m <Obstacle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vision-msg:iLeftEdgeInImageY-val is deprecated.  Use vision-msg:iLeftEdgeInImageY instead.")
  (iLeftEdgeInImageY m))

(cl:ensure-generic-function 'iRightEdgeInImageX-val :lambda-list '(m))
(cl:defmethod iRightEdgeInImageX-val ((m <Obstacle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vision-msg:iRightEdgeInImageX-val is deprecated.  Use vision-msg:iRightEdgeInImageX instead.")
  (iRightEdgeInImageX m))

(cl:ensure-generic-function 'iRightEdgeInImageY-val :lambda-list '(m))
(cl:defmethod iRightEdgeInImageY-val ((m <Obstacle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vision-msg:iRightEdgeInImageY-val is deprecated.  Use vision-msg:iRightEdgeInImageY instead.")
  (iRightEdgeInImageY m))

(cl:ensure-generic-function 'iHeightInImage-val :lambda-list '(m))
(cl:defmethod iHeightInImage-val ((m <Obstacle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vision-msg:iHeightInImage-val is deprecated.  Use vision-msg:iHeightInImage instead.")
  (iHeightInImage m))

(cl:ensure-generic-function 'inumber-val :lambda-list '(m))
(cl:defmethod inumber-val ((m <Obstacle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vision-msg:inumber-val is deprecated.  Use vision-msg:inumber instead.")
  (inumber m))

(cl:ensure-generic-function 'iOthersLeftEdgeInImageX-val :lambda-list '(m))
(cl:defmethod iOthersLeftEdgeInImageX-val ((m <Obstacle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vision-msg:iOthersLeftEdgeInImageX-val is deprecated.  Use vision-msg:iOthersLeftEdgeInImageX instead.")
  (iOthersLeftEdgeInImageX m))

(cl:ensure-generic-function 'iOthersRightEdgeInImageX-val :lambda-list '(m))
(cl:defmethod iOthersRightEdgeInImageX-val ((m <Obstacle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vision-msg:iOthersRightEdgeInImageX-val is deprecated.  Use vision-msg:iOthersRightEdgeInImageX instead.")
  (iOthersRightEdgeInImageX m))

(cl:ensure-generic-function 'iOthersInImageY-val :lambda-list '(m))
(cl:defmethod iOthersInImageY-val ((m <Obstacle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vision-msg:iOthersInImageY-val is deprecated.  Use vision-msg:iOthersInImageY instead.")
  (iOthersInImageY m))

(cl:ensure-generic-function 'iOthersHeightInImage-val :lambda-list '(m))
(cl:defmethod iOthersHeightInImage-val ((m <Obstacle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vision-msg:iOthersHeightInImage-val is deprecated.  Use vision-msg:iOthersHeightInImage instead.")
  (iOthersHeightInImage m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Obstacle>) ostream)
  "Serializes a message object of type '<Obstacle>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'bObstacleWasSeen)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'iLeftEdgeInImageX)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'iLeftEdgeInImageX)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'iLeftEdgeInImageY)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'iLeftEdgeInImageY)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'iRightEdgeInImageX)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'iRightEdgeInImageX)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'iRightEdgeInImageY)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'iRightEdgeInImageY)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'iHeightInImage)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'iHeightInImage)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'inumber)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'inumber)) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'iOthersLeftEdgeInImageX))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) ele) ostream))
   (cl:slot-value msg 'iOthersLeftEdgeInImageX))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'iOthersRightEdgeInImageX))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) ele) ostream))
   (cl:slot-value msg 'iOthersRightEdgeInImageX))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'iOthersInImageY))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) ele) ostream))
   (cl:slot-value msg 'iOthersInImageY))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'iOthersHeightInImage))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) ele) ostream))
   (cl:slot-value msg 'iOthersHeightInImage))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Obstacle>) istream)
  "Deserializes a message object of type '<Obstacle>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'bObstacleWasSeen)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'iLeftEdgeInImageX)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'iLeftEdgeInImageX)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'iLeftEdgeInImageY)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'iLeftEdgeInImageY)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'iRightEdgeInImageX)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'iRightEdgeInImageX)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'iRightEdgeInImageY)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'iRightEdgeInImageY)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'iHeightInImage)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'iHeightInImage)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'inumber)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'inumber)) (cl:read-byte istream))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'iOthersLeftEdgeInImageX) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'iOthersLeftEdgeInImageX)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:aref vals i)) (cl:read-byte istream)))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'iOthersRightEdgeInImageX) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'iOthersRightEdgeInImageX)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:aref vals i)) (cl:read-byte istream)))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'iOthersInImageY) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'iOthersInImageY)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:aref vals i)) (cl:read-byte istream)))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'iOthersHeightInImage) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'iOthersHeightInImage)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:aref vals i)) (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Obstacle>)))
  "Returns string type for a message object of type '<Obstacle>"
  "vision/Obstacle")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Obstacle)))
  "Returns string type for a message object of type 'Obstacle"
  "vision/Obstacle")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Obstacle>)))
  "Returns md5sum for a message object of type '<Obstacle>"
  "b7ab1788ead7712018d8f4217af3a9ee")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Obstacle)))
  "Returns md5sum for a message object of type 'Obstacle"
  "b7ab1788ead7712018d8f4217af3a9ee")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Obstacle>)))
  "Returns full string definition for message of type '<Obstacle>"
  (cl:format cl:nil "uint8 bObstacleWasSeen~%~%uint16 iLeftEdgeInImageX~%uint16 iLeftEdgeInImageY~%uint16 iRightEdgeInImageX~%uint16 iRightEdgeInImageY~%uint16 iHeightInImage~%~%uint16 inumber # apart from first obstacle, if there were 2 obstacles found, this is 1~%uint16[] iOthersLeftEdgeInImageX~%uint16[] iOthersRightEdgeInImageX~%uint16[] iOthersInImageY  # the same Y for both left and right~%uint16[] iOthersHeightInImage~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Obstacle)))
  "Returns full string definition for message of type 'Obstacle"
  (cl:format cl:nil "uint8 bObstacleWasSeen~%~%uint16 iLeftEdgeInImageX~%uint16 iLeftEdgeInImageY~%uint16 iRightEdgeInImageX~%uint16 iRightEdgeInImageY~%uint16 iHeightInImage~%~%uint16 inumber # apart from first obstacle, if there were 2 obstacles found, this is 1~%uint16[] iOthersLeftEdgeInImageX~%uint16[] iOthersRightEdgeInImageX~%uint16[] iOthersInImageY  # the same Y for both left and right~%uint16[] iOthersHeightInImage~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Obstacle>))
  (cl:+ 0
     1
     2
     2
     2
     2
     2
     2
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'iOthersLeftEdgeInImageX) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 2)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'iOthersRightEdgeInImageX) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 2)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'iOthersInImageY) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 2)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'iOthersHeightInImage) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 2)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Obstacle>))
  "Converts a ROS message object to a list"
  (cl:list 'Obstacle
    (cl:cons ':bObstacleWasSeen (bObstacleWasSeen msg))
    (cl:cons ':iLeftEdgeInImageX (iLeftEdgeInImageX msg))
    (cl:cons ':iLeftEdgeInImageY (iLeftEdgeInImageY msg))
    (cl:cons ':iRightEdgeInImageX (iRightEdgeInImageX msg))
    (cl:cons ':iRightEdgeInImageY (iRightEdgeInImageY msg))
    (cl:cons ':iHeightInImage (iHeightInImage msg))
    (cl:cons ':inumber (inumber msg))
    (cl:cons ':iOthersLeftEdgeInImageX (iOthersLeftEdgeInImageX msg))
    (cl:cons ':iOthersRightEdgeInImageX (iOthersRightEdgeInImageX msg))
    (cl:cons ':iOthersInImageY (iOthersInImageY msg))
    (cl:cons ':iOthersHeightInImage (iOthersHeightInImage msg))
))
