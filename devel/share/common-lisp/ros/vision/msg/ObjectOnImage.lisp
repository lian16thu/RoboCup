; Auto-generated. Do not edit!


(cl:in-package vision-msg)


;//! \htmlinclude ObjectOnImage.msg.html

(cl:defclass <ObjectOnImage> (roslisp-msg-protocol:ros-message)
  ((object_type
    :reader object_type
    :initarg :object_type
    :type cl:fixnum
    :initform 0)
   (u
    :reader u
    :initarg :u
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 0 :element-type 'cl:fixnum :initial-element 0))
   (v
    :reader v
    :initarg :v
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 0 :element-type 'cl:fixnum :initial-element 0)))
)

(cl:defclass ObjectOnImage (<ObjectOnImage>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ObjectOnImage>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ObjectOnImage)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name vision-msg:<ObjectOnImage> is deprecated: use vision-msg:ObjectOnImage instead.")))

(cl:ensure-generic-function 'object_type-val :lambda-list '(m))
(cl:defmethod object_type-val ((m <ObjectOnImage>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vision-msg:object_type-val is deprecated.  Use vision-msg:object_type instead.")
  (object_type m))

(cl:ensure-generic-function 'u-val :lambda-list '(m))
(cl:defmethod u-val ((m <ObjectOnImage>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vision-msg:u-val is deprecated.  Use vision-msg:u instead.")
  (u m))

(cl:ensure-generic-function 'v-val :lambda-list '(m))
(cl:defmethod v-val ((m <ObjectOnImage>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vision-msg:v-val is deprecated.  Use vision-msg:v instead.")
  (v m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ObjectOnImage>) ostream)
  "Serializes a message object of type '<ObjectOnImage>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'object_type)) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'u))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream))
   (cl:slot-value msg 'u))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'v))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream))
   (cl:slot-value msg 'v))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ObjectOnImage>) istream)
  "Deserializes a message object of type '<ObjectOnImage>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'object_type)) (cl:read-byte istream))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'u) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'u)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream)))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'v) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'v)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ObjectOnImage>)))
  "Returns string type for a message object of type '<ObjectOnImage>"
  "vision/ObjectOnImage")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ObjectOnImage)))
  "Returns string type for a message object of type 'ObjectOnImage"
  "vision/ObjectOnImage")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ObjectOnImage>)))
  "Returns md5sum for a message object of type '<ObjectOnImage>"
  "d8b2eee0990ad6385c56c5984bb6dea1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ObjectOnImage)))
  "Returns md5sum for a message object of type 'ObjectOnImage"
  "d8b2eee0990ad6385c56c5984bb6dea1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ObjectOnImage>)))
  "Returns full string definition for message of type '<ObjectOnImage>"
  (cl:format cl:nil "uint8 object_type~%uint8[] u~%uint8[] v~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ObjectOnImage)))
  "Returns full string definition for message of type 'ObjectOnImage"
  (cl:format cl:nil "uint8 object_type~%uint8[] u~%uint8[] v~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ObjectOnImage>))
  (cl:+ 0
     1
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'u) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'v) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ObjectOnImage>))
  "Converts a ROS message object to a list"
  (cl:list 'ObjectOnImage
    (cl:cons ':object_type (object_type msg))
    (cl:cons ':u (u msg))
    (cl:cons ':v (v msg))
))
