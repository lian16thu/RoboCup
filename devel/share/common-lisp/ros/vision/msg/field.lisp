; Auto-generated. Do not edit!


(cl:in-package vision-msg)


;//! \htmlinclude field.msg.html

(cl:defclass <field> (roslisp-msg-protocol:ros-message)
  ((width
    :reader width
    :initarg :width
    :type cl:fixnum
    :initform 0)
   (data
    :reader data
    :initarg :data
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 0 :element-type 'cl:fixnum :initial-element 0)))
)

(cl:defclass field (<field>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <field>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'field)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name vision-msg:<field> is deprecated: use vision-msg:field instead.")))

(cl:ensure-generic-function 'width-val :lambda-list '(m))
(cl:defmethod width-val ((m <field>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vision-msg:width-val is deprecated.  Use vision-msg:width instead.")
  (width m))

(cl:ensure-generic-function 'data-val :lambda-list '(m))
(cl:defmethod data-val ((m <field>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vision-msg:data-val is deprecated.  Use vision-msg:data instead.")
  (data m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <field>) ostream)
  "Serializes a message object of type '<field>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'width)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'width)) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'data))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) ele) ostream))
   (cl:slot-value msg 'data))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <field>) istream)
  "Deserializes a message object of type '<field>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'width)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'width)) (cl:read-byte istream))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'data) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'data)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:aref vals i)) (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<field>)))
  "Returns string type for a message object of type '<field>"
  "vision/field")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'field)))
  "Returns string type for a message object of type 'field"
  "vision/field")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<field>)))
  "Returns md5sum for a message object of type '<field>"
  "e7238b8238eb8992ed07002d15ada745")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'field)))
  "Returns md5sum for a message object of type 'field"
  "e7238b8238eb8992ed07002d15ada745")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<field>)))
  "Returns full string definition for message of type '<field>"
  (cl:format cl:nil "uint16 width~%uint16[] data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'field)))
  "Returns full string definition for message of type 'field"
  (cl:format cl:nil "uint16 width~%uint16[] data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <field>))
  (cl:+ 0
     2
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'data) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 2)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <field>))
  "Converts a ROS message object to a list"
  (cl:list 'field
    (cl:cons ':width (width msg))
    (cl:cons ':data (data msg))
))
