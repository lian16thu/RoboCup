; Auto-generated. Do not edit!


(cl:in-package decision-srv)


;//! \htmlinclude Pathplaning_the-request.msg.html

(cl:defclass <Pathplaning_the-request> (roslisp-msg-protocol:ros-message)
  ((state
    :reader state
    :initarg :state
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass Pathplaning_the-request (<Pathplaning_the-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Pathplaning_the-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Pathplaning_the-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name decision-srv:<Pathplaning_the-request> is deprecated: use decision-srv:Pathplaning_the-request instead.")))

(cl:ensure-generic-function 'state-val :lambda-list '(m))
(cl:defmethod state-val ((m <Pathplaning_the-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader decision-srv:state-val is deprecated.  Use decision-srv:state instead.")
  (state m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Pathplaning_the-request>) ostream)
  "Serializes a message object of type '<Pathplaning_the-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'state) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Pathplaning_the-request>) istream)
  "Deserializes a message object of type '<Pathplaning_the-request>"
    (cl:setf (cl:slot-value msg 'state) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Pathplaning_the-request>)))
  "Returns string type for a service object of type '<Pathplaning_the-request>"
  "decision/Pathplaning_theRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Pathplaning_the-request)))
  "Returns string type for a service object of type 'Pathplaning_the-request"
  "decision/Pathplaning_theRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Pathplaning_the-request>)))
  "Returns md5sum for a message object of type '<Pathplaning_the-request>"
  "743721b9c6ac4f45f0566da711cade92")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Pathplaning_the-request)))
  "Returns md5sum for a message object of type 'Pathplaning_the-request"
  "743721b9c6ac4f45f0566da711cade92")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Pathplaning_the-request>)))
  "Returns full string definition for message of type '<Pathplaning_the-request>"
  (cl:format cl:nil "bool state~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Pathplaning_the-request)))
  "Returns full string definition for message of type 'Pathplaning_the-request"
  (cl:format cl:nil "bool state~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Pathplaning_the-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Pathplaning_the-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Pathplaning_the-request
    (cl:cons ':state (state msg))
))
;//! \htmlinclude Pathplaning_the-response.msg.html

(cl:defclass <Pathplaning_the-response> (roslisp-msg-protocol:ros-message)
  ((theta_result
    :reader theta_result
    :initarg :theta_result
    :type cl:float
    :initform 0.0))
)

(cl:defclass Pathplaning_the-response (<Pathplaning_the-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Pathplaning_the-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Pathplaning_the-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name decision-srv:<Pathplaning_the-response> is deprecated: use decision-srv:Pathplaning_the-response instead.")))

(cl:ensure-generic-function 'theta_result-val :lambda-list '(m))
(cl:defmethod theta_result-val ((m <Pathplaning_the-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader decision-srv:theta_result-val is deprecated.  Use decision-srv:theta_result instead.")
  (theta_result m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Pathplaning_the-response>) ostream)
  "Serializes a message object of type '<Pathplaning_the-response>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'theta_result))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Pathplaning_the-response>) istream)
  "Deserializes a message object of type '<Pathplaning_the-response>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'theta_result) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Pathplaning_the-response>)))
  "Returns string type for a service object of type '<Pathplaning_the-response>"
  "decision/Pathplaning_theResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Pathplaning_the-response)))
  "Returns string type for a service object of type 'Pathplaning_the-response"
  "decision/Pathplaning_theResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Pathplaning_the-response>)))
  "Returns md5sum for a message object of type '<Pathplaning_the-response>"
  "743721b9c6ac4f45f0566da711cade92")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Pathplaning_the-response)))
  "Returns md5sum for a message object of type 'Pathplaning_the-response"
  "743721b9c6ac4f45f0566da711cade92")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Pathplaning_the-response>)))
  "Returns full string definition for message of type '<Pathplaning_the-response>"
  (cl:format cl:nil "float32 theta_result~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Pathplaning_the-response)))
  "Returns full string definition for message of type 'Pathplaning_the-response"
  (cl:format cl:nil "float32 theta_result~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Pathplaning_the-response>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Pathplaning_the-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Pathplaning_the-response
    (cl:cons ':theta_result (theta_result msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Pathplaning_the)))
  'Pathplaning_the-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Pathplaning_the)))
  'Pathplaning_the-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Pathplaning_the)))
  "Returns string type for a service object of type '<Pathplaning_the>"
  "decision/Pathplaning_the")