; Auto-generated. Do not edit!


(cl:in-package gamecontroller-msg)


;//! \htmlinclude gameControl.msg.html

(cl:defclass <gameControl> (roslisp-msg-protocol:ros-message)
  ((gameType
    :reader gameType
    :initarg :gameType
    :type cl:fixnum
    :initform 0)
   (state
    :reader state
    :initarg :state
    :type cl:fixnum
    :initform 0)
   (firstHalf
    :reader firstHalf
    :initarg :firstHalf
    :type cl:fixnum
    :initform 0)
   (kickOffTeam
    :reader kickOffTeam
    :initarg :kickOffTeam
    :type cl:fixnum
    :initform 0)
   (secondaryState
    :reader secondaryState
    :initarg :secondaryState
    :type cl:fixnum
    :initform 0)
   (secondaryStateTeam
    :reader secondaryStateTeam
    :initarg :secondaryStateTeam
    :type cl:fixnum
    :initform 0)
   (secondaryStateInfo
    :reader secondaryStateInfo
    :initarg :secondaryStateInfo
    :type cl:fixnum
    :initform 0)
   (dropInTeam
    :reader dropInTeam
    :initarg :dropInTeam
    :type cl:fixnum
    :initform 0)
   (dropInTime
    :reader dropInTime
    :initarg :dropInTime
    :type cl:fixnum
    :initform 0)
   (secsRemaining
    :reader secsRemaining
    :initarg :secsRemaining
    :type cl:fixnum
    :initform 0)
   (secondaryTime
    :reader secondaryTime
    :initarg :secondaryTime
    :type cl:fixnum
    :initform 0)
   (score
    :reader score
    :initarg :score
    :type cl:fixnum
    :initform 0)
   (penaltyShot
    :reader penaltyShot
    :initarg :penaltyShot
    :type cl:fixnum
    :initform 0)
   (singleShots
    :reader singleShots
    :initarg :singleShots
    :type cl:fixnum
    :initform 0)
   (penalty
    :reader penalty
    :initarg :penalty
    :type cl:fixnum
    :initform 0)
   (secsTillUnpenalised
    :reader secsTillUnpenalised
    :initarg :secsTillUnpenalised
    :type cl:fixnum
    :initform 0)
   (yellowCardCount
    :reader yellowCardCount
    :initarg :yellowCardCount
    :type cl:fixnum
    :initform 0)
   (redCardCount
    :reader redCardCount
    :initarg :redCardCount
    :type cl:fixnum
    :initform 0))
)

(cl:defclass gameControl (<gameControl>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <gameControl>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'gameControl)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name gamecontroller-msg:<gameControl> is deprecated: use gamecontroller-msg:gameControl instead.")))

(cl:ensure-generic-function 'gameType-val :lambda-list '(m))
(cl:defmethod gameType-val ((m <gameControl>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader gamecontroller-msg:gameType-val is deprecated.  Use gamecontroller-msg:gameType instead.")
  (gameType m))

(cl:ensure-generic-function 'state-val :lambda-list '(m))
(cl:defmethod state-val ((m <gameControl>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader gamecontroller-msg:state-val is deprecated.  Use gamecontroller-msg:state instead.")
  (state m))

(cl:ensure-generic-function 'firstHalf-val :lambda-list '(m))
(cl:defmethod firstHalf-val ((m <gameControl>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader gamecontroller-msg:firstHalf-val is deprecated.  Use gamecontroller-msg:firstHalf instead.")
  (firstHalf m))

(cl:ensure-generic-function 'kickOffTeam-val :lambda-list '(m))
(cl:defmethod kickOffTeam-val ((m <gameControl>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader gamecontroller-msg:kickOffTeam-val is deprecated.  Use gamecontroller-msg:kickOffTeam instead.")
  (kickOffTeam m))

(cl:ensure-generic-function 'secondaryState-val :lambda-list '(m))
(cl:defmethod secondaryState-val ((m <gameControl>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader gamecontroller-msg:secondaryState-val is deprecated.  Use gamecontroller-msg:secondaryState instead.")
  (secondaryState m))

(cl:ensure-generic-function 'secondaryStateTeam-val :lambda-list '(m))
(cl:defmethod secondaryStateTeam-val ((m <gameControl>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader gamecontroller-msg:secondaryStateTeam-val is deprecated.  Use gamecontroller-msg:secondaryStateTeam instead.")
  (secondaryStateTeam m))

(cl:ensure-generic-function 'secondaryStateInfo-val :lambda-list '(m))
(cl:defmethod secondaryStateInfo-val ((m <gameControl>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader gamecontroller-msg:secondaryStateInfo-val is deprecated.  Use gamecontroller-msg:secondaryStateInfo instead.")
  (secondaryStateInfo m))

(cl:ensure-generic-function 'dropInTeam-val :lambda-list '(m))
(cl:defmethod dropInTeam-val ((m <gameControl>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader gamecontroller-msg:dropInTeam-val is deprecated.  Use gamecontroller-msg:dropInTeam instead.")
  (dropInTeam m))

(cl:ensure-generic-function 'dropInTime-val :lambda-list '(m))
(cl:defmethod dropInTime-val ((m <gameControl>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader gamecontroller-msg:dropInTime-val is deprecated.  Use gamecontroller-msg:dropInTime instead.")
  (dropInTime m))

(cl:ensure-generic-function 'secsRemaining-val :lambda-list '(m))
(cl:defmethod secsRemaining-val ((m <gameControl>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader gamecontroller-msg:secsRemaining-val is deprecated.  Use gamecontroller-msg:secsRemaining instead.")
  (secsRemaining m))

(cl:ensure-generic-function 'secondaryTime-val :lambda-list '(m))
(cl:defmethod secondaryTime-val ((m <gameControl>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader gamecontroller-msg:secondaryTime-val is deprecated.  Use gamecontroller-msg:secondaryTime instead.")
  (secondaryTime m))

(cl:ensure-generic-function 'score-val :lambda-list '(m))
(cl:defmethod score-val ((m <gameControl>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader gamecontroller-msg:score-val is deprecated.  Use gamecontroller-msg:score instead.")
  (score m))

(cl:ensure-generic-function 'penaltyShot-val :lambda-list '(m))
(cl:defmethod penaltyShot-val ((m <gameControl>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader gamecontroller-msg:penaltyShot-val is deprecated.  Use gamecontroller-msg:penaltyShot instead.")
  (penaltyShot m))

(cl:ensure-generic-function 'singleShots-val :lambda-list '(m))
(cl:defmethod singleShots-val ((m <gameControl>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader gamecontroller-msg:singleShots-val is deprecated.  Use gamecontroller-msg:singleShots instead.")
  (singleShots m))

(cl:ensure-generic-function 'penalty-val :lambda-list '(m))
(cl:defmethod penalty-val ((m <gameControl>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader gamecontroller-msg:penalty-val is deprecated.  Use gamecontroller-msg:penalty instead.")
  (penalty m))

(cl:ensure-generic-function 'secsTillUnpenalised-val :lambda-list '(m))
(cl:defmethod secsTillUnpenalised-val ((m <gameControl>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader gamecontroller-msg:secsTillUnpenalised-val is deprecated.  Use gamecontroller-msg:secsTillUnpenalised instead.")
  (secsTillUnpenalised m))

(cl:ensure-generic-function 'yellowCardCount-val :lambda-list '(m))
(cl:defmethod yellowCardCount-val ((m <gameControl>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader gamecontroller-msg:yellowCardCount-val is deprecated.  Use gamecontroller-msg:yellowCardCount instead.")
  (yellowCardCount m))

(cl:ensure-generic-function 'redCardCount-val :lambda-list '(m))
(cl:defmethod redCardCount-val ((m <gameControl>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader gamecontroller-msg:redCardCount-val is deprecated.  Use gamecontroller-msg:redCardCount instead.")
  (redCardCount m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <gameControl>) ostream)
  "Serializes a message object of type '<gameControl>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'gameType)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'state)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'firstHalf)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'kickOffTeam)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'secondaryState)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'secondaryStateTeam)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'secondaryStateInfo)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'dropInTeam)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'dropInTime)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'dropInTime)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'secsRemaining)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'secsRemaining)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'secondaryTime)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'secondaryTime)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'score)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'penaltyShot)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'singleShots)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'singleShots)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'penalty)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'secsTillUnpenalised)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'yellowCardCount)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'redCardCount)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <gameControl>) istream)
  "Deserializes a message object of type '<gameControl>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'gameType)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'state)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'firstHalf)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'kickOffTeam)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'secondaryState)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'secondaryStateTeam)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'secondaryStateInfo)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'dropInTeam)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'dropInTime)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'dropInTime)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'secsRemaining)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'secsRemaining)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'secondaryTime)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'secondaryTime)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'score)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'penaltyShot)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'singleShots)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'singleShots)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'penalty)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'secsTillUnpenalised)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'yellowCardCount)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'redCardCount)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<gameControl>)))
  "Returns string type for a message object of type '<gameControl>"
  "gamecontroller/gameControl")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'gameControl)))
  "Returns string type for a message object of type 'gameControl"
  "gamecontroller/gameControl")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<gameControl>)))
  "Returns md5sum for a message object of type '<gameControl>"
  "8187bc9e4bf6fa1896498e321b213f47")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'gameControl)))
  "Returns md5sum for a message object of type 'gameControl"
  "8187bc9e4bf6fa1896498e321b213f47")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<gameControl>)))
  "Returns full string definition for message of type '<gameControl>"
  (cl:format cl:nil "uint8 gameType~%uint8 state~%uint8 firstHalf~%uint8 kickOffTeam~%uint8 secondaryState~%uint8 secondaryStateTeam~%uint8 secondaryStateInfo~%uint8 dropInTeam~%uint16 dropInTime~%uint16 secsRemaining~%uint16 secondaryTime~%~%uint8 score~%uint8 penaltyShot~%uint16 singleShots~%~%uint8 penalty~%uint8 secsTillUnpenalised~%uint8 yellowCardCount~%uint8 redCardCount~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'gameControl)))
  "Returns full string definition for message of type 'gameControl"
  (cl:format cl:nil "uint8 gameType~%uint8 state~%uint8 firstHalf~%uint8 kickOffTeam~%uint8 secondaryState~%uint8 secondaryStateTeam~%uint8 secondaryStateInfo~%uint8 dropInTeam~%uint16 dropInTime~%uint16 secsRemaining~%uint16 secondaryTime~%~%uint8 score~%uint8 penaltyShot~%uint16 singleShots~%~%uint8 penalty~%uint8 secsTillUnpenalised~%uint8 yellowCardCount~%uint8 redCardCount~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <gameControl>))
  (cl:+ 0
     1
     1
     1
     1
     1
     1
     1
     1
     2
     2
     2
     1
     1
     2
     1
     1
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <gameControl>))
  "Converts a ROS message object to a list"
  (cl:list 'gameControl
    (cl:cons ':gameType (gameType msg))
    (cl:cons ':state (state msg))
    (cl:cons ':firstHalf (firstHalf msg))
    (cl:cons ':kickOffTeam (kickOffTeam msg))
    (cl:cons ':secondaryState (secondaryState msg))
    (cl:cons ':secondaryStateTeam (secondaryStateTeam msg))
    (cl:cons ':secondaryStateInfo (secondaryStateInfo msg))
    (cl:cons ':dropInTeam (dropInTeam msg))
    (cl:cons ':dropInTime (dropInTime msg))
    (cl:cons ':secsRemaining (secsRemaining msg))
    (cl:cons ':secondaryTime (secondaryTime msg))
    (cl:cons ':score (score msg))
    (cl:cons ':penaltyShot (penaltyShot msg))
    (cl:cons ':singleShots (singleShots msg))
    (cl:cons ':penalty (penalty msg))
    (cl:cons ':secsTillUnpenalised (secsTillUnpenalised msg))
    (cl:cons ':yellowCardCount (yellowCardCount msg))
    (cl:cons ':redCardCount (redCardCount msg))
))
