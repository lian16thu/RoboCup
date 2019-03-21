;; Auto-generated. Do not edit!


(when (boundp 'decision::UDPReceived)
  (if (not (find-package "DECISION"))
    (make-package "DECISION"))
  (shadow 'UDPReceived (find-package "DECISION")))
(unless (find-package "DECISION::UDPRECEIVED")
  (make-package "DECISION::UDPRECEIVED"))

(in-package "ROS")
;;//! \htmlinclude UDPReceived.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass decision::UDPReceived
  :super ros::object
  :slots (_header _received_data ))

(defmethod decision::UDPReceived
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:received_data __received_data) (make-array 10 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _header __header)
   (setq _received_data __received_data)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:received_data
   (&optional __received_data)
   (if __received_data (setq _received_data __received_data)) _received_data)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float32[10] _received_data
    (* 4    10)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float32[10] _received_data
     (dotimes (i 10)
       (sys::poke (elt _received_data i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float32[10] _received_data
   (dotimes (i (length _received_data))
     (setf (elt _received_data i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     )
   ;;
   self)
  )

(setf (get decision::UDPReceived :md5sum-) "6946b75416098a1dfb2a52c2eb0f84b3")
(setf (get decision::UDPReceived :datatype-) "decision/UDPReceived")
(setf (get decision::UDPReceived :definition-)
      "std_msgs/Header header
float32[10] received_data

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
# 0: no frame
# 1: global frame
string frame_id

")



(provide :decision/UDPReceived "6946b75416098a1dfb2a52c2eb0f84b3")


