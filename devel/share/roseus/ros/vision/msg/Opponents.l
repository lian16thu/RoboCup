;; Auto-generated. Do not edit!


(when (boundp 'vision::Opponents)
  (if (not (find-package "VISION"))
    (make-package "VISION"))
  (shadow 'Opponents (find-package "VISION")))
(unless (find-package "VISION::OPPONENTS")
  (make-package "VISION::OPPONENTS"))

(in-package "ROS")
;;//! \htmlinclude Opponents.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass vision::Opponents
  :super ros::object
  :slots (_header _opponent_detected _opponent_number _opponent_range _opponent_bearing ))

(defmethod vision::Opponents
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:opponent_detected __opponent_detected) nil)
    ((:opponent_number __opponent_number) 0)
    ((:opponent_range __opponent_range) (make-array 5 :initial-element 0.0 :element-type :float))
    ((:opponent_bearing __opponent_bearing) (make-array 5 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _header __header)
   (setq _opponent_detected __opponent_detected)
   (setq _opponent_number (round __opponent_number))
   (setq _opponent_range __opponent_range)
   (setq _opponent_bearing __opponent_bearing)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:opponent_detected
   (&optional __opponent_detected)
   (if __opponent_detected (setq _opponent_detected __opponent_detected)) _opponent_detected)
  (:opponent_number
   (&optional __opponent_number)
   (if __opponent_number (setq _opponent_number __opponent_number)) _opponent_number)
  (:opponent_range
   (&optional __opponent_range)
   (if __opponent_range (setq _opponent_range __opponent_range)) _opponent_range)
  (:opponent_bearing
   (&optional __opponent_bearing)
   (if __opponent_bearing (setq _opponent_bearing __opponent_bearing)) _opponent_bearing)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; bool _opponent_detected
    1
    ;; uint8 _opponent_number
    1
    ;; float64[5] _opponent_range
    (* 8    5)
    ;; float64[5] _opponent_bearing
    (* 8    5)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; bool _opponent_detected
       (if _opponent_detected (write-byte -1 s) (write-byte 0 s))
     ;; uint8 _opponent_number
       (write-byte _opponent_number s)
     ;; float64[5] _opponent_range
     (dotimes (i 5)
       (sys::poke (elt _opponent_range i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[5] _opponent_bearing
     (dotimes (i 5)
       (sys::poke (elt _opponent_bearing i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; bool _opponent_detected
     (setq _opponent_detected (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; uint8 _opponent_number
     (setq _opponent_number (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; float64[5] _opponent_range
   (dotimes (i (length _opponent_range))
     (setf (elt _opponent_range i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[5] _opponent_bearing
   (dotimes (i (length _opponent_bearing))
     (setf (elt _opponent_bearing i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;;
   self)
  )

(setf (get vision::Opponents :md5sum-) "ef41a5e204617f0c2b713c70ec7cf155")
(setf (get vision::Opponents :datatype-) "vision/Opponents")
(setf (get vision::Opponents :definition-)
      "Header header
bool opponent_detected
uint8 opponent_number
float64[5] opponent_range
float64[5] opponent_bearing

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



(provide :vision/Opponents "ef41a5e204617f0c2b713c70ec7cf155")


