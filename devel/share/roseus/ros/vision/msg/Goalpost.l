;; Auto-generated. Do not edit!


(when (boundp 'vision::Goalpost)
  (if (not (find-package "VISION"))
    (make-package "VISION"))
  (shadow 'Goalpost (find-package "VISION")))
(unless (find-package "VISION::GOALPOST")
  (make-package "VISION::GOALPOST"))

(in-package "ROS")
;;//! \htmlinclude Goalpost.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass vision::Goalpost
  :super ros::object
  :slots (_header _goalpost_detected _goalpost_number _goalpost_left_range _goalpost_left_bearing _goalpost_right_range _goalpost_right_bearing _goalpost_center_range _goalpost_center_bearing ))

(defmethod vision::Goalpost
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:goalpost_detected __goalpost_detected) nil)
    ((:goalpost_number __goalpost_number) 0)
    ((:goalpost_left_range __goalpost_left_range) 0.0)
    ((:goalpost_left_bearing __goalpost_left_bearing) 0.0)
    ((:goalpost_right_range __goalpost_right_range) 0.0)
    ((:goalpost_right_bearing __goalpost_right_bearing) 0.0)
    ((:goalpost_center_range __goalpost_center_range) 0.0)
    ((:goalpost_center_bearing __goalpost_center_bearing) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _goalpost_detected __goalpost_detected)
   (setq _goalpost_number (round __goalpost_number))
   (setq _goalpost_left_range (float __goalpost_left_range))
   (setq _goalpost_left_bearing (float __goalpost_left_bearing))
   (setq _goalpost_right_range (float __goalpost_right_range))
   (setq _goalpost_right_bearing (float __goalpost_right_bearing))
   (setq _goalpost_center_range (float __goalpost_center_range))
   (setq _goalpost_center_bearing (float __goalpost_center_bearing))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:goalpost_detected
   (&optional __goalpost_detected)
   (if __goalpost_detected (setq _goalpost_detected __goalpost_detected)) _goalpost_detected)
  (:goalpost_number
   (&optional __goalpost_number)
   (if __goalpost_number (setq _goalpost_number __goalpost_number)) _goalpost_number)
  (:goalpost_left_range
   (&optional __goalpost_left_range)
   (if __goalpost_left_range (setq _goalpost_left_range __goalpost_left_range)) _goalpost_left_range)
  (:goalpost_left_bearing
   (&optional __goalpost_left_bearing)
   (if __goalpost_left_bearing (setq _goalpost_left_bearing __goalpost_left_bearing)) _goalpost_left_bearing)
  (:goalpost_right_range
   (&optional __goalpost_right_range)
   (if __goalpost_right_range (setq _goalpost_right_range __goalpost_right_range)) _goalpost_right_range)
  (:goalpost_right_bearing
   (&optional __goalpost_right_bearing)
   (if __goalpost_right_bearing (setq _goalpost_right_bearing __goalpost_right_bearing)) _goalpost_right_bearing)
  (:goalpost_center_range
   (&optional __goalpost_center_range)
   (if __goalpost_center_range (setq _goalpost_center_range __goalpost_center_range)) _goalpost_center_range)
  (:goalpost_center_bearing
   (&optional __goalpost_center_bearing)
   (if __goalpost_center_bearing (setq _goalpost_center_bearing __goalpost_center_bearing)) _goalpost_center_bearing)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; bool _goalpost_detected
    1
    ;; int8 _goalpost_number
    1
    ;; float64 _goalpost_left_range
    8
    ;; float64 _goalpost_left_bearing
    8
    ;; float64 _goalpost_right_range
    8
    ;; float64 _goalpost_right_bearing
    8
    ;; float64 _goalpost_center_range
    8
    ;; float64 _goalpost_center_bearing
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; bool _goalpost_detected
       (if _goalpost_detected (write-byte -1 s) (write-byte 0 s))
     ;; int8 _goalpost_number
       (write-byte _goalpost_number s)
     ;; float64 _goalpost_left_range
       (sys::poke _goalpost_left_range (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _goalpost_left_bearing
       (sys::poke _goalpost_left_bearing (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _goalpost_right_range
       (sys::poke _goalpost_right_range (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _goalpost_right_bearing
       (sys::poke _goalpost_right_bearing (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _goalpost_center_range
       (sys::poke _goalpost_center_range (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _goalpost_center_bearing
       (sys::poke _goalpost_center_bearing (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; bool _goalpost_detected
     (setq _goalpost_detected (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; int8 _goalpost_number
     (setq _goalpost_number (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _goalpost_number 127) (setq _goalpost_number (- _goalpost_number 256)))
   ;; float64 _goalpost_left_range
     (setq _goalpost_left_range (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _goalpost_left_bearing
     (setq _goalpost_left_bearing (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _goalpost_right_range
     (setq _goalpost_right_range (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _goalpost_right_bearing
     (setq _goalpost_right_bearing (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _goalpost_center_range
     (setq _goalpost_center_range (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _goalpost_center_bearing
     (setq _goalpost_center_bearing (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get vision::Goalpost :md5sum-) "c7e85911d22877f743fec7cea6c072ce")
(setf (get vision::Goalpost :datatype-) "vision/Goalpost")
(setf (get vision::Goalpost :definition-)
      "Header header
bool goalpost_detected
int8 goalpost_number   # 3 if only see the middle bar of goalpost. then send out the left and right point as left goalpost and right goalpost
float64 goalpost_left_range
float64 goalpost_left_bearing
float64 goalpost_right_range
float64 goalpost_right_bearing
float64 goalpost_center_range
float64 goalpost_center_bearing

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



(provide :vision/Goalpost "c7e85911d22877f743fec7cea6c072ce")


