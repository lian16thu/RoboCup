;; Auto-generated. Do not edit!


(when (boundp 'vision::Lines)
  (if (not (find-package "VISION"))
    (make-package "VISION"))
  (shadow 'Lines (find-package "VISION")))
(unless (find-package "VISION::LINES")
  (make-package "VISION::LINES"))

(in-package "ROS")
;;//! \htmlinclude Lines.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass vision::Lines
  :super ros::object
  :slots (_header _lines_number _line_start_x _line_start_y _line_end_x _line_end_y _line_confidence ))

(defmethod vision::Lines
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:lines_number __lines_number) 0)
    ((:line_start_x __line_start_x) (make-array 20 :initial-element 0.0 :element-type :float))
    ((:line_start_y __line_start_y) (make-array 20 :initial-element 0.0 :element-type :float))
    ((:line_end_x __line_end_x) (make-array 20 :initial-element 0.0 :element-type :float))
    ((:line_end_y __line_end_y) (make-array 20 :initial-element 0.0 :element-type :float))
    ((:line_confidence __line_confidence) (make-array 20 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _header __header)
   (setq _lines_number (round __lines_number))
   (setq _line_start_x __line_start_x)
   (setq _line_start_y __line_start_y)
   (setq _line_end_x __line_end_x)
   (setq _line_end_y __line_end_y)
   (setq _line_confidence __line_confidence)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:lines_number
   (&optional __lines_number)
   (if __lines_number (setq _lines_number __lines_number)) _lines_number)
  (:line_start_x
   (&optional __line_start_x)
   (if __line_start_x (setq _line_start_x __line_start_x)) _line_start_x)
  (:line_start_y
   (&optional __line_start_y)
   (if __line_start_y (setq _line_start_y __line_start_y)) _line_start_y)
  (:line_end_x
   (&optional __line_end_x)
   (if __line_end_x (setq _line_end_x __line_end_x)) _line_end_x)
  (:line_end_y
   (&optional __line_end_y)
   (if __line_end_y (setq _line_end_y __line_end_y)) _line_end_y)
  (:line_confidence
   (&optional __line_confidence)
   (if __line_confidence (setq _line_confidence __line_confidence)) _line_confidence)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; uint8 _lines_number
    1
    ;; float64[20] _line_start_x
    (* 8    20)
    ;; float64[20] _line_start_y
    (* 8    20)
    ;; float64[20] _line_end_x
    (* 8    20)
    ;; float64[20] _line_end_y
    (* 8    20)
    ;; float32[20] _line_confidence
    (* 4    20)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; uint8 _lines_number
       (write-byte _lines_number s)
     ;; float64[20] _line_start_x
     (dotimes (i 20)
       (sys::poke (elt _line_start_x i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[20] _line_start_y
     (dotimes (i 20)
       (sys::poke (elt _line_start_y i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[20] _line_end_x
     (dotimes (i 20)
       (sys::poke (elt _line_end_x i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[20] _line_end_y
     (dotimes (i 20)
       (sys::poke (elt _line_end_y i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float32[20] _line_confidence
     (dotimes (i 20)
       (sys::poke (elt _line_confidence i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; uint8 _lines_number
     (setq _lines_number (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; float64[20] _line_start_x
   (dotimes (i (length _line_start_x))
     (setf (elt _line_start_x i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[20] _line_start_y
   (dotimes (i (length _line_start_y))
     (setf (elt _line_start_y i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[20] _line_end_x
   (dotimes (i (length _line_end_x))
     (setf (elt _line_end_x i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[20] _line_end_y
   (dotimes (i (length _line_end_y))
     (setf (elt _line_end_y i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float32[20] _line_confidence
   (dotimes (i (length _line_confidence))
     (setf (elt _line_confidence i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     )
   ;;
   self)
  )

(setf (get vision::Lines :md5sum-) "26fb3619336498992b94003d734b3dd9")
(setf (get vision::Lines :datatype-) "vision/Lines")
(setf (get vision::Lines :definition-)
      "Header header
uint8 lines_number  
float64[20] line_start_x                 
float64[20] line_start_y 
float64[20] line_end_x                 
float64[20] line_end_y               
float32[20] line_confidence               # confidence 0..1

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



(provide :vision/Lines "26fb3619336498992b94003d734b3dd9")


