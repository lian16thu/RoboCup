;; Auto-generated. Do not edit!


(when (boundp 'vision::Landmarks)
  (if (not (find-package "VISION"))
    (make-package "VISION"))
  (shadow 'Landmarks (find-package "VISION")))
(unless (find-package "VISION::LANDMARKS")
  (make-package "VISION::LANDMARKS"))

(in-package "ROS")
;;//! \htmlinclude Landmarks.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass vision::Landmarks
  :super ros::object
  :slots (_header _landmark_number _landmark_type _landmark_range _landmark_bearing _landmark_confidence ))

(defmethod vision::Landmarks
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:landmark_number __landmark_number) 0)
    ((:landmark_type __landmark_type) (make-array 20 :initial-element 0 :element-type :char))
    ((:landmark_range __landmark_range) (make-array 20 :initial-element 0.0 :element-type :float))
    ((:landmark_bearing __landmark_bearing) (make-array 20 :initial-element 0.0 :element-type :float))
    ((:landmark_confidence __landmark_confidence) (make-array 20 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _header __header)
   (setq _landmark_number (round __landmark_number))
   (setq _landmark_type __landmark_type)
   (setq _landmark_range __landmark_range)
   (setq _landmark_bearing __landmark_bearing)
   (setq _landmark_confidence __landmark_confidence)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:landmark_number
   (&optional __landmark_number)
   (if __landmark_number (setq _landmark_number __landmark_number)) _landmark_number)
  (:landmark_type
   (&optional __landmark_type)
   (if __landmark_type (setq _landmark_type __landmark_type)) _landmark_type)
  (:landmark_range
   (&optional __landmark_range)
   (if __landmark_range (setq _landmark_range __landmark_range)) _landmark_range)
  (:landmark_bearing
   (&optional __landmark_bearing)
   (if __landmark_bearing (setq _landmark_bearing __landmark_bearing)) _landmark_bearing)
  (:landmark_confidence
   (&optional __landmark_confidence)
   (if __landmark_confidence (setq _landmark_confidence __landmark_confidence)) _landmark_confidence)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; uint8 _landmark_number
    1
    ;; uint8[20] _landmark_type
    (* 1    20)
    ;; float64[20] _landmark_range
    (* 8    20)
    ;; float64[20] _landmark_bearing
    (* 8    20)
    ;; float32[20] _landmark_confidence
    (* 4    20)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; uint8 _landmark_number
       (write-byte _landmark_number s)
     ;; uint8[20] _landmark_type
     (princ _landmark_type s)
     ;; float64[20] _landmark_range
     (dotimes (i 20)
       (sys::poke (elt _landmark_range i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[20] _landmark_bearing
     (dotimes (i 20)
       (sys::poke (elt _landmark_bearing i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float32[20] _landmark_confidence
     (dotimes (i 20)
       (sys::poke (elt _landmark_confidence i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; uint8 _landmark_number
     (setq _landmark_number (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8[20] _landmark_type
   (setq _landmark_type (make-array 20 :element-type :char))
   (replace _landmark_type buf :start2 ptr-) (incf ptr- 20)
   ;; float64[20] _landmark_range
   (dotimes (i (length _landmark_range))
     (setf (elt _landmark_range i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[20] _landmark_bearing
   (dotimes (i (length _landmark_bearing))
     (setf (elt _landmark_bearing i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float32[20] _landmark_confidence
   (dotimes (i (length _landmark_confidence))
     (setf (elt _landmark_confidence i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     )
   ;;
   self)
  )

(setf (get vision::Landmarks :md5sum-) "6b999f1904ab908fbb4ce607a9071d78")
(setf (get vision::Landmarks :datatype-) "vision/Landmarks")
(setf (get vision::Landmarks :definition-)
      "Header header
uint8 landmark_number  
uint8[20] landmark_type     	# 1 is X, 2 is T, 3 is L, 4 is unknown, 5 is penaltypoint
float64[20] landmark_range                 
float64[20] landmark_bearing               
float32[20] landmark_confidence               # confidence 0..1

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



(provide :vision/Landmarks "6b999f1904ab908fbb4ce607a9071d78")


