;; Auto-generated. Do not edit!


(when (boundp 'vision::Ball)
  (if (not (find-package "VISION"))
    (make-package "VISION"))
  (shadow 'Ball (find-package "VISION")))
(unless (find-package "VISION::BALL")
  (make-package "VISION::BALL"))

(in-package "ROS")
;;//! \htmlinclude Ball.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass vision::Ball
  :super ros::object
  :slots (_header _ball_detected _ball_range _ball_bearing _ball_radius _ball_center_x _ball_center_y _kick_time ))

(defmethod vision::Ball
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:ball_detected __ball_detected) nil)
    ((:ball_range __ball_range) 0.0)
    ((:ball_bearing __ball_bearing) 0.0)
    ((:ball_radius __ball_radius) 0.0)
    ((:ball_center_x __ball_center_x) 0)
    ((:ball_center_y __ball_center_y) 0)
    ((:kick_time __kick_time) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _ball_detected __ball_detected)
   (setq _ball_range (float __ball_range))
   (setq _ball_bearing (float __ball_bearing))
   (setq _ball_radius (float __ball_radius))
   (setq _ball_center_x (round __ball_center_x))
   (setq _ball_center_y (round __ball_center_y))
   (setq _kick_time (float __kick_time))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:ball_detected
   (&optional __ball_detected)
   (if __ball_detected (setq _ball_detected __ball_detected)) _ball_detected)
  (:ball_range
   (&optional __ball_range)
   (if __ball_range (setq _ball_range __ball_range)) _ball_range)
  (:ball_bearing
   (&optional __ball_bearing)
   (if __ball_bearing (setq _ball_bearing __ball_bearing)) _ball_bearing)
  (:ball_radius
   (&optional __ball_radius)
   (if __ball_radius (setq _ball_radius __ball_radius)) _ball_radius)
  (:ball_center_x
   (&optional __ball_center_x)
   (if __ball_center_x (setq _ball_center_x __ball_center_x)) _ball_center_x)
  (:ball_center_y
   (&optional __ball_center_y)
   (if __ball_center_y (setq _ball_center_y __ball_center_y)) _ball_center_y)
  (:kick_time
   (&optional __kick_time)
   (if __kick_time (setq _kick_time __kick_time)) _kick_time)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; bool _ball_detected
    1
    ;; float64 _ball_range
    8
    ;; float64 _ball_bearing
    8
    ;; float64 _ball_radius
    8
    ;; int64 _ball_center_x
    8
    ;; int64 _ball_center_y
    8
    ;; float64 _kick_time
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; bool _ball_detected
       (if _ball_detected (write-byte -1 s) (write-byte 0 s))
     ;; float64 _ball_range
       (sys::poke _ball_range (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _ball_bearing
       (sys::poke _ball_bearing (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _ball_radius
       (sys::poke _ball_radius (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; int64 _ball_center_x
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _ball_center_x (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _ball_center_x) (= (length (_ball_center_x . bv)) 2)) ;; bignum
              (write-long (ash (elt (_ball_center_x . bv) 0) 0) s)
              (write-long (ash (elt (_ball_center_x . bv) 1) -1) s))
             ((and (class _ball_center_x) (= (length (_ball_center_x . bv)) 1)) ;; big1
              (write-long (elt (_ball_center_x . bv) 0) s)
              (write-long (if (>= _ball_center_x 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _ball_center_x s)(write-long (if (>= _ball_center_x 0) 0 #xffffffff) s)))
     ;; int64 _ball_center_y
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _ball_center_y (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _ball_center_y) (= (length (_ball_center_y . bv)) 2)) ;; bignum
              (write-long (ash (elt (_ball_center_y . bv) 0) 0) s)
              (write-long (ash (elt (_ball_center_y . bv) 1) -1) s))
             ((and (class _ball_center_y) (= (length (_ball_center_y . bv)) 1)) ;; big1
              (write-long (elt (_ball_center_y . bv) 0) s)
              (write-long (if (>= _ball_center_y 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _ball_center_y s)(write-long (if (>= _ball_center_y 0) 0 #xffffffff) s)))
     ;; float64 _kick_time
       (sys::poke _kick_time (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; bool _ball_detected
     (setq _ball_detected (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; float64 _ball_range
     (setq _ball_range (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _ball_bearing
     (setq _ball_bearing (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _ball_radius
     (setq _ball_radius (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; int64 _ball_center_x
#+(or :alpha :irix6 :x86_64)
      (setf _ball_center_x (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _ball_center_x (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; int64 _ball_center_y
#+(or :alpha :irix6 :x86_64)
      (setf _ball_center_y (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _ball_center_y (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; float64 _kick_time
     (setq _kick_time (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get vision::Ball :md5sum-) "07dd74c62bc00b3a12c5e1cd0e3a4159")
(setf (get vision::Ball :datatype-) "vision/Ball")
(setf (get vision::Ball :definition-)
      "Header header
bool ball_detected
float64 ball_range
float64 ball_bearing
float64 ball_radius
int64 ball_center_x
int64 ball_center_y
float64 kick_time

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



(provide :vision/Ball "07dd74c62bc00b3a12c5e1cd0e3a4159")


