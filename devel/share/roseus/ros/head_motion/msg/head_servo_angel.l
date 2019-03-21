;; Auto-generated. Do not edit!


(when (boundp 'head_motion::head_servo_angel)
  (if (not (find-package "HEAD_MOTION"))
    (make-package "HEAD_MOTION"))
  (shadow 'head_servo_angel (find-package "HEAD_MOTION")))
(unless (find-package "HEAD_MOTION::HEAD_SERVO_ANGEL")
  (make-package "HEAD_MOTION::HEAD_SERVO_ANGEL"))

(in-package "ROS")
;;//! \htmlinclude head_servo_angel.msg.html


(defclass head_motion::head_servo_angel
  :super ros::object
  :slots (_pitch _yaw ))

(defmethod head_motion::head_servo_angel
  (:init
   (&key
    ((:pitch __pitch) 0.0)
    ((:yaw __yaw) 0.0)
    )
   (send-super :init)
   (setq _pitch (float __pitch))
   (setq _yaw (float __yaw))
   self)
  (:pitch
   (&optional __pitch)
   (if __pitch (setq _pitch __pitch)) _pitch)
  (:yaw
   (&optional __yaw)
   (if __yaw (setq _yaw __yaw)) _yaw)
  (:serialization-length
   ()
   (+
    ;; float64 _pitch
    8
    ;; float64 _yaw
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _pitch
       (sys::poke _pitch (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _yaw
       (sys::poke _yaw (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _pitch
     (setq _pitch (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _yaw
     (setq _yaw (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get head_motion::head_servo_angel :md5sum-) "64da7800e4533f600c4d4b06d8aaa339")
(setf (get head_motion::head_servo_angel :datatype-) "head_motion/head_servo_angel")
(setf (get head_motion::head_servo_angel :definition-)
      "float64 pitch
float64 yaw

")



(provide :head_motion/head_servo_angel "64da7800e4533f600c4d4b06d8aaa339")


