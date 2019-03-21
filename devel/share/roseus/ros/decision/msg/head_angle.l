;; Auto-generated. Do not edit!


(when (boundp 'decision::head_angle)
  (if (not (find-package "DECISION"))
    (make-package "DECISION"))
  (shadow 'head_angle (find-package "DECISION")))
(unless (find-package "DECISION::HEAD_ANGLE")
  (make-package "DECISION::HEAD_ANGLE"))

(in-package "ROS")
;;//! \htmlinclude head_angle.msg.html


(defclass decision::head_angle
  :super ros::object
  :slots (_angle_head_pitch _angle_head_yaw ))

(defmethod decision::head_angle
  (:init
   (&key
    ((:angle_head_pitch __angle_head_pitch) 0.0)
    ((:angle_head_yaw __angle_head_yaw) 0.0)
    )
   (send-super :init)
   (setq _angle_head_pitch (float __angle_head_pitch))
   (setq _angle_head_yaw (float __angle_head_yaw))
   self)
  (:angle_head_pitch
   (&optional __angle_head_pitch)
   (if __angle_head_pitch (setq _angle_head_pitch __angle_head_pitch)) _angle_head_pitch)
  (:angle_head_yaw
   (&optional __angle_head_yaw)
   (if __angle_head_yaw (setq _angle_head_yaw __angle_head_yaw)) _angle_head_yaw)
  (:serialization-length
   ()
   (+
    ;; float64 _angle_head_pitch
    8
    ;; float64 _angle_head_yaw
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _angle_head_pitch
       (sys::poke _angle_head_pitch (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _angle_head_yaw
       (sys::poke _angle_head_yaw (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _angle_head_pitch
     (setq _angle_head_pitch (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _angle_head_yaw
     (setq _angle_head_yaw (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get decision::head_angle :md5sum-) "13464c771052649b3693335e0dc785b9")
(setf (get decision::head_angle :datatype-) "decision/head_angle")
(setf (get decision::head_angle :definition-)
      "float64 angle_head_pitch	#pitch
float64 angle_head_yaw  	#yaw

")



(provide :decision/head_angle "13464c771052649b3693335e0dc785b9")


