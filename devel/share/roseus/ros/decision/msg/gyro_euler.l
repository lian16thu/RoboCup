;; Auto-generated. Do not edit!


(when (boundp 'decision::gyro_euler)
  (if (not (find-package "DECISION"))
    (make-package "DECISION"))
  (shadow 'gyro_euler (find-package "DECISION")))
(unless (find-package "DECISION::GYRO_EULER")
  (make-package "DECISION::GYRO_EULER"))

(in-package "ROS")
;;//! \htmlinclude gyro_euler.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass decision::gyro_euler
  :super ros::object
  :slots (_euler_angle ))

(defmethod decision::gyro_euler
  (:init
   (&key
    ((:euler_angle __euler_angle) (instance geometry_msgs::Vector3 :init))
    )
   (send-super :init)
   (setq _euler_angle __euler_angle)
   self)
  (:euler_angle
   (&rest __euler_angle)
   (if (keywordp (car __euler_angle))
       (send* _euler_angle __euler_angle)
     (progn
       (if __euler_angle (setq _euler_angle (car __euler_angle)))
       _euler_angle)))
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/Vector3 _euler_angle
    (send _euler_angle :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/Vector3 _euler_angle
       (send _euler_angle :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/Vector3 _euler_angle
     (send _euler_angle :deserialize buf ptr-) (incf ptr- (send _euler_angle :serialization-length))
   ;;
   self)
  )

(setf (get decision::gyro_euler :md5sum-) "a0ed725f607be591af507cbf7836e5ed")
(setf (get decision::gyro_euler :datatype-) "decision/gyro_euler")
(setf (get decision::gyro_euler :definition-)
      "geometry_msgs/Vector3 euler_angle

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z
")



(provide :decision/gyro_euler "a0ed725f607be591af507cbf7836e5ed")


