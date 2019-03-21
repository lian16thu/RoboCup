;; Auto-generated. Do not edit!


(when (boundp 'head_motion::head_control)
  (if (not (find-package "HEAD_MOTION"))
    (make-package "HEAD_MOTION"))
  (shadow 'head_control (find-package "HEAD_MOTION")))
(unless (find-package "HEAD_MOTION::HEAD_CONTROL")
  (make-package "HEAD_MOTION::HEAD_CONTROL"))
(unless (find-package "HEAD_MOTION::HEAD_CONTROLREQUEST")
  (make-package "HEAD_MOTION::HEAD_CONTROLREQUEST"))
(unless (find-package "HEAD_MOTION::HEAD_CONTROLRESPONSE")
  (make-package "HEAD_MOTION::HEAD_CONTROLRESPONSE"))

(in-package "ROS")





(defclass head_motion::head_controlRequest
  :super ros::object
  :slots (_pitch _yaw _PID ))

(defmethod head_motion::head_controlRequest
  (:init
   (&key
    ((:pitch __pitch) 0.0)
    ((:yaw __yaw) 0.0)
    ((:PID __PID) 0)
    )
   (send-super :init)
   (setq _pitch (float __pitch))
   (setq _yaw (float __yaw))
   (setq _PID (round __PID))
   self)
  (:pitch
   (&optional __pitch)
   (if __pitch (setq _pitch __pitch)) _pitch)
  (:yaw
   (&optional __yaw)
   (if __yaw (setq _yaw __yaw)) _yaw)
  (:PID
   (&optional __PID)
   (if __PID (setq _PID __PID)) _PID)
  (:serialization-length
   ()
   (+
    ;; float64 _pitch
    8
    ;; float64 _yaw
    8
    ;; int32 _PID
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _pitch
       (sys::poke _pitch (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _yaw
       (sys::poke _yaw (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; int32 _PID
       (write-long _PID s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _pitch
     (setq _pitch (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _yaw
     (setq _yaw (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; int32 _PID
     (setq _PID (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(defclass head_motion::head_controlResponse
  :super ros::object
  :slots (_result ))

(defmethod head_motion::head_controlResponse
  (:init
   (&key
    ((:result __result) 0)
    )
   (send-super :init)
   (setq _result (round __result))
   self)
  (:result
   (&optional __result)
   (if __result (setq _result __result)) _result)
  (:serialization-length
   ()
   (+
    ;; int32 _result
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _result
       (write-long _result s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _result
     (setq _result (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(defclass head_motion::head_control
  :super ros::object
  :slots ())

(setf (get head_motion::head_control :md5sum-) "e4aff7be97fd6298f4a9b30300ac7316")
(setf (get head_motion::head_control :datatype-) "head_motion/head_control")
(setf (get head_motion::head_control :request) head_motion::head_controlRequest)
(setf (get head_motion::head_control :response) head_motion::head_controlResponse)

(defmethod head_motion::head_controlRequest
  (:response () (instance head_motion::head_controlResponse :init)))

(setf (get head_motion::head_controlRequest :md5sum-) "e4aff7be97fd6298f4a9b30300ac7316")
(setf (get head_motion::head_controlRequest :datatype-) "head_motion/head_controlRequest")
(setf (get head_motion::head_controlRequest :definition-)
      "float64 pitch
float64 yaw
int32 PID
---
int32 result
")

(setf (get head_motion::head_controlResponse :md5sum-) "e4aff7be97fd6298f4a9b30300ac7316")
(setf (get head_motion::head_controlResponse :datatype-) "head_motion/head_controlResponse")
(setf (get head_motion::head_controlResponse :definition-)
      "float64 pitch
float64 yaw
int32 PID
---
int32 result
")



(provide :head_motion/head_control "e4aff7be97fd6298f4a9b30300ac7316")


