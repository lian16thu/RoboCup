;; Auto-generated. Do not edit!


(when (boundp 'decision::head)
  (if (not (find-package "DECISION"))
    (make-package "DECISION"))
  (shadow 'head (find-package "DECISION")))
(unless (find-package "DECISION::HEAD")
  (make-package "DECISION::HEAD"))
(unless (find-package "DECISION::HEADREQUEST")
  (make-package "DECISION::HEADREQUEST"))
(unless (find-package "DECISION::HEADRESPONSE")
  (make-package "DECISION::HEADRESPONSE"))

(in-package "ROS")





(defclass decision::headRequest
  :super ros::object
  :slots (_pitch _yaw _PID ))

(defmethod decision::headRequest
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

(defclass decision::headResponse
  :super ros::object
  :slots (_result ))

(defmethod decision::headResponse
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

(defclass decision::head
  :super ros::object
  :slots ())

(setf (get decision::head :md5sum-) "e4aff7be97fd6298f4a9b30300ac7316")
(setf (get decision::head :datatype-) "decision/head")
(setf (get decision::head :request) decision::headRequest)
(setf (get decision::head :response) decision::headResponse)

(defmethod decision::headRequest
  (:response () (instance decision::headResponse :init)))

(setf (get decision::headRequest :md5sum-) "e4aff7be97fd6298f4a9b30300ac7316")
(setf (get decision::headRequest :datatype-) "decision/headRequest")
(setf (get decision::headRequest :definition-)
      "float64 pitch
float64 yaw
int32 PID
---
int32 result

")

(setf (get decision::headResponse :md5sum-) "e4aff7be97fd6298f4a9b30300ac7316")
(setf (get decision::headResponse :datatype-) "decision/headResponse")
(setf (get decision::headResponse :definition-)
      "float64 pitch
float64 yaw
int32 PID
---
int32 result

")



(provide :decision/head "e4aff7be97fd6298f4a9b30300ac7316")


