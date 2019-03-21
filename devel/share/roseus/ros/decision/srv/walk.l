;; Auto-generated. Do not edit!


(when (boundp 'decision::walk)
  (if (not (find-package "DECISION"))
    (make-package "DECISION"))
  (shadow 'walk (find-package "DECISION")))
(unless (find-package "DECISION::WALK")
  (make-package "DECISION::WALK"))
(unless (find-package "DECISION::WALKREQUEST")
  (make-package "DECISION::WALKREQUEST"))
(unless (find-package "DECISION::WALKRESPONSE")
  (make-package "DECISION::WALKRESPONSE"))

(in-package "ROS")





(defclass decision::walkRequest
  :super ros::object
  :slots (_type _speed_x _speed_y _rotation_speed _step ))

(defmethod decision::walkRequest
  (:init
   (&key
    ((:type __type) 0)
    ((:speed_x __speed_x) 0.0)
    ((:speed_y __speed_y) 0.0)
    ((:rotation_speed __rotation_speed) 0.0)
    ((:step __step) 0)
    )
   (send-super :init)
   (setq _type (round __type))
   (setq _speed_x (float __speed_x))
   (setq _speed_y (float __speed_y))
   (setq _rotation_speed (float __rotation_speed))
   (setq _step (round __step))
   self)
  (:type
   (&optional __type)
   (if __type (setq _type __type)) _type)
  (:speed_x
   (&optional __speed_x)
   (if __speed_x (setq _speed_x __speed_x)) _speed_x)
  (:speed_y
   (&optional __speed_y)
   (if __speed_y (setq _speed_y __speed_y)) _speed_y)
  (:rotation_speed
   (&optional __rotation_speed)
   (if __rotation_speed (setq _rotation_speed __rotation_speed)) _rotation_speed)
  (:step
   (&optional __step)
   (if __step (setq _step __step)) _step)
  (:serialization-length
   ()
   (+
    ;; int32 _type
    4
    ;; float64 _speed_x
    8
    ;; float64 _speed_y
    8
    ;; float64 _rotation_speed
    8
    ;; int32 _step
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _type
       (write-long _type s)
     ;; float64 _speed_x
       (sys::poke _speed_x (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _speed_y
       (sys::poke _speed_y (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _rotation_speed
       (sys::poke _rotation_speed (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; int32 _step
       (write-long _step s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _type
     (setq _type (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float64 _speed_x
     (setq _speed_x (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _speed_y
     (setq _speed_y (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _rotation_speed
     (setq _rotation_speed (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; int32 _step
     (setq _step (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(defclass decision::walkResponse
  :super ros::object
  :slots (_results ))

(defmethod decision::walkResponse
  (:init
   (&key
    ((:results __results) 0)
    )
   (send-super :init)
   (setq _results (round __results))
   self)
  (:results
   (&optional __results)
   (if __results (setq _results __results)) _results)
  (:serialization-length
   ()
   (+
    ;; int32 _results
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _results
       (write-long _results s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _results
     (setq _results (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(defclass decision::walk
  :super ros::object
  :slots ())

(setf (get decision::walk :md5sum-) "abed49acefdf43a65d0e6ffd2bf04f9b")
(setf (get decision::walk :datatype-) "decision/walk")
(setf (get decision::walk :request) decision::walkRequest)
(setf (get decision::walk :response) decision::walkResponse)

(defmethod decision::walkRequest
  (:response () (instance decision::walkResponse :init)))

(setf (get decision::walkRequest :md5sum-) "abed49acefdf43a65d0e6ffd2bf04f9b")
(setf (get decision::walkRequest :datatype-) "decision/walkRequest")
(setf (get decision::walkRequest :definition-)
      "int32 type
float64 speed_x
float64 speed_y
float64 rotation_speed
int32 step
---
int32 results

")

(setf (get decision::walkResponse :md5sum-) "abed49acefdf43a65d0e6ffd2bf04f9b")
(setf (get decision::walkResponse :datatype-) "decision/walkResponse")
(setf (get decision::walkResponse :definition-)
      "int32 type
float64 speed_x
float64 speed_y
float64 rotation_speed
int32 step
---
int32 results

")



(provide :decision/walk "abed49acefdf43a65d0e6ffd2bf04f9b")


