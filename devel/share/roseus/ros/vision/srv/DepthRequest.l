;; Auto-generated. Do not edit!


(when (boundp 'vision::DepthRequest)
  (if (not (find-package "VISION"))
    (make-package "VISION"))
  (shadow 'DepthRequest (find-package "VISION")))
(unless (find-package "VISION::DEPTHREQUEST")
  (make-package "VISION::DEPTHREQUEST"))
(unless (find-package "VISION::DEPTHREQUESTREQUEST")
  (make-package "VISION::DEPTHREQUESTREQUEST"))
(unless (find-package "VISION::DEPTHREQUESTRESPONSE")
  (make-package "VISION::DEPTHREQUESTRESPONSE"))

(in-package "ROS")





(defclass vision::DepthRequestRequest
  :super ros::object
  :slots (_u _v ))

(defmethod vision::DepthRequestRequest
  (:init
   (&key
    ((:u __u) 0)
    ((:v __v) 0)
    )
   (send-super :init)
   (setq _u (round __u))
   (setq _v (round __v))
   self)
  (:u
   (&optional __u)
   (if __u (setq _u __u)) _u)
  (:v
   (&optional __v)
   (if __v (setq _v __v)) _v)
  (:serialization-length
   ()
   (+
    ;; int32 _u
    4
    ;; int32 _v
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _u
       (write-long _u s)
     ;; int32 _v
       (write-long _v s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _u
     (setq _u (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _v
     (setq _v (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(defclass vision::DepthRequestResponse
  :super ros::object
  :slots (_depth ))

(defmethod vision::DepthRequestResponse
  (:init
   (&key
    ((:depth __depth) 0.0)
    )
   (send-super :init)
   (setq _depth (float __depth))
   self)
  (:depth
   (&optional __depth)
   (if __depth (setq _depth __depth)) _depth)
  (:serialization-length
   ()
   (+
    ;; float64 _depth
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _depth
       (sys::poke _depth (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _depth
     (setq _depth (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(defclass vision::DepthRequest
  :super ros::object
  :slots ())

(setf (get vision::DepthRequest :md5sum-) "1036fa6d5d2b46f7aa7aaecd35620560")
(setf (get vision::DepthRequest :datatype-) "vision/DepthRequest")
(setf (get vision::DepthRequest :request) vision::DepthRequestRequest)
(setf (get vision::DepthRequest :response) vision::DepthRequestResponse)

(defmethod vision::DepthRequestRequest
  (:response () (instance vision::DepthRequestResponse :init)))

(setf (get vision::DepthRequestRequest :md5sum-) "1036fa6d5d2b46f7aa7aaecd35620560")
(setf (get vision::DepthRequestRequest :datatype-) "vision/DepthRequestRequest")
(setf (get vision::DepthRequestRequest :definition-)
      "

int32 u
int32 v
---
float64 depth

")

(setf (get vision::DepthRequestResponse :md5sum-) "1036fa6d5d2b46f7aa7aaecd35620560")
(setf (get vision::DepthRequestResponse :datatype-) "vision/DepthRequestResponse")
(setf (get vision::DepthRequestResponse :definition-)
      "

int32 u
int32 v
---
float64 depth

")



(provide :vision/DepthRequest "1036fa6d5d2b46f7aa7aaecd35620560")


