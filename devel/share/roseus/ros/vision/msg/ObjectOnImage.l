;; Auto-generated. Do not edit!


(when (boundp 'vision::ObjectOnImage)
  (if (not (find-package "VISION"))
    (make-package "VISION"))
  (shadow 'ObjectOnImage (find-package "VISION")))
(unless (find-package "VISION::OBJECTONIMAGE")
  (make-package "VISION::OBJECTONIMAGE"))

(in-package "ROS")
;;//! \htmlinclude ObjectOnImage.msg.html


(defclass vision::ObjectOnImage
  :super ros::object
  :slots (_object_type _u _v ))

(defmethod vision::ObjectOnImage
  (:init
   (&key
    ((:object_type __object_type) 0)
    ((:u __u) (make-array 0 :initial-element 0 :element-type :char))
    ((:v __v) (make-array 0 :initial-element 0 :element-type :char))
    )
   (send-super :init)
   (setq _object_type (round __object_type))
   (setq _u __u)
   (setq _v __v)
   self)
  (:object_type
   (&optional __object_type)
   (if __object_type (setq _object_type __object_type)) _object_type)
  (:u
   (&optional __u)
   (if __u (setq _u __u)) _u)
  (:v
   (&optional __v)
   (if __v (setq _v __v)) _v)
  (:serialization-length
   ()
   (+
    ;; uint8 _object_type
    1
    ;; uint8[] _u
    (* 1    (length _u)) 4
    ;; uint8[] _v
    (* 1    (length _v)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8 _object_type
       (write-byte _object_type s)
     ;; uint8[] _u
     (write-long (length _u) s)
     (princ _u s)
     ;; uint8[] _v
     (write-long (length _v) s)
     (princ _v s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8 _object_type
     (setq _object_type (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8[] _u
   (let ((n (sys::peek buf ptr- :integer))) (incf ptr- 4)
     (setq _u (make-array n :element-type :char))
     (replace _u buf :start2 ptr-) (incf ptr- n))
   ;; uint8[] _v
   (let ((n (sys::peek buf ptr- :integer))) (incf ptr- 4)
     (setq _v (make-array n :element-type :char))
     (replace _v buf :start2 ptr-) (incf ptr- n))
   ;;
   self)
  )

(setf (get vision::ObjectOnImage :md5sum-) "d8b2eee0990ad6385c56c5984bb6dea1")
(setf (get vision::ObjectOnImage :datatype-) "vision/ObjectOnImage")
(setf (get vision::ObjectOnImage :definition-)
      "uint8 object_type
uint8[] u
uint8[] v

")



(provide :vision/ObjectOnImage "d8b2eee0990ad6385c56c5984bb6dea1")


