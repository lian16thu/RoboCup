;; Auto-generated. Do not edit!


(when (boundp 'vision::field)
  (if (not (find-package "VISION"))
    (make-package "VISION"))
  (shadow 'field (find-package "VISION")))
(unless (find-package "VISION::FIELD")
  (make-package "VISION::FIELD"))

(in-package "ROS")
;;//! \htmlinclude field.msg.html


(defclass vision::field
  :super ros::object
  :slots (_width _data ))

(defmethod vision::field
  (:init
   (&key
    ((:width __width) 0)
    ((:data __data) (make-array 0 :initial-element 0 :element-type :integer))
    )
   (send-super :init)
   (setq _width (round __width))
   (setq _data __data)
   self)
  (:width
   (&optional __width)
   (if __width (setq _width __width)) _width)
  (:data
   (&optional __data)
   (if __data (setq _data __data)) _data)
  (:serialization-length
   ()
   (+
    ;; uint16 _width
    2
    ;; uint16[] _data
    (* 2    (length _data)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint16 _width
       (write-word _width s)
     ;; uint16[] _data
     (write-long (length _data) s)
     (dotimes (i (length _data))
       (write-word (elt _data i) s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint16 _width
     (setq _width (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint16[] _data
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _data (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _data i) (sys::peek buf ptr- :short)) (incf ptr- 2)
     ))
   ;;
   self)
  )

(setf (get vision::field :md5sum-) "e7238b8238eb8992ed07002d15ada745")
(setf (get vision::field :datatype-) "vision/field")
(setf (get vision::field :definition-)
      "uint16 width
uint16[] data

")



(provide :vision/field "e7238b8238eb8992ed07002d15ada745")


