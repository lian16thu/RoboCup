;; Auto-generated. Do not edit!


(when (boundp 'decision::Pathplaning_the)
  (if (not (find-package "DECISION"))
    (make-package "DECISION"))
  (shadow 'Pathplaning_the (find-package "DECISION")))
(unless (find-package "DECISION::PATHPLANING_THE")
  (make-package "DECISION::PATHPLANING_THE"))
(unless (find-package "DECISION::PATHPLANING_THEREQUEST")
  (make-package "DECISION::PATHPLANING_THEREQUEST"))
(unless (find-package "DECISION::PATHPLANING_THERESPONSE")
  (make-package "DECISION::PATHPLANING_THERESPONSE"))

(in-package "ROS")





(defclass decision::Pathplaning_theRequest
  :super ros::object
  :slots (_state ))

(defmethod decision::Pathplaning_theRequest
  (:init
   (&key
    ((:state __state) nil)
    )
   (send-super :init)
   (setq _state __state)
   self)
  (:state
   (&optional __state)
   (if __state (setq _state __state)) _state)
  (:serialization-length
   ()
   (+
    ;; bool _state
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _state
       (if _state (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _state
     (setq _state (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass decision::Pathplaning_theResponse
  :super ros::object
  :slots (_theta_result ))

(defmethod decision::Pathplaning_theResponse
  (:init
   (&key
    ((:theta_result __theta_result) 0.0)
    )
   (send-super :init)
   (setq _theta_result (float __theta_result))
   self)
  (:theta_result
   (&optional __theta_result)
   (if __theta_result (setq _theta_result __theta_result)) _theta_result)
  (:serialization-length
   ()
   (+
    ;; float32 _theta_result
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _theta_result
       (sys::poke _theta_result (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _theta_result
     (setq _theta_result (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(defclass decision::Pathplaning_the
  :super ros::object
  :slots ())

(setf (get decision::Pathplaning_the :md5sum-) "743721b9c6ac4f45f0566da711cade92")
(setf (get decision::Pathplaning_the :datatype-) "decision/Pathplaning_the")
(setf (get decision::Pathplaning_the :request) decision::Pathplaning_theRequest)
(setf (get decision::Pathplaning_the :response) decision::Pathplaning_theResponse)

(defmethod decision::Pathplaning_theRequest
  (:response () (instance decision::Pathplaning_theResponse :init)))

(setf (get decision::Pathplaning_theRequest :md5sum-) "743721b9c6ac4f45f0566da711cade92")
(setf (get decision::Pathplaning_theRequest :datatype-) "decision/Pathplaning_theRequest")
(setf (get decision::Pathplaning_theRequest :definition-)
      "bool state
---
float32 theta_result

")

(setf (get decision::Pathplaning_theResponse :md5sum-) "743721b9c6ac4f45f0566da711cade92")
(setf (get decision::Pathplaning_theResponse :datatype-) "decision/Pathplaning_theResponse")
(setf (get decision::Pathplaning_theResponse :definition-)
      "bool state
---
float32 theta_result

")



(provide :decision/Pathplaning_the "743721b9c6ac4f45f0566da711cade92")


