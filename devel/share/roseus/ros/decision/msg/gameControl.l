;; Auto-generated. Do not edit!


(when (boundp 'decision::gameControl)
  (if (not (find-package "DECISION"))
    (make-package "DECISION"))
  (shadow 'gameControl (find-package "DECISION")))
(unless (find-package "DECISION::GAMECONTROL")
  (make-package "DECISION::GAMECONTROL"))

(in-package "ROS")
;;//! \htmlinclude gameControl.msg.html


(defclass decision::gameControl
  :super ros::object
  :slots (_state _secsRemaining ))

(defmethod decision::gameControl
  (:init
   (&key
    ((:state __state) 0)
    ((:secsRemaining __secsRemaining) 0)
    )
   (send-super :init)
   (setq _state (round __state))
   (setq _secsRemaining (round __secsRemaining))
   self)
  (:state
   (&optional __state)
   (if __state (setq _state __state)) _state)
  (:secsRemaining
   (&optional __secsRemaining)
   (if __secsRemaining (setq _secsRemaining __secsRemaining)) _secsRemaining)
  (:serialization-length
   ()
   (+
    ;; int32 _state
    4
    ;; int32 _secsRemaining
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _state
       (write-long _state s)
     ;; int32 _secsRemaining
       (write-long _secsRemaining s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _state
     (setq _state (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _secsRemaining
     (setq _secsRemaining (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get decision::gameControl :md5sum-) "4f8fd4e28d3da76570da7ab09212fb90")
(setf (get decision::gameControl :datatype-) "decision/gameControl")
(setf (get decision::gameControl :definition-)
      "int32 state
int32 secsRemaining

")



(provide :decision/gameControl "4f8fd4e28d3da76570da7ab09212fb90")


