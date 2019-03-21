;; Auto-generated. Do not edit!


(when (boundp 'decision::GoalData)
  (if (not (find-package "DECISION"))
    (make-package "DECISION"))
  (shadow 'GoalData (find-package "DECISION")))
(unless (find-package "DECISION::GOALDATA")
  (make-package "DECISION::GOALDATA"))

(in-package "ROS")
;;//! \htmlinclude GoalData.msg.html


(defclass decision::GoalData
  :super ros::object
  :slots (_goal _leftx _lefty _rightx _righty ))

(defmethod decision::GoalData
  (:init
   (&key
    ((:goal __goal) 0)
    ((:leftx __leftx) 0.0)
    ((:lefty __lefty) 0.0)
    ((:rightx __rightx) 0.0)
    ((:righty __righty) 0.0)
    )
   (send-super :init)
   (setq _goal (round __goal))
   (setq _leftx (float __leftx))
   (setq _lefty (float __lefty))
   (setq _rightx (float __rightx))
   (setq _righty (float __righty))
   self)
  (:goal
   (&optional __goal)
   (if __goal (setq _goal __goal)) _goal)
  (:leftx
   (&optional __leftx)
   (if __leftx (setq _leftx __leftx)) _leftx)
  (:lefty
   (&optional __lefty)
   (if __lefty (setq _lefty __lefty)) _lefty)
  (:rightx
   (&optional __rightx)
   (if __rightx (setq _rightx __rightx)) _rightx)
  (:righty
   (&optional __righty)
   (if __righty (setq _righty __righty)) _righty)
  (:serialization-length
   ()
   (+
    ;; uint8 _goal
    1
    ;; float32 _leftx
    4
    ;; float32 _lefty
    4
    ;; float32 _rightx
    4
    ;; float32 _righty
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8 _goal
       (write-byte _goal s)
     ;; float32 _leftx
       (sys::poke _leftx (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _lefty
       (sys::poke _lefty (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _rightx
       (sys::poke _rightx (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _righty
       (sys::poke _righty (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8 _goal
     (setq _goal (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; float32 _leftx
     (setq _leftx (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _lefty
     (setq _lefty (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _rightx
     (setq _rightx (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _righty
     (setq _righty (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get decision::GoalData :md5sum-) "657b4b89e12eaea48f7336974eb25b11")
(setf (get decision::GoalData :datatype-) "decision/GoalData")
(setf (get decision::GoalData :definition-)
      "uint8 goal
float32 leftx
float32 lefty
float32 rightx
float32 righty


")



(provide :decision/GoalData "657b4b89e12eaea48f7336974eb25b11")


