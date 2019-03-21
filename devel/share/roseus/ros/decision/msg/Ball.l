;; Auto-generated. Do not edit!


(when (boundp 'decision::Ball)
  (if (not (find-package "DECISION"))
    (make-package "DECISION"))
  (shadow 'Ball (find-package "DECISION")))
(unless (find-package "DECISION::BALL")
  (make-package "DECISION::BALL"))

(in-package "ROS")
;;//! \htmlinclude Ball.msg.html


(defclass decision::Ball
  :super ros::object
  :slots (_bBallWasSeen _iCenterInImageX _iCenterInImageY _iRadiusInImage _fDistance _fAngle ))

(defmethod decision::Ball
  (:init
   (&key
    ((:bBallWasSeen __bBallWasSeen) 0)
    ((:iCenterInImageX __iCenterInImageX) 0)
    ((:iCenterInImageY __iCenterInImageY) 0)
    ((:iRadiusInImage __iRadiusInImage) 0)
    ((:fDistance __fDistance) 0.0)
    ((:fAngle __fAngle) 0.0)
    )
   (send-super :init)
   (setq _bBallWasSeen (round __bBallWasSeen))
   (setq _iCenterInImageX (round __iCenterInImageX))
   (setq _iCenterInImageY (round __iCenterInImageY))
   (setq _iRadiusInImage (round __iRadiusInImage))
   (setq _fDistance (float __fDistance))
   (setq _fAngle (float __fAngle))
   self)
  (:bBallWasSeen
   (&optional __bBallWasSeen)
   (if __bBallWasSeen (setq _bBallWasSeen __bBallWasSeen)) _bBallWasSeen)
  (:iCenterInImageX
   (&optional __iCenterInImageX)
   (if __iCenterInImageX (setq _iCenterInImageX __iCenterInImageX)) _iCenterInImageX)
  (:iCenterInImageY
   (&optional __iCenterInImageY)
   (if __iCenterInImageY (setq _iCenterInImageY __iCenterInImageY)) _iCenterInImageY)
  (:iRadiusInImage
   (&optional __iRadiusInImage)
   (if __iRadiusInImage (setq _iRadiusInImage __iRadiusInImage)) _iRadiusInImage)
  (:fDistance
   (&optional __fDistance)
   (if __fDistance (setq _fDistance __fDistance)) _fDistance)
  (:fAngle
   (&optional __fAngle)
   (if __fAngle (setq _fAngle __fAngle)) _fAngle)
  (:serialization-length
   ()
   (+
    ;; uint8 _bBallWasSeen
    1
    ;; uint16 _iCenterInImageX
    2
    ;; uint16 _iCenterInImageY
    2
    ;; uint16 _iRadiusInImage
    2
    ;; float32 _fDistance
    4
    ;; float32 _fAngle
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8 _bBallWasSeen
       (write-byte _bBallWasSeen s)
     ;; uint16 _iCenterInImageX
       (write-word _iCenterInImageX s)
     ;; uint16 _iCenterInImageY
       (write-word _iCenterInImageY s)
     ;; uint16 _iRadiusInImage
       (write-word _iRadiusInImage s)
     ;; float32 _fDistance
       (sys::poke _fDistance (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _fAngle
       (sys::poke _fAngle (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8 _bBallWasSeen
     (setq _bBallWasSeen (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint16 _iCenterInImageX
     (setq _iCenterInImageX (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint16 _iCenterInImageY
     (setq _iCenterInImageY (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint16 _iRadiusInImage
     (setq _iRadiusInImage (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; float32 _fDistance
     (setq _fDistance (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _fAngle
     (setq _fAngle (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get decision::Ball :md5sum-) "b1776e77d61a18b8d498a831f6c9807a")
(setf (get decision::Ball :datatype-) "decision/Ball")
(setf (get decision::Ball :definition-)
      "uint8 bBallWasSeen
uint16 iCenterInImageX
uint16 iCenterInImageY
uint16 iRadiusInImage
float32 fDistance
float32 fAngle

")



(provide :decision/Ball "b1776e77d61a18b8d498a831f6c9807a")


