;; Auto-generated. Do not edit!


(when (boundp 'decision::Obstacle)
  (if (not (find-package "DECISION"))
    (make-package "DECISION"))
  (shadow 'Obstacle (find-package "DECISION")))
(unless (find-package "DECISION::OBSTACLE")
  (make-package "DECISION::OBSTACLE"))

(in-package "ROS")
;;//! \htmlinclude Obstacle.msg.html


(defclass decision::Obstacle
  :super ros::object
  :slots (_bObstacleWasSeen _iObstacleNumber _iLeftEdgeInImageX _iLeftEdgeInImageY _iRightEdgeInImageX _iRightEdgeInImageY _iHeightInImage _fDistance _fAngle ))

(defmethod decision::Obstacle
  (:init
   (&key
    ((:bObstacleWasSeen __bObstacleWasSeen) 0)
    ((:iObstacleNumber __iObstacleNumber) 0)
    ((:iLeftEdgeInImageX __iLeftEdgeInImageX) (make-array 0 :initial-element 0 :element-type :integer))
    ((:iLeftEdgeInImageY __iLeftEdgeInImageY) (make-array 0 :initial-element 0 :element-type :integer))
    ((:iRightEdgeInImageX __iRightEdgeInImageX) (make-array 0 :initial-element 0 :element-type :integer))
    ((:iRightEdgeInImageY __iRightEdgeInImageY) (make-array 0 :initial-element 0 :element-type :integer))
    ((:iHeightInImage __iHeightInImage) (make-array 0 :initial-element 0 :element-type :integer))
    ((:fDistance __fDistance) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:fAngle __fAngle) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _bObstacleWasSeen (round __bObstacleWasSeen))
   (setq _iObstacleNumber (round __iObstacleNumber))
   (setq _iLeftEdgeInImageX __iLeftEdgeInImageX)
   (setq _iLeftEdgeInImageY __iLeftEdgeInImageY)
   (setq _iRightEdgeInImageX __iRightEdgeInImageX)
   (setq _iRightEdgeInImageY __iRightEdgeInImageY)
   (setq _iHeightInImage __iHeightInImage)
   (setq _fDistance __fDistance)
   (setq _fAngle __fAngle)
   self)
  (:bObstacleWasSeen
   (&optional __bObstacleWasSeen)
   (if __bObstacleWasSeen (setq _bObstacleWasSeen __bObstacleWasSeen)) _bObstacleWasSeen)
  (:iObstacleNumber
   (&optional __iObstacleNumber)
   (if __iObstacleNumber (setq _iObstacleNumber __iObstacleNumber)) _iObstacleNumber)
  (:iLeftEdgeInImageX
   (&optional __iLeftEdgeInImageX)
   (if __iLeftEdgeInImageX (setq _iLeftEdgeInImageX __iLeftEdgeInImageX)) _iLeftEdgeInImageX)
  (:iLeftEdgeInImageY
   (&optional __iLeftEdgeInImageY)
   (if __iLeftEdgeInImageY (setq _iLeftEdgeInImageY __iLeftEdgeInImageY)) _iLeftEdgeInImageY)
  (:iRightEdgeInImageX
   (&optional __iRightEdgeInImageX)
   (if __iRightEdgeInImageX (setq _iRightEdgeInImageX __iRightEdgeInImageX)) _iRightEdgeInImageX)
  (:iRightEdgeInImageY
   (&optional __iRightEdgeInImageY)
   (if __iRightEdgeInImageY (setq _iRightEdgeInImageY __iRightEdgeInImageY)) _iRightEdgeInImageY)
  (:iHeightInImage
   (&optional __iHeightInImage)
   (if __iHeightInImage (setq _iHeightInImage __iHeightInImage)) _iHeightInImage)
  (:fDistance
   (&optional __fDistance)
   (if __fDistance (setq _fDistance __fDistance)) _fDistance)
  (:fAngle
   (&optional __fAngle)
   (if __fAngle (setq _fAngle __fAngle)) _fAngle)
  (:serialization-length
   ()
   (+
    ;; uint8 _bObstacleWasSeen
    1
    ;; uint16 _iObstacleNumber
    2
    ;; uint16[] _iLeftEdgeInImageX
    (* 2    (length _iLeftEdgeInImageX)) 4
    ;; uint16[] _iLeftEdgeInImageY
    (* 2    (length _iLeftEdgeInImageY)) 4
    ;; uint16[] _iRightEdgeInImageX
    (* 2    (length _iRightEdgeInImageX)) 4
    ;; uint16[] _iRightEdgeInImageY
    (* 2    (length _iRightEdgeInImageY)) 4
    ;; uint16[] _iHeightInImage
    (* 2    (length _iHeightInImage)) 4
    ;; float32[] _fDistance
    (* 4    (length _fDistance)) 4
    ;; float32[] _fAngle
    (* 4    (length _fAngle)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8 _bObstacleWasSeen
       (write-byte _bObstacleWasSeen s)
     ;; uint16 _iObstacleNumber
       (write-word _iObstacleNumber s)
     ;; uint16[] _iLeftEdgeInImageX
     (write-long (length _iLeftEdgeInImageX) s)
     (dotimes (i (length _iLeftEdgeInImageX))
       (write-word (elt _iLeftEdgeInImageX i) s)
       )
     ;; uint16[] _iLeftEdgeInImageY
     (write-long (length _iLeftEdgeInImageY) s)
     (dotimes (i (length _iLeftEdgeInImageY))
       (write-word (elt _iLeftEdgeInImageY i) s)
       )
     ;; uint16[] _iRightEdgeInImageX
     (write-long (length _iRightEdgeInImageX) s)
     (dotimes (i (length _iRightEdgeInImageX))
       (write-word (elt _iRightEdgeInImageX i) s)
       )
     ;; uint16[] _iRightEdgeInImageY
     (write-long (length _iRightEdgeInImageY) s)
     (dotimes (i (length _iRightEdgeInImageY))
       (write-word (elt _iRightEdgeInImageY i) s)
       )
     ;; uint16[] _iHeightInImage
     (write-long (length _iHeightInImage) s)
     (dotimes (i (length _iHeightInImage))
       (write-word (elt _iHeightInImage i) s)
       )
     ;; float32[] _fDistance
     (write-long (length _fDistance) s)
     (dotimes (i (length _fDistance))
       (sys::poke (elt _fDistance i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[] _fAngle
     (write-long (length _fAngle) s)
     (dotimes (i (length _fAngle))
       (sys::poke (elt _fAngle i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8 _bObstacleWasSeen
     (setq _bObstacleWasSeen (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint16 _iObstacleNumber
     (setq _iObstacleNumber (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint16[] _iLeftEdgeInImageX
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _iLeftEdgeInImageX (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _iLeftEdgeInImageX i) (sys::peek buf ptr- :short)) (incf ptr- 2)
     ))
   ;; uint16[] _iLeftEdgeInImageY
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _iLeftEdgeInImageY (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _iLeftEdgeInImageY i) (sys::peek buf ptr- :short)) (incf ptr- 2)
     ))
   ;; uint16[] _iRightEdgeInImageX
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _iRightEdgeInImageX (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _iRightEdgeInImageX i) (sys::peek buf ptr- :short)) (incf ptr- 2)
     ))
   ;; uint16[] _iRightEdgeInImageY
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _iRightEdgeInImageY (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _iRightEdgeInImageY i) (sys::peek buf ptr- :short)) (incf ptr- 2)
     ))
   ;; uint16[] _iHeightInImage
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _iHeightInImage (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _iHeightInImage i) (sys::peek buf ptr- :short)) (incf ptr- 2)
     ))
   ;; float32[] _fDistance
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _fDistance (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _fDistance i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32[] _fAngle
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _fAngle (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _fAngle i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;;
   self)
  )

(setf (get decision::Obstacle :md5sum-) "da6170297c4a034fb40d276bbabe829c")
(setf (get decision::Obstacle :datatype-) "decision/Obstacle")
(setf (get decision::Obstacle :definition-)
      "uint8 bObstacleWasSeen

uint16 iObstacleNumber  #all together, if 2 obstacles seen, it's 2
uint16[] iLeftEdgeInImageX
uint16[] iLeftEdgeInImageY
uint16[] iRightEdgeInImageX
uint16[] iRightEdgeInImageY
uint16[] iHeightInImage
float32[] fDistance
float32[] fAngle

")



(provide :decision/Obstacle "da6170297c4a034fb40d276bbabe829c")


