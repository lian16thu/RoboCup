;; Auto-generated. Do not edit!


(when (boundp 'vision::Obstacle)
  (if (not (find-package "VISION"))
    (make-package "VISION"))
  (shadow 'Obstacle (find-package "VISION")))
(unless (find-package "VISION::OBSTACLE")
  (make-package "VISION::OBSTACLE"))

(in-package "ROS")
;;//! \htmlinclude Obstacle.msg.html


(defclass vision::Obstacle
  :super ros::object
  :slots (_bObstacleWasSeen _iLeftEdgeInImageX _iLeftEdgeInImageY _iRightEdgeInImageX _iRightEdgeInImageY _iHeightInImage _inumber _iOthersLeftEdgeInImageX _iOthersRightEdgeInImageX _iOthersInImageY _iOthersHeightInImage ))

(defmethod vision::Obstacle
  (:init
   (&key
    ((:bObstacleWasSeen __bObstacleWasSeen) 0)
    ((:iLeftEdgeInImageX __iLeftEdgeInImageX) 0)
    ((:iLeftEdgeInImageY __iLeftEdgeInImageY) 0)
    ((:iRightEdgeInImageX __iRightEdgeInImageX) 0)
    ((:iRightEdgeInImageY __iRightEdgeInImageY) 0)
    ((:iHeightInImage __iHeightInImage) 0)
    ((:inumber __inumber) 0)
    ((:iOthersLeftEdgeInImageX __iOthersLeftEdgeInImageX) (make-array 0 :initial-element 0 :element-type :integer))
    ((:iOthersRightEdgeInImageX __iOthersRightEdgeInImageX) (make-array 0 :initial-element 0 :element-type :integer))
    ((:iOthersInImageY __iOthersInImageY) (make-array 0 :initial-element 0 :element-type :integer))
    ((:iOthersHeightInImage __iOthersHeightInImage) (make-array 0 :initial-element 0 :element-type :integer))
    )
   (send-super :init)
   (setq _bObstacleWasSeen (round __bObstacleWasSeen))
   (setq _iLeftEdgeInImageX (round __iLeftEdgeInImageX))
   (setq _iLeftEdgeInImageY (round __iLeftEdgeInImageY))
   (setq _iRightEdgeInImageX (round __iRightEdgeInImageX))
   (setq _iRightEdgeInImageY (round __iRightEdgeInImageY))
   (setq _iHeightInImage (round __iHeightInImage))
   (setq _inumber (round __inumber))
   (setq _iOthersLeftEdgeInImageX __iOthersLeftEdgeInImageX)
   (setq _iOthersRightEdgeInImageX __iOthersRightEdgeInImageX)
   (setq _iOthersInImageY __iOthersInImageY)
   (setq _iOthersHeightInImage __iOthersHeightInImage)
   self)
  (:bObstacleWasSeen
   (&optional __bObstacleWasSeen)
   (if __bObstacleWasSeen (setq _bObstacleWasSeen __bObstacleWasSeen)) _bObstacleWasSeen)
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
  (:inumber
   (&optional __inumber)
   (if __inumber (setq _inumber __inumber)) _inumber)
  (:iOthersLeftEdgeInImageX
   (&optional __iOthersLeftEdgeInImageX)
   (if __iOthersLeftEdgeInImageX (setq _iOthersLeftEdgeInImageX __iOthersLeftEdgeInImageX)) _iOthersLeftEdgeInImageX)
  (:iOthersRightEdgeInImageX
   (&optional __iOthersRightEdgeInImageX)
   (if __iOthersRightEdgeInImageX (setq _iOthersRightEdgeInImageX __iOthersRightEdgeInImageX)) _iOthersRightEdgeInImageX)
  (:iOthersInImageY
   (&optional __iOthersInImageY)
   (if __iOthersInImageY (setq _iOthersInImageY __iOthersInImageY)) _iOthersInImageY)
  (:iOthersHeightInImage
   (&optional __iOthersHeightInImage)
   (if __iOthersHeightInImage (setq _iOthersHeightInImage __iOthersHeightInImage)) _iOthersHeightInImage)
  (:serialization-length
   ()
   (+
    ;; uint8 _bObstacleWasSeen
    1
    ;; uint16 _iLeftEdgeInImageX
    2
    ;; uint16 _iLeftEdgeInImageY
    2
    ;; uint16 _iRightEdgeInImageX
    2
    ;; uint16 _iRightEdgeInImageY
    2
    ;; uint16 _iHeightInImage
    2
    ;; uint16 _inumber
    2
    ;; uint16[] _iOthersLeftEdgeInImageX
    (* 2    (length _iOthersLeftEdgeInImageX)) 4
    ;; uint16[] _iOthersRightEdgeInImageX
    (* 2    (length _iOthersRightEdgeInImageX)) 4
    ;; uint16[] _iOthersInImageY
    (* 2    (length _iOthersInImageY)) 4
    ;; uint16[] _iOthersHeightInImage
    (* 2    (length _iOthersHeightInImage)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8 _bObstacleWasSeen
       (write-byte _bObstacleWasSeen s)
     ;; uint16 _iLeftEdgeInImageX
       (write-word _iLeftEdgeInImageX s)
     ;; uint16 _iLeftEdgeInImageY
       (write-word _iLeftEdgeInImageY s)
     ;; uint16 _iRightEdgeInImageX
       (write-word _iRightEdgeInImageX s)
     ;; uint16 _iRightEdgeInImageY
       (write-word _iRightEdgeInImageY s)
     ;; uint16 _iHeightInImage
       (write-word _iHeightInImage s)
     ;; uint16 _inumber
       (write-word _inumber s)
     ;; uint16[] _iOthersLeftEdgeInImageX
     (write-long (length _iOthersLeftEdgeInImageX) s)
     (dotimes (i (length _iOthersLeftEdgeInImageX))
       (write-word (elt _iOthersLeftEdgeInImageX i) s)
       )
     ;; uint16[] _iOthersRightEdgeInImageX
     (write-long (length _iOthersRightEdgeInImageX) s)
     (dotimes (i (length _iOthersRightEdgeInImageX))
       (write-word (elt _iOthersRightEdgeInImageX i) s)
       )
     ;; uint16[] _iOthersInImageY
     (write-long (length _iOthersInImageY) s)
     (dotimes (i (length _iOthersInImageY))
       (write-word (elt _iOthersInImageY i) s)
       )
     ;; uint16[] _iOthersHeightInImage
     (write-long (length _iOthersHeightInImage) s)
     (dotimes (i (length _iOthersHeightInImage))
       (write-word (elt _iOthersHeightInImage i) s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8 _bObstacleWasSeen
     (setq _bObstacleWasSeen (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint16 _iLeftEdgeInImageX
     (setq _iLeftEdgeInImageX (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint16 _iLeftEdgeInImageY
     (setq _iLeftEdgeInImageY (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint16 _iRightEdgeInImageX
     (setq _iRightEdgeInImageX (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint16 _iRightEdgeInImageY
     (setq _iRightEdgeInImageY (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint16 _iHeightInImage
     (setq _iHeightInImage (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint16 _inumber
     (setq _inumber (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint16[] _iOthersLeftEdgeInImageX
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _iOthersLeftEdgeInImageX (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _iOthersLeftEdgeInImageX i) (sys::peek buf ptr- :short)) (incf ptr- 2)
     ))
   ;; uint16[] _iOthersRightEdgeInImageX
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _iOthersRightEdgeInImageX (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _iOthersRightEdgeInImageX i) (sys::peek buf ptr- :short)) (incf ptr- 2)
     ))
   ;; uint16[] _iOthersInImageY
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _iOthersInImageY (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _iOthersInImageY i) (sys::peek buf ptr- :short)) (incf ptr- 2)
     ))
   ;; uint16[] _iOthersHeightInImage
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _iOthersHeightInImage (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _iOthersHeightInImage i) (sys::peek buf ptr- :short)) (incf ptr- 2)
     ))
   ;;
   self)
  )

(setf (get vision::Obstacle :md5sum-) "b7ab1788ead7712018d8f4217af3a9ee")
(setf (get vision::Obstacle :datatype-) "vision/Obstacle")
(setf (get vision::Obstacle :definition-)
      "uint8 bObstacleWasSeen

uint16 iLeftEdgeInImageX
uint16 iLeftEdgeInImageY
uint16 iRightEdgeInImageX
uint16 iRightEdgeInImageY
uint16 iHeightInImage

uint16 inumber # apart from first obstacle, if there were 2 obstacles found, this is 1
uint16[] iOthersLeftEdgeInImageX
uint16[] iOthersRightEdgeInImageX
uint16[] iOthersInImageY  # the same Y for both left and right
uint16[] iOthersHeightInImage

")



(provide :vision/Obstacle "b7ab1788ead7712018d8f4217af3a9ee")


