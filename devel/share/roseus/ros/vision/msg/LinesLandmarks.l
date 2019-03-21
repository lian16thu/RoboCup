;; Auto-generated. Do not edit!


(when (boundp 'vision::LinesLandmarks)
  (if (not (find-package "VISION"))
    (make-package "VISION"))
  (shadow 'LinesLandmarks (find-package "VISION")))
(unless (find-package "VISION::LINESLANDMARKS")
  (make-package "VISION::LINESLANDMARKS"))

(in-package "ROS")
;;//! \htmlinclude LinesLandmarks.msg.html


(defclass vision::LinesLandmarks
  :super ros::object
  :slots (_lines _landmarks ))

(defmethod vision::LinesLandmarks
  (:init
   (&key
    ((:lines __lines) (let (r) (dotimes (i 0) (push (instance vision::Line :init) r)) r))
    ((:landmarks __landmarks) (let (r) (dotimes (i 0) (push (instance vision::Landmark :init) r)) r))
    )
   (send-super :init)
   (setq _lines __lines)
   (setq _landmarks __landmarks)
   self)
  (:lines
   (&rest __lines)
   (if (keywordp (car __lines))
       (send* _lines __lines)
     (progn
       (if __lines (setq _lines (car __lines)))
       _lines)))
  (:landmarks
   (&rest __landmarks)
   (if (keywordp (car __landmarks))
       (send* _landmarks __landmarks)
     (progn
       (if __landmarks (setq _landmarks (car __landmarks)))
       _landmarks)))
  (:serialization-length
   ()
   (+
    ;; vision/Line[] _lines
    (apply #'+ (send-all _lines :serialization-length)) 4
    ;; vision/Landmark[] _landmarks
    (apply #'+ (send-all _landmarks :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; vision/Line[] _lines
     (write-long (length _lines) s)
     (dolist (elem _lines)
       (send elem :serialize s)
       )
     ;; vision/Landmark[] _landmarks
     (write-long (length _landmarks) s)
     (dolist (elem _landmarks)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; vision/Line[] _lines
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _lines (let (r) (dotimes (i n) (push (instance vision::Line :init) r)) r))
     (dolist (elem- _lines)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; vision/Landmark[] _landmarks
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _landmarks (let (r) (dotimes (i n) (push (instance vision::Landmark :init) r)) r))
     (dolist (elem- _landmarks)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get vision::LinesLandmarks :md5sum-) "fb66f810489cc88ff8dd15c871c9425d")
(setf (get vision::LinesLandmarks :datatype-) "vision/LinesLandmarks")
(setf (get vision::LinesLandmarks :definition-)
      "Line[] lines
Landmark[] landmarks

================================================================================
MSG: vision/Line
float32 x1 #line start
float32 y1
float32 x2 #line end
float32 y2

================================================================================
MSG: vision/Landmark
geometry_msgs/Pose2D pose        # Pose
uint8 type                       # Type (see localization::field_model::WorldObject::Type)
float32 confidence               # confidence 0..1

================================================================================
MSG: geometry_msgs/Pose2D
# Deprecated
# Please use the full 3D pose.

# In general our recommendation is to use a full 3D representation of everything and for 2D specific applications make the appropriate projections into the plane for their calculations but optimally will preserve the 3D information during processing.

# If we have parallel copies of 2D datatypes every UI and other pipeline will end up needing to have dual interfaces to plot everything. And you will end up with not being able to use 3D tools for 2D use cases even if they're completely valid, as you'd have to reimplement it with different inputs and outputs. It's not particularly hard to plot the 2D pose or compute the yaw error for the Pose message and there are already tools and libraries that can do this for you.


# This expresses a position and orientation on a 2D manifold.

float64 x
float64 y
float64 theta

")



(provide :vision/LinesLandmarks "fb66f810489cc88ff8dd15c871c9425d")


