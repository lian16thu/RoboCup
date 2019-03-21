;; Auto-generated. Do not edit!


(when (boundp 'localization::WorldObjects)
  (if (not (find-package "LOCALIZATION"))
    (make-package "LOCALIZATION"))
  (shadow 'WorldObjects (find-package "LOCALIZATION")))
(unless (find-package "LOCALIZATION::WORLDOBJECTS")
  (make-package "LOCALIZATION::WORLDOBJECTS"))

(in-package "ROS")
;;//! \htmlinclude WorldObjects.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass localization::WorldObjects
  :super ros::object
  :slots (_header _objects _goalposts _lines _obstacles ))

(defmethod localization::WorldObjects
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:objects __objects) (let (r) (dotimes (i 0) (push (instance localization::ObjectsDetected :init) r)) r))
    ((:goalposts __goalposts) (let (r) (dotimes (i 0) (push (instance localization::GoalpostsDetected :init) r)) r))
    ((:lines __lines) (let (r) (dotimes (i 0) (push (instance localization::LinesDetected :init) r)) r))
    ((:obstacles __obstacles) (let (r) (dotimes (i 0) (push (instance localization::ObstaclesDetected :init) r)) r))
    )
   (send-super :init)
   (setq _header __header)
   (setq _objects __objects)
   (setq _goalposts __goalposts)
   (setq _lines __lines)
   (setq _obstacles __obstacles)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:objects
   (&rest __objects)
   (if (keywordp (car __objects))
       (send* _objects __objects)
     (progn
       (if __objects (setq _objects (car __objects)))
       _objects)))
  (:goalposts
   (&rest __goalposts)
   (if (keywordp (car __goalposts))
       (send* _goalposts __goalposts)
     (progn
       (if __goalposts (setq _goalposts (car __goalposts)))
       _goalposts)))
  (:lines
   (&rest __lines)
   (if (keywordp (car __lines))
       (send* _lines __lines)
     (progn
       (if __lines (setq _lines (car __lines)))
       _lines)))
  (:obstacles
   (&rest __obstacles)
   (if (keywordp (car __obstacles))
       (send* _obstacles __obstacles)
     (progn
       (if __obstacles (setq _obstacles (car __obstacles)))
       _obstacles)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; localization/ObjectsDetected[] _objects
    (apply #'+ (send-all _objects :serialization-length)) 4
    ;; localization/GoalpostsDetected[] _goalposts
    (apply #'+ (send-all _goalposts :serialization-length)) 4
    ;; localization/LinesDetected[] _lines
    (apply #'+ (send-all _lines :serialization-length)) 4
    ;; localization/ObstaclesDetected[] _obstacles
    (apply #'+ (send-all _obstacles :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; localization/ObjectsDetected[] _objects
     (write-long (length _objects) s)
     (dolist (elem _objects)
       (send elem :serialize s)
       )
     ;; localization/GoalpostsDetected[] _goalposts
     (write-long (length _goalposts) s)
     (dolist (elem _goalposts)
       (send elem :serialize s)
       )
     ;; localization/LinesDetected[] _lines
     (write-long (length _lines) s)
     (dolist (elem _lines)
       (send elem :serialize s)
       )
     ;; localization/ObstaclesDetected[] _obstacles
     (write-long (length _obstacles) s)
     (dolist (elem _obstacles)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; localization/ObjectsDetected[] _objects
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _objects (let (r) (dotimes (i n) (push (instance localization::ObjectsDetected :init) r)) r))
     (dolist (elem- _objects)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; localization/GoalpostsDetected[] _goalposts
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _goalposts (let (r) (dotimes (i n) (push (instance localization::GoalpostsDetected :init) r)) r))
     (dolist (elem- _goalposts)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; localization/LinesDetected[] _lines
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _lines (let (r) (dotimes (i n) (push (instance localization::LinesDetected :init) r)) r))
     (dolist (elem- _lines)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; localization/ObstaclesDetected[] _obstacles
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _obstacles (let (r) (dotimes (i n) (push (instance localization::ObstaclesDetected :init) r)) r))
     (dolist (elem- _obstacles)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get localization::WorldObjects :md5sum-) "b3b402e0a736a3c111358cf72848e29c")
(setf (get localization::WorldObjects :datatype-) "localization/WorldObjects")
(setf (get localization::WorldObjects :definition-)
      "Header header
#add more for the lines and such, like in detections
ObjectsDetected[] objects
GoalpostsDetected[] goalposts
LinesDetected[] lines
ObstaclesDetected[] obstacles



================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
# 0: no frame
# 1: global frame
string frame_id

================================================================================
MSG: localization/ObjectsDetected
geometry_msgs/Pose2D pose        # Pose
uint8 type                       # Type (see field_model::WorldObject::Type)
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

================================================================================
MSG: localization/GoalpostsDetected
geometry_msgs/Pose2D pose        # Pose
uint8 type                       # Type (see field_model::WorldObject::Type)
float32 confidence               # confidence 0..1

================================================================================
MSG: localization/LinesDetected
float32 x1
float32 y1
float32 x2
float32 y2

================================================================================
MSG: localization/ObstaclesDetected
geometry_msgs/Pose2D pose        # Pose
uint8 type                       # Type 0 for opponent, 1 for other obstacle
float32 confidence               # confidence 0..1

")



(provide :localization/WorldObjects "b3b402e0a736a3c111358cf72848e29c")


