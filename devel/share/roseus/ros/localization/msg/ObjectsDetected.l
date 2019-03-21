;; Auto-generated. Do not edit!


(when (boundp 'localization::ObjectsDetected)
  (if (not (find-package "LOCALIZATION"))
    (make-package "LOCALIZATION"))
  (shadow 'ObjectsDetected (find-package "LOCALIZATION")))
(unless (find-package "LOCALIZATION::OBJECTSDETECTED")
  (make-package "LOCALIZATION::OBJECTSDETECTED"))

(in-package "ROS")
;;//! \htmlinclude ObjectsDetected.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass localization::ObjectsDetected
  :super ros::object
  :slots (_pose _type _confidence ))

(defmethod localization::ObjectsDetected
  (:init
   (&key
    ((:pose __pose) (instance geometry_msgs::Pose2D :init))
    ((:type __type) 0)
    ((:confidence __confidence) 0.0)
    )
   (send-super :init)
   (setq _pose __pose)
   (setq _type (round __type))
   (setq _confidence (float __confidence))
   self)
  (:pose
   (&rest __pose)
   (if (keywordp (car __pose))
       (send* _pose __pose)
     (progn
       (if __pose (setq _pose (car __pose)))
       _pose)))
  (:type
   (&optional __type)
   (if __type (setq _type __type)) _type)
  (:confidence
   (&optional __confidence)
   (if __confidence (setq _confidence __confidence)) _confidence)
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/Pose2D _pose
    (send _pose :serialization-length)
    ;; uint8 _type
    1
    ;; float32 _confidence
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/Pose2D _pose
       (send _pose :serialize s)
     ;; uint8 _type
       (write-byte _type s)
     ;; float32 _confidence
       (sys::poke _confidence (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/Pose2D _pose
     (send _pose :deserialize buf ptr-) (incf ptr- (send _pose :serialization-length))
   ;; uint8 _type
     (setq _type (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; float32 _confidence
     (setq _confidence (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get localization::ObjectsDetected :md5sum-) "45117ab335e578bb32c6e4a315f61f4d")
(setf (get localization::ObjectsDetected :datatype-) "localization/ObjectsDetected")
(setf (get localization::ObjectsDetected :definition-)
      "geometry_msgs/Pose2D pose        # Pose
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

")



(provide :localization/ObjectsDetected "45117ab335e578bb32c6e4a315f61f4d")


