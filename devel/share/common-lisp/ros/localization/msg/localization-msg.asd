
(cl:in-package :asdf)

(defsystem "localization-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "GoalpostsDetected" :depends-on ("_package_GoalpostsDetected"))
    (:file "_package_GoalpostsDetected" :depends-on ("_package"))
    (:file "LinesDetected" :depends-on ("_package_LinesDetected"))
    (:file "_package_LinesDetected" :depends-on ("_package"))
    (:file "MeanPoseConfStamped" :depends-on ("_package_MeanPoseConfStamped"))
    (:file "_package_MeanPoseConfStamped" :depends-on ("_package"))
    (:file "ObjectsDetected" :depends-on ("_package_ObjectsDetected"))
    (:file "_package_ObjectsDetected" :depends-on ("_package"))
    (:file "ObstaclesDetected" :depends-on ("_package_ObstaclesDetected"))
    (:file "_package_ObstaclesDetected" :depends-on ("_package"))
    (:file "OutputData" :depends-on ("_package_OutputData"))
    (:file "_package_OutputData" :depends-on ("_package"))
    (:file "Particle" :depends-on ("_package_Particle"))
    (:file "_package_Particle" :depends-on ("_package"))
    (:file "ParticleSet" :depends-on ("_package_ParticleSet"))
    (:file "_package_ParticleSet" :depends-on ("_package"))
    (:file "WorldObjects" :depends-on ("_package_WorldObjects"))
    (:file "_package_WorldObjects" :depends-on ("_package"))
  ))