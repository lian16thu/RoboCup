
(cl:in-package :asdf)

(defsystem "decision-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "Ball" :depends-on ("_package_Ball"))
    (:file "_package_Ball" :depends-on ("_package"))
    (:file "GoalData" :depends-on ("_package_GoalData"))
    (:file "_package_GoalData" :depends-on ("_package"))
    (:file "Obstacle" :depends-on ("_package_Obstacle"))
    (:file "_package_Obstacle" :depends-on ("_package"))
    (:file "OutputData" :depends-on ("_package_OutputData"))
    (:file "_package_OutputData" :depends-on ("_package"))
    (:file "SerialReceived" :depends-on ("_package_SerialReceived"))
    (:file "_package_SerialReceived" :depends-on ("_package"))
    (:file "UDPReceived" :depends-on ("_package_UDPReceived"))
    (:file "_package_UDPReceived" :depends-on ("_package"))
    (:file "gameControl" :depends-on ("_package_gameControl"))
    (:file "_package_gameControl" :depends-on ("_package"))
    (:file "gyro_euler" :depends-on ("_package_gyro_euler"))
    (:file "_package_gyro_euler" :depends-on ("_package"))
    (:file "head_angle" :depends-on ("_package_head_angle"))
    (:file "_package_head_angle" :depends-on ("_package"))
  ))