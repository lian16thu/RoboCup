
(cl:in-package :asdf)

(defsystem "head_motion-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "head_pose" :depends-on ("_package_head_pose"))
    (:file "_package_head_pose" :depends-on ("_package"))
    (:file "head_servo_angel" :depends-on ("_package_head_servo_angel"))
    (:file "_package_head_servo_angel" :depends-on ("_package"))
  ))