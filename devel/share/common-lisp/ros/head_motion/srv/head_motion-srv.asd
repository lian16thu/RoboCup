
(cl:in-package :asdf)

(defsystem "head_motion-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "head_control" :depends-on ("_package_head_control"))
    (:file "_package_head_control" :depends-on ("_package"))
  ))