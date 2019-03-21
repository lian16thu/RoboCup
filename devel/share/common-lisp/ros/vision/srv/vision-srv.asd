
(cl:in-package :asdf)

(defsystem "vision-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "DepthRequest" :depends-on ("_package_DepthRequest"))
    (:file "_package_DepthRequest" :depends-on ("_package"))
  ))