
(cl:in-package :asdf)

(defsystem "gamecontroller-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "gameControl" :depends-on ("_package_gameControl"))
    (:file "_package_gameControl" :depends-on ("_package"))
  ))