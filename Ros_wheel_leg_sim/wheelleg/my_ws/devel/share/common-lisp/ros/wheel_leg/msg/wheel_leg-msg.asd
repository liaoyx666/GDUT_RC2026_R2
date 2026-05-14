
(cl:in-package :asdf)

(defsystem "wheel_leg-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "wl_control" :depends-on ("_package_wl_control"))
    (:file "_package_wl_control" :depends-on ("_package"))
  ))