
(cl:in-package :asdf)

(defsystem "kobra_plugins-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "ptz_msg" :depends-on ("_package_ptz_msg"))
    (:file "_package_ptz_msg" :depends-on ("_package"))
  ))