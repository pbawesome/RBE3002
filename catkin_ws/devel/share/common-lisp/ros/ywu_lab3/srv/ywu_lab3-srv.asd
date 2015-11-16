
(cl:in-package :asdf)

(defsystem "ywu_lab3-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :nav_msgs-msg
)
  :components ((:file "_package")
    (:file "aStar" :depends-on ("_package_aStar"))
    (:file "_package_aStar" :depends-on ("_package"))
  ))