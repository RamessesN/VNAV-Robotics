
(cl:in-package :asdf)

(defsystem "tesse_ros_bridge-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "CollisionStats" :depends-on ("_package_CollisionStats"))
    (:file "_package_CollisionStats" :depends-on ("_package"))
  ))