
(cl:in-package :asdf)

(defsystem "tesse_ros_bridge-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
)
  :components ((:file "_package")
    (:file "ObjectSpawnRequestService" :depends-on ("_package_ObjectSpawnRequestService"))
    (:file "_package_ObjectSpawnRequestService" :depends-on ("_package"))
    (:file "RepositionRequestService" :depends-on ("_package_RepositionRequestService"))
    (:file "_package_RepositionRequestService" :depends-on ("_package"))
    (:file "SceneRequestService" :depends-on ("_package_SceneRequestService"))
    (:file "_package_SceneRequestService" :depends-on ("_package"))
  ))