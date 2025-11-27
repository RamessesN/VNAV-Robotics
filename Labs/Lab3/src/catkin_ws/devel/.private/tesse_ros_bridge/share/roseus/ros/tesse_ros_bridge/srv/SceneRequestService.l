;; Auto-generated. Do not edit!


(when (boundp 'tesse_ros_bridge::SceneRequestService)
  (if (not (find-package "TESSE_ROS_BRIDGE"))
    (make-package "TESSE_ROS_BRIDGE"))
  (shadow 'SceneRequestService (find-package "TESSE_ROS_BRIDGE")))
(unless (find-package "TESSE_ROS_BRIDGE::SCENEREQUESTSERVICE")
  (make-package "TESSE_ROS_BRIDGE::SCENEREQUESTSERVICE"))
(unless (find-package "TESSE_ROS_BRIDGE::SCENEREQUESTSERVICEREQUEST")
  (make-package "TESSE_ROS_BRIDGE::SCENEREQUESTSERVICEREQUEST"))
(unless (find-package "TESSE_ROS_BRIDGE::SCENEREQUESTSERVICERESPONSE")
  (make-package "TESSE_ROS_BRIDGE::SCENEREQUESTSERVICERESPONSE"))

(in-package "ROS")





(defclass tesse_ros_bridge::SceneRequestServiceRequest
  :super ros::object
  :slots (_id ))

(defmethod tesse_ros_bridge::SceneRequestServiceRequest
  (:init
   (&key
    ((:id __id) 0)
    )
   (send-super :init)
   (setq _id (round __id))
   self)
  (:id
   (&optional __id)
   (if __id (setq _id __id)) _id)
  (:serialization-length
   ()
   (+
    ;; int8 _id
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int8 _id
       (write-byte _id s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int8 _id
     (setq _id (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _id 127) (setq _id (- _id 256)))
   ;;
   self)
  )

(defclass tesse_ros_bridge::SceneRequestServiceResponse
  :super ros::object
  :slots (_success ))

(defmethod tesse_ros_bridge::SceneRequestServiceResponse
  (:init
   (&key
    ((:success __success) nil)
    )
   (send-super :init)
   (setq _success __success)
   self)
  (:success
   (&optional (__success :null))
   (if (not (eq __success :null)) (setq _success __success)) _success)
  (:serialization-length
   ()
   (+
    ;; bool _success
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass tesse_ros_bridge::SceneRequestService
  :super ros::object
  :slots ())

(setf (get tesse_ros_bridge::SceneRequestService :md5sum-) "c185ea87dce4170cbb159dbc7c2d24cf")
(setf (get tesse_ros_bridge::SceneRequestService :datatype-) "tesse_ros_bridge/SceneRequestService")
(setf (get tesse_ros_bridge::SceneRequestService :request) tesse_ros_bridge::SceneRequestServiceRequest)
(setf (get tesse_ros_bridge::SceneRequestService :response) tesse_ros_bridge::SceneRequestServiceResponse)

(defmethod tesse_ros_bridge::SceneRequestServiceRequest
  (:response () (instance tesse_ros_bridge::SceneRequestServiceResponse :init)))

(setf (get tesse_ros_bridge::SceneRequestServiceRequest :md5sum-) "c185ea87dce4170cbb159dbc7c2d24cf")
(setf (get tesse_ros_bridge::SceneRequestServiceRequest :datatype-) "tesse_ros_bridge/SceneRequestServiceRequest")
(setf (get tesse_ros_bridge::SceneRequestServiceRequest :definition-)
      "## Scene Change Request Service

# Request fields
int8 id  # integer id of scene request
---

# Response fields
bool success  # true if scene change was completed without exception, false otw

")

(setf (get tesse_ros_bridge::SceneRequestServiceResponse :md5sum-) "c185ea87dce4170cbb159dbc7c2d24cf")
(setf (get tesse_ros_bridge::SceneRequestServiceResponse :datatype-) "tesse_ros_bridge/SceneRequestServiceResponse")
(setf (get tesse_ros_bridge::SceneRequestServiceResponse :definition-)
      "## Scene Change Request Service

# Request fields
int8 id  # integer id of scene request
---

# Response fields
bool success  # true if scene change was completed without exception, false otw

")



(provide :tesse_ros_bridge/SceneRequestService "c185ea87dce4170cbb159dbc7c2d24cf")


