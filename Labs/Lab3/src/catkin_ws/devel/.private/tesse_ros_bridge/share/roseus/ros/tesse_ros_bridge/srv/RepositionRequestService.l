;; Auto-generated. Do not edit!


(when (boundp 'tesse_ros_bridge::RepositionRequestService)
  (if (not (find-package "TESSE_ROS_BRIDGE"))
    (make-package "TESSE_ROS_BRIDGE"))
  (shadow 'RepositionRequestService (find-package "TESSE_ROS_BRIDGE")))
(unless (find-package "TESSE_ROS_BRIDGE::REPOSITIONREQUESTSERVICE")
  (make-package "TESSE_ROS_BRIDGE::REPOSITIONREQUESTSERVICE"))
(unless (find-package "TESSE_ROS_BRIDGE::REPOSITIONREQUESTSERVICEREQUEST")
  (make-package "TESSE_ROS_BRIDGE::REPOSITIONREQUESTSERVICEREQUEST"))
(unless (find-package "TESSE_ROS_BRIDGE::REPOSITIONREQUESTSERVICERESPONSE")
  (make-package "TESSE_ROS_BRIDGE::REPOSITIONREQUESTSERVICERESPONSE"))

(in-package "ROS")

(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))




(defclass tesse_ros_bridge::RepositionRequestServiceRequest
  :super ros::object
  :slots (_pose ))

(defmethod tesse_ros_bridge::RepositionRequestServiceRequest
  (:init
   (&key
    ((:pose __pose) (instance geometry_msgs::Pose :init))
    )
   (send-super :init)
   (setq _pose __pose)
   self)
  (:pose
   (&rest __pose)
   (if (keywordp (car __pose))
       (send* _pose __pose)
     (progn
       (if __pose (setq _pose (car __pose)))
       _pose)))
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/Pose _pose
    (send _pose :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/Pose _pose
       (send _pose :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/Pose _pose
     (send _pose :deserialize buf ptr-) (incf ptr- (send _pose :serialization-length))
   ;;
   self)
  )

(defclass tesse_ros_bridge::RepositionRequestServiceResponse
  :super ros::object
  :slots (_success ))

(defmethod tesse_ros_bridge::RepositionRequestServiceResponse
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

(defclass tesse_ros_bridge::RepositionRequestService
  :super ros::object
  :slots ())

(setf (get tesse_ros_bridge::RepositionRequestService :md5sum-) "b66f4c8c2fa81b98544cf0f3e10950ee")
(setf (get tesse_ros_bridge::RepositionRequestService :datatype-) "tesse_ros_bridge/RepositionRequestService")
(setf (get tesse_ros_bridge::RepositionRequestService :request) tesse_ros_bridge::RepositionRequestServiceRequest)
(setf (get tesse_ros_bridge::RepositionRequestService :response) tesse_ros_bridge::RepositionRequestServiceResponse)

(defmethod tesse_ros_bridge::RepositionRequestServiceRequest
  (:response () (instance tesse_ros_bridge::RepositionRequestServiceResponse :init)))

(setf (get tesse_ros_bridge::RepositionRequestServiceRequest :md5sum-) "b66f4c8c2fa81b98544cf0f3e10950ee")
(setf (get tesse_ros_bridge::RepositionRequestServiceRequest :datatype-) "tesse_ros_bridge/RepositionRequestServiceRequest")
(setf (get tesse_ros_bridge::RepositionRequestServiceRequest :definition-)
      "## Reposition Request Service

# Request fields
geometry_msgs/Pose pose

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
---

# Response fields
bool success  # true if the reposition happens without exception, false otw
")

(setf (get tesse_ros_bridge::RepositionRequestServiceResponse :md5sum-) "b66f4c8c2fa81b98544cf0f3e10950ee")
(setf (get tesse_ros_bridge::RepositionRequestServiceResponse :datatype-) "tesse_ros_bridge/RepositionRequestServiceResponse")
(setf (get tesse_ros_bridge::RepositionRequestServiceResponse :definition-)
      "## Reposition Request Service

# Request fields
geometry_msgs/Pose pose

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
---

# Response fields
bool success  # true if the reposition happens without exception, false otw
")



(provide :tesse_ros_bridge/RepositionRequestService "b66f4c8c2fa81b98544cf0f3e10950ee")


