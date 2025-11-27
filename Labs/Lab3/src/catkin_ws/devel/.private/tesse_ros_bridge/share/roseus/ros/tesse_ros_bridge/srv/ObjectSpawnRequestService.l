;; Auto-generated. Do not edit!


(when (boundp 'tesse_ros_bridge::ObjectSpawnRequestService)
  (if (not (find-package "TESSE_ROS_BRIDGE"))
    (make-package "TESSE_ROS_BRIDGE"))
  (shadow 'ObjectSpawnRequestService (find-package "TESSE_ROS_BRIDGE")))
(unless (find-package "TESSE_ROS_BRIDGE::OBJECTSPAWNREQUESTSERVICE")
  (make-package "TESSE_ROS_BRIDGE::OBJECTSPAWNREQUESTSERVICE"))
(unless (find-package "TESSE_ROS_BRIDGE::OBJECTSPAWNREQUESTSERVICEREQUEST")
  (make-package "TESSE_ROS_BRIDGE::OBJECTSPAWNREQUESTSERVICEREQUEST"))
(unless (find-package "TESSE_ROS_BRIDGE::OBJECTSPAWNREQUESTSERVICERESPONSE")
  (make-package "TESSE_ROS_BRIDGE::OBJECTSPAWNREQUESTSERVICERESPONSE"))

(in-package "ROS")

(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))




(defclass tesse_ros_bridge::ObjectSpawnRequestServiceRequest
  :super ros::object
  :slots (_id _pose _params ))

(defmethod tesse_ros_bridge::ObjectSpawnRequestServiceRequest
  (:init
   (&key
    ((:id __id) 0)
    ((:pose __pose) (instance geometry_msgs::Pose :init))
    ((:params __params) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _id (round __id))
   (setq _pose __pose)
   (setq _params __params)
   self)
  (:id
   (&optional __id)
   (if __id (setq _id __id)) _id)
  (:pose
   (&rest __pose)
   (if (keywordp (car __pose))
       (send* _pose __pose)
     (progn
       (if __pose (setq _pose (car __pose)))
       _pose)))
  (:params
   (&optional __params)
   (if __params (setq _params __params)) _params)
  (:serialization-length
   ()
   (+
    ;; int8 _id
    1
    ;; geometry_msgs/Pose _pose
    (send _pose :serialization-length)
    ;; float64[] _params
    (* 8    (length _params)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int8 _id
       (write-byte _id s)
     ;; geometry_msgs/Pose _pose
       (send _pose :serialize s)
     ;; float64[] _params
     (write-long (length _params) s)
     (dotimes (i (length _params))
       (sys::poke (elt _params i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int8 _id
     (setq _id (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _id 127) (setq _id (- _id 256)))
   ;; geometry_msgs/Pose _pose
     (send _pose :deserialize buf ptr-) (incf ptr- (send _pose :serialization-length))
   ;; float64[] _params
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _params (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _params i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;;
   self)
  )

(defclass tesse_ros_bridge::ObjectSpawnRequestServiceResponse
  :super ros::object
  :slots (_success ))

(defmethod tesse_ros_bridge::ObjectSpawnRequestServiceResponse
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

(defclass tesse_ros_bridge::ObjectSpawnRequestService
  :super ros::object
  :slots ())

(setf (get tesse_ros_bridge::ObjectSpawnRequestService :md5sum-) "fccdcc49ae24f177c566e20905c89987")
(setf (get tesse_ros_bridge::ObjectSpawnRequestService :datatype-) "tesse_ros_bridge/ObjectSpawnRequestService")
(setf (get tesse_ros_bridge::ObjectSpawnRequestService :request) tesse_ros_bridge::ObjectSpawnRequestServiceRequest)
(setf (get tesse_ros_bridge::ObjectSpawnRequestService :response) tesse_ros_bridge::ObjectSpawnRequestServiceResponse)

(defmethod tesse_ros_bridge::ObjectSpawnRequestServiceRequest
  (:response () (instance tesse_ros_bridge::ObjectSpawnRequestServiceResponse :init)))

(setf (get tesse_ros_bridge::ObjectSpawnRequestServiceRequest :md5sum-) "fccdcc49ae24f177c566e20905c89987")
(setf (get tesse_ros_bridge::ObjectSpawnRequestServiceRequest :datatype-) "tesse_ros_bridge/ObjectSpawnRequestServiceRequest")
(setf (get tesse_ros_bridge::ObjectSpawnRequestServiceRequest :definition-)
      "## Object Spawn Request Service

# Request fields
int8 id  # integer id of object class
geometry_msgs/Pose pose
float64[] params

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
bool success  # true if object spawns without exception, false otw

")

(setf (get tesse_ros_bridge::ObjectSpawnRequestServiceResponse :md5sum-) "fccdcc49ae24f177c566e20905c89987")
(setf (get tesse_ros_bridge::ObjectSpawnRequestServiceResponse :datatype-) "tesse_ros_bridge/ObjectSpawnRequestServiceResponse")
(setf (get tesse_ros_bridge::ObjectSpawnRequestServiceResponse :definition-)
      "## Object Spawn Request Service

# Request fields
int8 id  # integer id of object class
geometry_msgs/Pose pose
float64[] params

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
bool success  # true if object spawns without exception, false otw

")



(provide :tesse_ros_bridge/ObjectSpawnRequestService "fccdcc49ae24f177c566e20905c89987")


