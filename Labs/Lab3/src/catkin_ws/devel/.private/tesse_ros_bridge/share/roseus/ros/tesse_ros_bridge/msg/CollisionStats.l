;; Auto-generated. Do not edit!


(when (boundp 'tesse_ros_bridge::CollisionStats)
  (if (not (find-package "TESSE_ROS_BRIDGE"))
    (make-package "TESSE_ROS_BRIDGE"))
  (shadow 'CollisionStats (find-package "TESSE_ROS_BRIDGE")))
(unless (find-package "TESSE_ROS_BRIDGE::COLLISIONSTATS")
  (make-package "TESSE_ROS_BRIDGE::COLLISIONSTATS"))

(in-package "ROS")
;;//! \htmlinclude CollisionStats.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass tesse_ros_bridge::CollisionStats
  :super ros::object
  :slots (_header _is_collision _object_name ))

(defmethod tesse_ros_bridge::CollisionStats
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:is_collision __is_collision) nil)
    ((:object_name __object_name) "")
    )
   (send-super :init)
   (setq _header __header)
   (setq _is_collision __is_collision)
   (setq _object_name (string __object_name))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:is_collision
   (&optional (__is_collision :null))
   (if (not (eq __is_collision :null)) (setq _is_collision __is_collision)) _is_collision)
  (:object_name
   (&optional __object_name)
   (if __object_name (setq _object_name __object_name)) _object_name)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; bool _is_collision
    1
    ;; string _object_name
    4 (length _object_name)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; bool _is_collision
       (if _is_collision (write-byte -1 s) (write-byte 0 s))
     ;; string _object_name
       (write-long (length _object_name) s) (princ _object_name s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; bool _is_collision
     (setq _is_collision (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; string _object_name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _object_name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get tesse_ros_bridge::CollisionStats :md5sum-) "c6cf75271bbaaafc8a8e7fb4896411d3")
(setf (get tesse_ros_bridge::CollisionStats :datatype-) "tesse_ros_bridge/CollisionStats")
(setf (get tesse_ros_bridge::CollisionStats :definition-)
      "# Collision Status and Statistics Message
# if `is_collision` is True, then `object_name` will represent the object
#   with which the agent is colliding.

Header header

bool is_collision
string object_name

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

")



(provide :tesse_ros_bridge/CollisionStats "c6cf75271bbaaafc8a8e7fb4896411d3")


