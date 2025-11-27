; Auto-generated. Do not edit!


(cl:in-package tesse_ros_bridge-msg)


;//! \htmlinclude CollisionStats.msg.html

(cl:defclass <CollisionStats> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (is_collision
    :reader is_collision
    :initarg :is_collision
    :type cl:boolean
    :initform cl:nil)
   (object_name
    :reader object_name
    :initarg :object_name
    :type cl:string
    :initform ""))
)

(cl:defclass CollisionStats (<CollisionStats>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CollisionStats>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CollisionStats)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name tesse_ros_bridge-msg:<CollisionStats> is deprecated: use tesse_ros_bridge-msg:CollisionStats instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <CollisionStats>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tesse_ros_bridge-msg:header-val is deprecated.  Use tesse_ros_bridge-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'is_collision-val :lambda-list '(m))
(cl:defmethod is_collision-val ((m <CollisionStats>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tesse_ros_bridge-msg:is_collision-val is deprecated.  Use tesse_ros_bridge-msg:is_collision instead.")
  (is_collision m))

(cl:ensure-generic-function 'object_name-val :lambda-list '(m))
(cl:defmethod object_name-val ((m <CollisionStats>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tesse_ros_bridge-msg:object_name-val is deprecated.  Use tesse_ros_bridge-msg:object_name instead.")
  (object_name m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CollisionStats>) ostream)
  "Serializes a message object of type '<CollisionStats>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'is_collision) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'object_name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'object_name))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CollisionStats>) istream)
  "Deserializes a message object of type '<CollisionStats>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:slot-value msg 'is_collision) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'object_name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'object_name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CollisionStats>)))
  "Returns string type for a message object of type '<CollisionStats>"
  "tesse_ros_bridge/CollisionStats")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CollisionStats)))
  "Returns string type for a message object of type 'CollisionStats"
  "tesse_ros_bridge/CollisionStats")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CollisionStats>)))
  "Returns md5sum for a message object of type '<CollisionStats>"
  "c6cf75271bbaaafc8a8e7fb4896411d3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CollisionStats)))
  "Returns md5sum for a message object of type 'CollisionStats"
  "c6cf75271bbaaafc8a8e7fb4896411d3")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CollisionStats>)))
  "Returns full string definition for message of type '<CollisionStats>"
  (cl:format cl:nil "# Collision Status and Statistics Message~%# if `is_collision` is True, then `object_name` will represent the object~%#   with which the agent is colliding.~%~%Header header~%~%bool is_collision~%string object_name~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CollisionStats)))
  "Returns full string definition for message of type 'CollisionStats"
  (cl:format cl:nil "# Collision Status and Statistics Message~%# if `is_collision` is True, then `object_name` will represent the object~%#   with which the agent is colliding.~%~%Header header~%~%bool is_collision~%string object_name~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CollisionStats>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
     4 (cl:length (cl:slot-value msg 'object_name))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CollisionStats>))
  "Converts a ROS message object to a list"
  (cl:list 'CollisionStats
    (cl:cons ':header (header msg))
    (cl:cons ':is_collision (is_collision msg))
    (cl:cons ':object_name (object_name msg))
))
