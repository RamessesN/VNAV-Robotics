; Auto-generated. Do not edit!


(cl:in-package tesse_ros_bridge-srv)


;//! \htmlinclude ObjectSpawnRequestService-request.msg.html

(cl:defclass <ObjectSpawnRequestService-request> (roslisp-msg-protocol:ros-message)
  ((id
    :reader id
    :initarg :id
    :type cl:fixnum
    :initform 0)
   (pose
    :reader pose
    :initarg :pose
    :type geometry_msgs-msg:Pose
    :initform (cl:make-instance 'geometry_msgs-msg:Pose))
   (params
    :reader params
    :initarg :params
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass ObjectSpawnRequestService-request (<ObjectSpawnRequestService-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ObjectSpawnRequestService-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ObjectSpawnRequestService-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name tesse_ros_bridge-srv:<ObjectSpawnRequestService-request> is deprecated: use tesse_ros_bridge-srv:ObjectSpawnRequestService-request instead.")))

(cl:ensure-generic-function 'id-val :lambda-list '(m))
(cl:defmethod id-val ((m <ObjectSpawnRequestService-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tesse_ros_bridge-srv:id-val is deprecated.  Use tesse_ros_bridge-srv:id instead.")
  (id m))

(cl:ensure-generic-function 'pose-val :lambda-list '(m))
(cl:defmethod pose-val ((m <ObjectSpawnRequestService-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tesse_ros_bridge-srv:pose-val is deprecated.  Use tesse_ros_bridge-srv:pose instead.")
  (pose m))

(cl:ensure-generic-function 'params-val :lambda-list '(m))
(cl:defmethod params-val ((m <ObjectSpawnRequestService-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tesse_ros_bridge-srv:params-val is deprecated.  Use tesse_ros_bridge-srv:params instead.")
  (params m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ObjectSpawnRequestService-request>) ostream)
  "Serializes a message object of type '<ObjectSpawnRequestService-request>"
  (cl:let* ((signed (cl:slot-value msg 'id)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'pose) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'params))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'params))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ObjectSpawnRequestService-request>) istream)
  "Deserializes a message object of type '<ObjectSpawnRequestService-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'id) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'pose) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'params) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'params)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ObjectSpawnRequestService-request>)))
  "Returns string type for a service object of type '<ObjectSpawnRequestService-request>"
  "tesse_ros_bridge/ObjectSpawnRequestServiceRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ObjectSpawnRequestService-request)))
  "Returns string type for a service object of type 'ObjectSpawnRequestService-request"
  "tesse_ros_bridge/ObjectSpawnRequestServiceRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ObjectSpawnRequestService-request>)))
  "Returns md5sum for a message object of type '<ObjectSpawnRequestService-request>"
  "fccdcc49ae24f177c566e20905c89987")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ObjectSpawnRequestService-request)))
  "Returns md5sum for a message object of type 'ObjectSpawnRequestService-request"
  "fccdcc49ae24f177c566e20905c89987")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ObjectSpawnRequestService-request>)))
  "Returns full string definition for message of type '<ObjectSpawnRequestService-request>"
  (cl:format cl:nil "## Object Spawn Request Service~%~%# Request fields~%int8 id  # integer id of object class~%geometry_msgs/Pose pose~%float64[] params~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ObjectSpawnRequestService-request)))
  "Returns full string definition for message of type 'ObjectSpawnRequestService-request"
  (cl:format cl:nil "## Object Spawn Request Service~%~%# Request fields~%int8 id  # integer id of object class~%geometry_msgs/Pose pose~%float64[] params~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ObjectSpawnRequestService-request>))
  (cl:+ 0
     1
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'pose))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'params) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ObjectSpawnRequestService-request>))
  "Converts a ROS message object to a list"
  (cl:list 'ObjectSpawnRequestService-request
    (cl:cons ':id (id msg))
    (cl:cons ':pose (pose msg))
    (cl:cons ':params (params msg))
))
;//! \htmlinclude ObjectSpawnRequestService-response.msg.html

(cl:defclass <ObjectSpawnRequestService-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass ObjectSpawnRequestService-response (<ObjectSpawnRequestService-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ObjectSpawnRequestService-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ObjectSpawnRequestService-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name tesse_ros_bridge-srv:<ObjectSpawnRequestService-response> is deprecated: use tesse_ros_bridge-srv:ObjectSpawnRequestService-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <ObjectSpawnRequestService-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tesse_ros_bridge-srv:success-val is deprecated.  Use tesse_ros_bridge-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ObjectSpawnRequestService-response>) ostream)
  "Serializes a message object of type '<ObjectSpawnRequestService-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ObjectSpawnRequestService-response>) istream)
  "Deserializes a message object of type '<ObjectSpawnRequestService-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ObjectSpawnRequestService-response>)))
  "Returns string type for a service object of type '<ObjectSpawnRequestService-response>"
  "tesse_ros_bridge/ObjectSpawnRequestServiceResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ObjectSpawnRequestService-response)))
  "Returns string type for a service object of type 'ObjectSpawnRequestService-response"
  "tesse_ros_bridge/ObjectSpawnRequestServiceResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ObjectSpawnRequestService-response>)))
  "Returns md5sum for a message object of type '<ObjectSpawnRequestService-response>"
  "fccdcc49ae24f177c566e20905c89987")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ObjectSpawnRequestService-response)))
  "Returns md5sum for a message object of type 'ObjectSpawnRequestService-response"
  "fccdcc49ae24f177c566e20905c89987")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ObjectSpawnRequestService-response>)))
  "Returns full string definition for message of type '<ObjectSpawnRequestService-response>"
  (cl:format cl:nil "~%# Response fields~%bool success  # true if object spawns without exception, false otw~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ObjectSpawnRequestService-response)))
  "Returns full string definition for message of type 'ObjectSpawnRequestService-response"
  (cl:format cl:nil "~%# Response fields~%bool success  # true if object spawns without exception, false otw~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ObjectSpawnRequestService-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ObjectSpawnRequestService-response>))
  "Converts a ROS message object to a list"
  (cl:list 'ObjectSpawnRequestService-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'ObjectSpawnRequestService)))
  'ObjectSpawnRequestService-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'ObjectSpawnRequestService)))
  'ObjectSpawnRequestService-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ObjectSpawnRequestService)))
  "Returns string type for a service object of type '<ObjectSpawnRequestService>"
  "tesse_ros_bridge/ObjectSpawnRequestService")