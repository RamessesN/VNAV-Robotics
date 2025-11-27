; Auto-generated. Do not edit!


(cl:in-package tesse_ros_bridge-srv)


;//! \htmlinclude SceneRequestService-request.msg.html

(cl:defclass <SceneRequestService-request> (roslisp-msg-protocol:ros-message)
  ((id
    :reader id
    :initarg :id
    :type cl:fixnum
    :initform 0))
)

(cl:defclass SceneRequestService-request (<SceneRequestService-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SceneRequestService-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SceneRequestService-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name tesse_ros_bridge-srv:<SceneRequestService-request> is deprecated: use tesse_ros_bridge-srv:SceneRequestService-request instead.")))

(cl:ensure-generic-function 'id-val :lambda-list '(m))
(cl:defmethod id-val ((m <SceneRequestService-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tesse_ros_bridge-srv:id-val is deprecated.  Use tesse_ros_bridge-srv:id instead.")
  (id m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SceneRequestService-request>) ostream)
  "Serializes a message object of type '<SceneRequestService-request>"
  (cl:let* ((signed (cl:slot-value msg 'id)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SceneRequestService-request>) istream)
  "Deserializes a message object of type '<SceneRequestService-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'id) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SceneRequestService-request>)))
  "Returns string type for a service object of type '<SceneRequestService-request>"
  "tesse_ros_bridge/SceneRequestServiceRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SceneRequestService-request)))
  "Returns string type for a service object of type 'SceneRequestService-request"
  "tesse_ros_bridge/SceneRequestServiceRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SceneRequestService-request>)))
  "Returns md5sum for a message object of type '<SceneRequestService-request>"
  "c185ea87dce4170cbb159dbc7c2d24cf")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SceneRequestService-request)))
  "Returns md5sum for a message object of type 'SceneRequestService-request"
  "c185ea87dce4170cbb159dbc7c2d24cf")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SceneRequestService-request>)))
  "Returns full string definition for message of type '<SceneRequestService-request>"
  (cl:format cl:nil "## Scene Change Request Service~%~%# Request fields~%int8 id  # integer id of scene request~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SceneRequestService-request)))
  "Returns full string definition for message of type 'SceneRequestService-request"
  (cl:format cl:nil "## Scene Change Request Service~%~%# Request fields~%int8 id  # integer id of scene request~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SceneRequestService-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SceneRequestService-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SceneRequestService-request
    (cl:cons ':id (id msg))
))
;//! \htmlinclude SceneRequestService-response.msg.html

(cl:defclass <SceneRequestService-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass SceneRequestService-response (<SceneRequestService-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SceneRequestService-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SceneRequestService-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name tesse_ros_bridge-srv:<SceneRequestService-response> is deprecated: use tesse_ros_bridge-srv:SceneRequestService-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <SceneRequestService-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tesse_ros_bridge-srv:success-val is deprecated.  Use tesse_ros_bridge-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SceneRequestService-response>) ostream)
  "Serializes a message object of type '<SceneRequestService-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SceneRequestService-response>) istream)
  "Deserializes a message object of type '<SceneRequestService-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SceneRequestService-response>)))
  "Returns string type for a service object of type '<SceneRequestService-response>"
  "tesse_ros_bridge/SceneRequestServiceResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SceneRequestService-response)))
  "Returns string type for a service object of type 'SceneRequestService-response"
  "tesse_ros_bridge/SceneRequestServiceResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SceneRequestService-response>)))
  "Returns md5sum for a message object of type '<SceneRequestService-response>"
  "c185ea87dce4170cbb159dbc7c2d24cf")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SceneRequestService-response)))
  "Returns md5sum for a message object of type 'SceneRequestService-response"
  "c185ea87dce4170cbb159dbc7c2d24cf")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SceneRequestService-response>)))
  "Returns full string definition for message of type '<SceneRequestService-response>"
  (cl:format cl:nil "~%# Response fields~%bool success  # true if scene change was completed without exception, false otw~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SceneRequestService-response)))
  "Returns full string definition for message of type 'SceneRequestService-response"
  (cl:format cl:nil "~%# Response fields~%bool success  # true if scene change was completed without exception, false otw~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SceneRequestService-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SceneRequestService-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SceneRequestService-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SceneRequestService)))
  'SceneRequestService-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SceneRequestService)))
  'SceneRequestService-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SceneRequestService)))
  "Returns string type for a service object of type '<SceneRequestService>"
  "tesse_ros_bridge/SceneRequestService")