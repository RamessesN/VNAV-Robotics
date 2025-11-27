; Auto-generated. Do not edit!


(cl:in-package tesse_ros_bridge-srv)


;//! \htmlinclude RepositionRequestService-request.msg.html

(cl:defclass <RepositionRequestService-request> (roslisp-msg-protocol:ros-message)
  ((pose
    :reader pose
    :initarg :pose
    :type geometry_msgs-msg:Pose
    :initform (cl:make-instance 'geometry_msgs-msg:Pose)))
)

(cl:defclass RepositionRequestService-request (<RepositionRequestService-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RepositionRequestService-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RepositionRequestService-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name tesse_ros_bridge-srv:<RepositionRequestService-request> is deprecated: use tesse_ros_bridge-srv:RepositionRequestService-request instead.")))

(cl:ensure-generic-function 'pose-val :lambda-list '(m))
(cl:defmethod pose-val ((m <RepositionRequestService-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tesse_ros_bridge-srv:pose-val is deprecated.  Use tesse_ros_bridge-srv:pose instead.")
  (pose m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RepositionRequestService-request>) ostream)
  "Serializes a message object of type '<RepositionRequestService-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'pose) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RepositionRequestService-request>) istream)
  "Deserializes a message object of type '<RepositionRequestService-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'pose) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RepositionRequestService-request>)))
  "Returns string type for a service object of type '<RepositionRequestService-request>"
  "tesse_ros_bridge/RepositionRequestServiceRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RepositionRequestService-request)))
  "Returns string type for a service object of type 'RepositionRequestService-request"
  "tesse_ros_bridge/RepositionRequestServiceRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RepositionRequestService-request>)))
  "Returns md5sum for a message object of type '<RepositionRequestService-request>"
  "b66f4c8c2fa81b98544cf0f3e10950ee")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RepositionRequestService-request)))
  "Returns md5sum for a message object of type 'RepositionRequestService-request"
  "b66f4c8c2fa81b98544cf0f3e10950ee")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RepositionRequestService-request>)))
  "Returns full string definition for message of type '<RepositionRequestService-request>"
  (cl:format cl:nil "## Reposition Request Service~%~%# Request fields~%geometry_msgs/Pose pose~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RepositionRequestService-request)))
  "Returns full string definition for message of type 'RepositionRequestService-request"
  (cl:format cl:nil "## Reposition Request Service~%~%# Request fields~%geometry_msgs/Pose pose~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RepositionRequestService-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'pose))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RepositionRequestService-request>))
  "Converts a ROS message object to a list"
  (cl:list 'RepositionRequestService-request
    (cl:cons ':pose (pose msg))
))
;//! \htmlinclude RepositionRequestService-response.msg.html

(cl:defclass <RepositionRequestService-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass RepositionRequestService-response (<RepositionRequestService-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RepositionRequestService-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RepositionRequestService-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name tesse_ros_bridge-srv:<RepositionRequestService-response> is deprecated: use tesse_ros_bridge-srv:RepositionRequestService-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <RepositionRequestService-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tesse_ros_bridge-srv:success-val is deprecated.  Use tesse_ros_bridge-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RepositionRequestService-response>) ostream)
  "Serializes a message object of type '<RepositionRequestService-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RepositionRequestService-response>) istream)
  "Deserializes a message object of type '<RepositionRequestService-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RepositionRequestService-response>)))
  "Returns string type for a service object of type '<RepositionRequestService-response>"
  "tesse_ros_bridge/RepositionRequestServiceResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RepositionRequestService-response)))
  "Returns string type for a service object of type 'RepositionRequestService-response"
  "tesse_ros_bridge/RepositionRequestServiceResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RepositionRequestService-response>)))
  "Returns md5sum for a message object of type '<RepositionRequestService-response>"
  "b66f4c8c2fa81b98544cf0f3e10950ee")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RepositionRequestService-response)))
  "Returns md5sum for a message object of type 'RepositionRequestService-response"
  "b66f4c8c2fa81b98544cf0f3e10950ee")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RepositionRequestService-response>)))
  "Returns full string definition for message of type '<RepositionRequestService-response>"
  (cl:format cl:nil "~%# Response fields~%bool success  # true if the reposition happens without exception, false otw~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RepositionRequestService-response)))
  "Returns full string definition for message of type 'RepositionRequestService-response"
  (cl:format cl:nil "~%# Response fields~%bool success  # true if the reposition happens without exception, false otw~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RepositionRequestService-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RepositionRequestService-response>))
  "Converts a ROS message object to a list"
  (cl:list 'RepositionRequestService-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'RepositionRequestService)))
  'RepositionRequestService-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'RepositionRequestService)))
  'RepositionRequestService-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RepositionRequestService)))
  "Returns string type for a service object of type '<RepositionRequestService>"
  "tesse_ros_bridge/RepositionRequestService")