; Auto-generated. Do not edit!


(cl:in-package ywu_lab3-srv)


;//! \htmlinclude aStar-request.msg.html

(cl:defclass <aStar-request> (roslisp-msg-protocol:ros-message)
  ((x0
    :reader x0
    :initarg :x0
    :type cl:integer
    :initform 0)
   (y0
    :reader y0
    :initarg :y0
    :type cl:integer
    :initform 0)
   (x1
    :reader x1
    :initarg :x1
    :type cl:integer
    :initform 0)
   (y1
    :reader y1
    :initarg :y1
    :type cl:integer
    :initform 0))
)

(cl:defclass aStar-request (<aStar-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <aStar-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'aStar-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ywu_lab3-srv:<aStar-request> is deprecated: use ywu_lab3-srv:aStar-request instead.")))

(cl:ensure-generic-function 'x0-val :lambda-list '(m))
(cl:defmethod x0-val ((m <aStar-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ywu_lab3-srv:x0-val is deprecated.  Use ywu_lab3-srv:x0 instead.")
  (x0 m))

(cl:ensure-generic-function 'y0-val :lambda-list '(m))
(cl:defmethod y0-val ((m <aStar-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ywu_lab3-srv:y0-val is deprecated.  Use ywu_lab3-srv:y0 instead.")
  (y0 m))

(cl:ensure-generic-function 'x1-val :lambda-list '(m))
(cl:defmethod x1-val ((m <aStar-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ywu_lab3-srv:x1-val is deprecated.  Use ywu_lab3-srv:x1 instead.")
  (x1 m))

(cl:ensure-generic-function 'y1-val :lambda-list '(m))
(cl:defmethod y1-val ((m <aStar-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ywu_lab3-srv:y1-val is deprecated.  Use ywu_lab3-srv:y1 instead.")
  (y1 m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <aStar-request>) ostream)
  "Serializes a message object of type '<aStar-request>"
  (cl:let* ((signed (cl:slot-value msg 'x0)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'y0)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'x1)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'y1)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <aStar-request>) istream)
  "Deserializes a message object of type '<aStar-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'x0) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'y0) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'x1) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'y1) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<aStar-request>)))
  "Returns string type for a service object of type '<aStar-request>"
  "ywu_lab3/aStarRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'aStar-request)))
  "Returns string type for a service object of type 'aStar-request"
  "ywu_lab3/aStarRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<aStar-request>)))
  "Returns md5sum for a message object of type '<aStar-request>"
  "dcbb5abadadc742196f01dc62dfaa036")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'aStar-request)))
  "Returns md5sum for a message object of type 'aStar-request"
  "dcbb5abadadc742196f01dc62dfaa036")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<aStar-request>)))
  "Returns full string definition for message of type '<aStar-request>"
  (cl:format cl:nil "int32 x0~%int32 y0~%int32 x1~%int32 y1~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'aStar-request)))
  "Returns full string definition for message of type 'aStar-request"
  (cl:format cl:nil "int32 x0~%int32 y0~%int32 x1~%int32 y1~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <aStar-request>))
  (cl:+ 0
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <aStar-request>))
  "Converts a ROS message object to a list"
  (cl:list 'aStar-request
    (cl:cons ':x0 (x0 msg))
    (cl:cons ':y0 (y0 msg))
    (cl:cons ':x1 (x1 msg))
    (cl:cons ':y1 (y1 msg))
))
;//! \htmlinclude aStar-response.msg.html

(cl:defclass <aStar-response> (roslisp-msg-protocol:ros-message)
  ((path
    :reader path
    :initarg :path
    :type nav_msgs-msg:Path
    :initform (cl:make-instance 'nav_msgs-msg:Path)))
)

(cl:defclass aStar-response (<aStar-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <aStar-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'aStar-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ywu_lab3-srv:<aStar-response> is deprecated: use ywu_lab3-srv:aStar-response instead.")))

(cl:ensure-generic-function 'path-val :lambda-list '(m))
(cl:defmethod path-val ((m <aStar-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ywu_lab3-srv:path-val is deprecated.  Use ywu_lab3-srv:path instead.")
  (path m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <aStar-response>) ostream)
  "Serializes a message object of type '<aStar-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'path) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <aStar-response>) istream)
  "Deserializes a message object of type '<aStar-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'path) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<aStar-response>)))
  "Returns string type for a service object of type '<aStar-response>"
  "ywu_lab3/aStarResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'aStar-response)))
  "Returns string type for a service object of type 'aStar-response"
  "ywu_lab3/aStarResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<aStar-response>)))
  "Returns md5sum for a message object of type '<aStar-response>"
  "dcbb5abadadc742196f01dc62dfaa036")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'aStar-response)))
  "Returns md5sum for a message object of type 'aStar-response"
  "dcbb5abadadc742196f01dc62dfaa036")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<aStar-response>)))
  "Returns full string definition for message of type '<aStar-response>"
  (cl:format cl:nil "~%nav_msgs/Path path~%~%~%================================================================================~%MSG: nav_msgs/Path~%#An array of poses that represents a Path for a robot to follow~%Header header~%geometry_msgs/PoseStamped[] poses~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'aStar-response)))
  "Returns full string definition for message of type 'aStar-response"
  (cl:format cl:nil "~%nav_msgs/Path path~%~%~%================================================================================~%MSG: nav_msgs/Path~%#An array of poses that represents a Path for a robot to follow~%Header header~%geometry_msgs/PoseStamped[] poses~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <aStar-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'path))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <aStar-response>))
  "Converts a ROS message object to a list"
  (cl:list 'aStar-response
    (cl:cons ':path (path msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'aStar)))
  'aStar-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'aStar)))
  'aStar-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'aStar)))
  "Returns string type for a service object of type '<aStar>"
  "ywu_lab3/aStar")