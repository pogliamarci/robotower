; Auto-generated. Do not edit!


(cl:in-package SpyKee-msg)


;//! \htmlinclude Motion.msg.html

(cl:defclass <Motion> (roslisp-msg-protocol:ros-message)
  ((leftTrack
    :reader leftTrack
    :initarg :leftTrack
    :type cl:fixnum
    :initform 0)
   (rightTrack
    :reader rightTrack
    :initarg :rightTrack
    :type cl:fixnum
    :initform 0))
)

(cl:defclass Motion (<Motion>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Motion>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Motion)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name SpyKee-msg:<Motion> is deprecated: use SpyKee-msg:Motion instead.")))

(cl:ensure-generic-function 'leftTrack-val :lambda-list '(m))
(cl:defmethod leftTrack-val ((m <Motion>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader SpyKee-msg:leftTrack-val is deprecated.  Use SpyKee-msg:leftTrack instead.")
  (leftTrack m))

(cl:ensure-generic-function 'rightTrack-val :lambda-list '(m))
(cl:defmethod rightTrack-val ((m <Motion>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader SpyKee-msg:rightTrack-val is deprecated.  Use SpyKee-msg:rightTrack instead.")
  (rightTrack m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Motion>) ostream)
  "Serializes a message object of type '<Motion>"
  (cl:let* ((signed (cl:slot-value msg 'leftTrack)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'rightTrack)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Motion>) istream)
  "Deserializes a message object of type '<Motion>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'leftTrack) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'rightTrack) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Motion>)))
  "Returns string type for a message object of type '<Motion>"
  "SpyKee/Motion")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Motion)))
  "Returns string type for a message object of type 'Motion"
  "SpyKee/Motion")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Motion>)))
  "Returns md5sum for a message object of type '<Motion>"
  "8b9854287dfc6ff663000aa3d6914b20")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Motion)))
  "Returns md5sum for a message object of type 'Motion"
  "8b9854287dfc6ff663000aa3d6914b20")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Motion>)))
  "Returns full string definition for message of type '<Motion>"
  (cl:format cl:nil "int8 leftTrack~%int8 rightTrack~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Motion)))
  "Returns full string definition for message of type 'Motion"
  (cl:format cl:nil "int8 leftTrack~%int8 rightTrack~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Motion>))
  (cl:+ 0
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Motion>))
  "Converts a ROS message object to a list"
  (cl:list 'Motion
    (cl:cons ':leftTrack (leftTrack msg))
    (cl:cons ':rightTrack (rightTrack msg))
))
