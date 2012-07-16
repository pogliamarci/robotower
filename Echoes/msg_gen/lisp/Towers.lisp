; Auto-generated. Do not edit!


(cl:in-package Echoes-msg)


;//! \htmlinclude Towers.msg.html

(cl:defclass <Towers> (roslisp-msg-protocol:ros-message)
  ((isTowerDestroyed
    :reader isTowerDestroyed
    :initarg :isTowerDestroyed
    :type cl:fixnum
    :initform 0)
   (destroyedFactories
    :reader destroyedFactories
    :initarg :destroyedFactories
    :type cl:fixnum
    :initform 0))
)

(cl:defclass Towers (<Towers>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Towers>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Towers)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name Echoes-msg:<Towers> is deprecated: use Echoes-msg:Towers instead.")))

(cl:ensure-generic-function 'isTowerDestroyed-val :lambda-list '(m))
(cl:defmethod isTowerDestroyed-val ((m <Towers>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Echoes-msg:isTowerDestroyed-val is deprecated.  Use Echoes-msg:isTowerDestroyed instead.")
  (isTowerDestroyed m))

(cl:ensure-generic-function 'destroyedFactories-val :lambda-list '(m))
(cl:defmethod destroyedFactories-val ((m <Towers>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Echoes-msg:destroyedFactories-val is deprecated.  Use Echoes-msg:destroyedFactories instead.")
  (destroyedFactories m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Towers>) ostream)
  "Serializes a message object of type '<Towers>"
  (cl:let* ((signed (cl:slot-value msg 'isTowerDestroyed)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'destroyedFactories)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Towers>) istream)
  "Deserializes a message object of type '<Towers>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'isTowerDestroyed) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'destroyedFactories) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Towers>)))
  "Returns string type for a message object of type '<Towers>"
  "Echoes/Towers")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Towers)))
  "Returns string type for a message object of type 'Towers"
  "Echoes/Towers")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Towers>)))
  "Returns md5sum for a message object of type '<Towers>"
  "320b7d3f9b63efbcdce4894d7f51a261")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Towers)))
  "Returns md5sum for a message object of type 'Towers"
  "320b7d3f9b63efbcdce4894d7f51a261")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Towers>)))
  "Returns full string definition for message of type '<Towers>"
  (cl:format cl:nil "int8 isTowerDestroyed~%int8 destroyedFactories~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Towers)))
  "Returns full string definition for message of type 'Towers"
  (cl:format cl:nil "int8 isTowerDestroyed~%int8 destroyedFactories~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Towers>))
  (cl:+ 0
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Towers>))
  "Converts a ROS message object to a list"
  (cl:list 'Towers
    (cl:cons ':isTowerDestroyed (isTowerDestroyed msg))
    (cl:cons ':destroyedFactories (destroyedFactories msg))
))
