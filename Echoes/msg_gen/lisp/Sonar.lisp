; Auto-generated. Do not edit!


(cl:in-package Echoes-msg)


;//! \htmlinclude Sonar.msg.html

(cl:defclass <Sonar> (roslisp-msg-protocol:ros-message)
  ((north
    :reader north
    :initarg :north
    :type cl:integer
    :initform 0)
   (south
    :reader south
    :initarg :south
    :type cl:integer
    :initform 0)
   (east
    :reader east
    :initarg :east
    :type cl:integer
    :initform 0)
   (west
    :reader west
    :initarg :west
    :type cl:integer
    :initform 0))
)

(cl:defclass Sonar (<Sonar>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Sonar>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Sonar)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name Echoes-msg:<Sonar> is deprecated: use Echoes-msg:Sonar instead.")))

(cl:ensure-generic-function 'north-val :lambda-list '(m))
(cl:defmethod north-val ((m <Sonar>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Echoes-msg:north-val is deprecated.  Use Echoes-msg:north instead.")
  (north m))

(cl:ensure-generic-function 'south-val :lambda-list '(m))
(cl:defmethod south-val ((m <Sonar>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Echoes-msg:south-val is deprecated.  Use Echoes-msg:south instead.")
  (south m))

(cl:ensure-generic-function 'east-val :lambda-list '(m))
(cl:defmethod east-val ((m <Sonar>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Echoes-msg:east-val is deprecated.  Use Echoes-msg:east instead.")
  (east m))

(cl:ensure-generic-function 'west-val :lambda-list '(m))
(cl:defmethod west-val ((m <Sonar>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Echoes-msg:west-val is deprecated.  Use Echoes-msg:west instead.")
  (west m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Sonar>) ostream)
  "Serializes a message object of type '<Sonar>"
  (cl:let* ((signed (cl:slot-value msg 'north)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'south)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'east)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'west)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Sonar>) istream)
  "Deserializes a message object of type '<Sonar>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'north) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'south) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'east) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'west) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Sonar>)))
  "Returns string type for a message object of type '<Sonar>"
  "Echoes/Sonar")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Sonar)))
  "Returns string type for a message object of type 'Sonar"
  "Echoes/Sonar")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Sonar>)))
  "Returns md5sum for a message object of type '<Sonar>"
  "ae16dc531081c382844920fa0d8562a0")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Sonar)))
  "Returns md5sum for a message object of type 'Sonar"
  "ae16dc531081c382844920fa0d8562a0")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Sonar>)))
  "Returns full string definition for message of type '<Sonar>"
  (cl:format cl:nil "int32 north~%int32 south~%int32 east~%int32 west~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Sonar)))
  "Returns full string definition for message of type 'Sonar"
  (cl:format cl:nil "int32 north~%int32 south~%int32 east~%int32 west~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Sonar>))
  (cl:+ 0
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Sonar>))
  "Converts a ROS message object to a list"
  (cl:list 'Sonar
    (cl:cons ':north (north msg))
    (cl:cons ':south (south msg))
    (cl:cons ':east (east msg))
    (cl:cons ':west (west msg))
))
