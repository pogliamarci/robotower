; Auto-generated. Do not edit!


(cl:in-package sonar-msg)


;//! \htmlinclude Sonar.msg.html

(cl:defclass <Sonar> (roslisp-msg-protocol:ros-message)
  ((north
    :reader north
    :initarg :north
    :type cl:float
    :initform 0.0)
   (south
    :reader south
    :initarg :south
    :type cl:float
    :initform 0.0)
   (east
    :reader east
    :initarg :east
    :type cl:float
    :initform 0.0)
   (west
    :reader west
    :initarg :west
    :type cl:float
    :initform 0.0))
)

(cl:defclass Sonar (<Sonar>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Sonar>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Sonar)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name sonar-msg:<Sonar> is deprecated: use sonar-msg:Sonar instead.")))

(cl:ensure-generic-function 'north-val :lambda-list '(m))
(cl:defmethod north-val ((m <Sonar>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sonar-msg:north-val is deprecated.  Use sonar-msg:north instead.")
  (north m))

(cl:ensure-generic-function 'south-val :lambda-list '(m))
(cl:defmethod south-val ((m <Sonar>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sonar-msg:south-val is deprecated.  Use sonar-msg:south instead.")
  (south m))

(cl:ensure-generic-function 'east-val :lambda-list '(m))
(cl:defmethod east-val ((m <Sonar>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sonar-msg:east-val is deprecated.  Use sonar-msg:east instead.")
  (east m))

(cl:ensure-generic-function 'west-val :lambda-list '(m))
(cl:defmethod west-val ((m <Sonar>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sonar-msg:west-val is deprecated.  Use sonar-msg:west instead.")
  (west m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Sonar>) ostream)
  "Serializes a message object of type '<Sonar>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'north))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'south))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'east))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'west))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Sonar>) istream)
  "Deserializes a message object of type '<Sonar>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'north) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'south) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'east) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'west) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Sonar>)))
  "Returns string type for a message object of type '<Sonar>"
  "sonar/Sonar")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Sonar)))
  "Returns string type for a message object of type 'Sonar"
  "sonar/Sonar")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Sonar>)))
  "Returns md5sum for a message object of type '<Sonar>"
  "bfb04bad3590e564c12fae1e3d9f10d9")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Sonar)))
  "Returns md5sum for a message object of type 'Sonar"
  "bfb04bad3590e564c12fae1e3d9f10d9")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Sonar>)))
  "Returns full string definition for message of type '<Sonar>"
  (cl:format cl:nil "float32 north~%float32 south~%float32 east~%float32 west~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Sonar)))
  "Returns full string definition for message of type 'Sonar"
  (cl:format cl:nil "float32 north~%float32 south~%float32 east~%float32 west~%~%~%"))
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
