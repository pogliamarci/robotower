; Auto-generated. Do not edit!


(cl:in-package Vision-msg)


;//! \htmlinclude Results.msg.html

(cl:defclass <Results> (roslisp-msg-protocol:ros-message)
  ((towerFound
    :reader towerFound
    :initarg :towerFound
    :type cl:boolean
    :initform cl:nil)
   (towerPos
    :reader towerPos
    :initarg :towerPos
    :type cl:integer
    :initform 0)
   (towerBlobHeight
    :reader towerBlobHeight
    :initarg :towerBlobHeight
    :type cl:integer
    :initform 0)
   (towerSize
    :reader towerSize
    :initarg :towerSize
    :type cl:integer
    :initform 0)
   (factoryFound
    :reader factoryFound
    :initarg :factoryFound
    :type cl:boolean
    :initform cl:nil)
   (factoryPos
    :reader factoryPos
    :initarg :factoryPos
    :type cl:integer
    :initform 0))
)

(cl:defclass Results (<Results>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Results>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Results)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name Vision-msg:<Results> is deprecated: use Vision-msg:Results instead.")))

(cl:ensure-generic-function 'towerFound-val :lambda-list '(m))
(cl:defmethod towerFound-val ((m <Results>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Vision-msg:towerFound-val is deprecated.  Use Vision-msg:towerFound instead.")
  (towerFound m))

(cl:ensure-generic-function 'towerPos-val :lambda-list '(m))
(cl:defmethod towerPos-val ((m <Results>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Vision-msg:towerPos-val is deprecated.  Use Vision-msg:towerPos instead.")
  (towerPos m))

(cl:ensure-generic-function 'towerBlobHeight-val :lambda-list '(m))
(cl:defmethod towerBlobHeight-val ((m <Results>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Vision-msg:towerBlobHeight-val is deprecated.  Use Vision-msg:towerBlobHeight instead.")
  (towerBlobHeight m))

(cl:ensure-generic-function 'towerSize-val :lambda-list '(m))
(cl:defmethod towerSize-val ((m <Results>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Vision-msg:towerSize-val is deprecated.  Use Vision-msg:towerSize instead.")
  (towerSize m))

(cl:ensure-generic-function 'factoryFound-val :lambda-list '(m))
(cl:defmethod factoryFound-val ((m <Results>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Vision-msg:factoryFound-val is deprecated.  Use Vision-msg:factoryFound instead.")
  (factoryFound m))

(cl:ensure-generic-function 'factoryPos-val :lambda-list '(m))
(cl:defmethod factoryPos-val ((m <Results>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Vision-msg:factoryPos-val is deprecated.  Use Vision-msg:factoryPos instead.")
  (factoryPos m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Results>) ostream)
  "Serializes a message object of type '<Results>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'towerFound) 1 0)) ostream)
  (cl:let* ((signed (cl:slot-value msg 'towerPos)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'towerBlobHeight)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'towerSize)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'factoryFound) 1 0)) ostream)
  (cl:let* ((signed (cl:slot-value msg 'factoryPos)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Results>) istream)
  "Deserializes a message object of type '<Results>"
    (cl:setf (cl:slot-value msg 'towerFound) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'towerPos) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'towerBlobHeight) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'towerSize) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:setf (cl:slot-value msg 'factoryFound) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'factoryPos) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Results>)))
  "Returns string type for a message object of type '<Results>"
  "Vision/Results")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Results)))
  "Returns string type for a message object of type 'Results"
  "Vision/Results")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Results>)))
  "Returns md5sum for a message object of type '<Results>"
  "5fb63429aaf2fdd1f1810340dee4cad4")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Results)))
  "Returns md5sum for a message object of type 'Results"
  "5fb63429aaf2fdd1f1810340dee4cad4")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Results>)))
  "Returns full string definition for message of type '<Results>"
  (cl:format cl:nil "bool towerFound~%int32  towerPos~%int32 towerBlobHeight~%int32 towerSize~%bool factoryFound~%int32  factoryPos~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Results)))
  "Returns full string definition for message of type 'Results"
  (cl:format cl:nil "bool towerFound~%int32  towerPos~%int32 towerBlobHeight~%int32 towerSize~%bool factoryFound~%int32  factoryPos~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Results>))
  (cl:+ 0
     1
     4
     4
     4
     1
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Results>))
  "Converts a ROS message object to a list"
  (cl:list 'Results
    (cl:cons ':towerFound (towerFound msg))
    (cl:cons ':towerPos (towerPos msg))
    (cl:cons ':towerBlobHeight (towerBlobHeight msg))
    (cl:cons ':towerSize (towerSize msg))
    (cl:cons ':factoryFound (factoryFound msg))
    (cl:cons ':factoryPos (factoryPos msg))
))
