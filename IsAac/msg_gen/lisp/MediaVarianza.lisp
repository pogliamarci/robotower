; Auto-generated. Do not edit!


(cl:in-package IsAac-msg)


;//! \htmlinclude MediaVarianza.msg.html

(cl:defclass <MediaVarianza> (roslisp-msg-protocol:ros-message)
  ((Average
    :reader Average
    :initarg :Average
    :type cl:float
    :initform 0.0)
   (Variance
    :reader Variance
    :initarg :Variance
    :type cl:float
    :initform 0.0)
   (Time
    :reader Time
    :initarg :Time
    :type cl:integer
    :initform 0))
)

(cl:defclass MediaVarianza (<MediaVarianza>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MediaVarianza>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MediaVarianza)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name IsAac-msg:<MediaVarianza> is deprecated: use IsAac-msg:MediaVarianza instead.")))

(cl:ensure-generic-function 'Average-val :lambda-list '(m))
(cl:defmethod Average-val ((m <MediaVarianza>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader IsAac-msg:Average-val is deprecated.  Use IsAac-msg:Average instead.")
  (Average m))

(cl:ensure-generic-function 'Variance-val :lambda-list '(m))
(cl:defmethod Variance-val ((m <MediaVarianza>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader IsAac-msg:Variance-val is deprecated.  Use IsAac-msg:Variance instead.")
  (Variance m))

(cl:ensure-generic-function 'Time-val :lambda-list '(m))
(cl:defmethod Time-val ((m <MediaVarianza>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader IsAac-msg:Time-val is deprecated.  Use IsAac-msg:Time instead.")
  (Time m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MediaVarianza>) ostream)
  "Serializes a message object of type '<MediaVarianza>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'Average))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'Variance))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'Time)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'Time)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'Time)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'Time)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MediaVarianza>) istream)
  "Deserializes a message object of type '<MediaVarianza>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'Average) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'Variance) (roslisp-utils:decode-single-float-bits bits)))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'Time)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'Time)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'Time)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'Time)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MediaVarianza>)))
  "Returns string type for a message object of type '<MediaVarianza>"
  "IsAac/MediaVarianza")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MediaVarianza)))
  "Returns string type for a message object of type 'MediaVarianza"
  "IsAac/MediaVarianza")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MediaVarianza>)))
  "Returns md5sum for a message object of type '<MediaVarianza>"
  "b496fceed5ec68264fce67157b6713d7")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MediaVarianza)))
  "Returns md5sum for a message object of type 'MediaVarianza"
  "b496fceed5ec68264fce67157b6713d7")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MediaVarianza>)))
  "Returns full string definition for message of type '<MediaVarianza>"
  (cl:format cl:nil "float32 Average~%float32 Variance~%uint32 Time~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MediaVarianza)))
  "Returns full string definition for message of type 'MediaVarianza"
  (cl:format cl:nil "float32 Average~%float32 Variance~%uint32 Time~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MediaVarianza>))
  (cl:+ 0
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MediaVarianza>))
  "Converts a ROS message object to a list"
  (cl:list 'MediaVarianza
    (cl:cons ':Average (Average msg))
    (cl:cons ':Variance (Variance msg))
    (cl:cons ':Time (Time msg))
))
