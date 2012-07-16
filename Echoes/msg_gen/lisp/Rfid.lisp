; Auto-generated. Do not edit!


(cl:in-package Echoes-msg)


;//! \htmlinclude Rfid.msg.html

(cl:defclass <Rfid> (roslisp-msg-protocol:ros-message)
  ((id
    :reader id
    :initarg :id
    :type cl:string
    :initform ""))
)

(cl:defclass Rfid (<Rfid>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Rfid>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Rfid)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name Echoes-msg:<Rfid> is deprecated: use Echoes-msg:Rfid instead.")))

(cl:ensure-generic-function 'id-val :lambda-list '(m))
(cl:defmethod id-val ((m <Rfid>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Echoes-msg:id-val is deprecated.  Use Echoes-msg:id instead.")
  (id m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Rfid>) ostream)
  "Serializes a message object of type '<Rfid>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'id))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'id))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Rfid>) istream)
  "Deserializes a message object of type '<Rfid>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'id) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'id) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Rfid>)))
  "Returns string type for a message object of type '<Rfid>"
  "Echoes/Rfid")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Rfid)))
  "Returns string type for a message object of type 'Rfid"
  "Echoes/Rfid")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Rfid>)))
  "Returns md5sum for a message object of type '<Rfid>"
  "bbfcda76036ebbe3d36caf7af80b260c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Rfid)))
  "Returns md5sum for a message object of type 'Rfid"
  "bbfcda76036ebbe3d36caf7af80b260c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Rfid>)))
  "Returns full string definition for message of type '<Rfid>"
  (cl:format cl:nil "string id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Rfid)))
  "Returns full string definition for message of type 'Rfid"
  (cl:format cl:nil "string id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Rfid>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'id))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Rfid>))
  "Converts a ROS message object to a list"
  (cl:list 'Rfid
    (cl:cons ':id (id msg))
))
