; Auto-generated. Do not edit!


(cl:in-package SpyKee-msg)


;//! \htmlinclude Vision.msg.html

(cl:defclass <Vision> (roslisp-msg-protocol:ros-message)
  ((rcvd
    :reader rcvd
    :initarg :rcvd
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass Vision (<Vision>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Vision>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Vision)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name SpyKee-msg:<Vision> is deprecated: use SpyKee-msg:Vision instead.")))

(cl:ensure-generic-function 'rcvd-val :lambda-list '(m))
(cl:defmethod rcvd-val ((m <Vision>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader SpyKee-msg:rcvd-val is deprecated.  Use SpyKee-msg:rcvd instead.")
  (rcvd m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Vision>) ostream)
  "Serializes a message object of type '<Vision>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'rcvd) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Vision>) istream)
  "Deserializes a message object of type '<Vision>"
    (cl:setf (cl:slot-value msg 'rcvd) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Vision>)))
  "Returns string type for a message object of type '<Vision>"
  "SpyKee/Vision")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Vision)))
  "Returns string type for a message object of type 'Vision"
  "SpyKee/Vision")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Vision>)))
  "Returns md5sum for a message object of type '<Vision>"
  "ef16f97691d5a33b2bd3dca95054413d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Vision)))
  "Returns md5sum for a message object of type 'Vision"
  "ef16f97691d5a33b2bd3dca95054413d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Vision>)))
  "Returns full string definition for message of type '<Vision>"
  (cl:format cl:nil "bool rcvd ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Vision)))
  "Returns full string definition for message of type 'Vision"
  (cl:format cl:nil "bool rcvd ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Vision>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Vision>))
  "Converts a ROS message object to a list"
  (cl:list 'Vision
    (cl:cons ':rcvd (rcvd msg))
))
