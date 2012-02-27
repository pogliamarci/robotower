; Auto-generated. Do not edit!


(cl:in-package sonar-msg)


;//! \htmlinclude Led.msg.html

(cl:defclass <Led> (roslisp-msg-protocol:ros-message)
  ((greenOn
    :reader greenOn
    :initarg :greenOn
    :type cl:boolean
    :initform cl:nil)
   (numRedOn
    :reader numRedOn
    :initarg :numRedOn
    :type cl:fixnum
    :initform 0)
   (yellowOn
    :reader yellowOn
    :initarg :yellowOn
    :type (cl:vector cl:boolean)
   :initform (cl:make-array 4 :element-type 'cl:boolean :initial-element cl:nil)))
)

(cl:defclass Led (<Led>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Led>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Led)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name sonar-msg:<Led> is deprecated: use sonar-msg:Led instead.")))

(cl:ensure-generic-function 'greenOn-val :lambda-list '(m))
(cl:defmethod greenOn-val ((m <Led>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sonar-msg:greenOn-val is deprecated.  Use sonar-msg:greenOn instead.")
  (greenOn m))

(cl:ensure-generic-function 'numRedOn-val :lambda-list '(m))
(cl:defmethod numRedOn-val ((m <Led>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sonar-msg:numRedOn-val is deprecated.  Use sonar-msg:numRedOn instead.")
  (numRedOn m))

(cl:ensure-generic-function 'yellowOn-val :lambda-list '(m))
(cl:defmethod yellowOn-val ((m <Led>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sonar-msg:yellowOn-val is deprecated.  Use sonar-msg:yellowOn instead.")
  (yellowOn m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Led>) ostream)
  "Serializes a message object of type '<Led>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'greenOn) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'numRedOn)) ostream)
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if ele 1 0)) ostream))
   (cl:slot-value msg 'yellowOn))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Led>) istream)
  "Deserializes a message object of type '<Led>"
    (cl:setf (cl:slot-value msg 'greenOn) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'numRedOn)) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'yellowOn) (cl:make-array 4))
  (cl:let ((vals (cl:slot-value msg 'yellowOn)))
    (cl:dotimes (i 4)
    (cl:setf (cl:aref vals i) (cl:not (cl:zerop (cl:read-byte istream))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Led>)))
  "Returns string type for a message object of type '<Led>"
  "sonar/Led")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Led)))
  "Returns string type for a message object of type 'Led"
  "sonar/Led")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Led>)))
  "Returns md5sum for a message object of type '<Led>"
  "b4f59e8c864d23e3c9d2105fe9c83902")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Led)))
  "Returns md5sum for a message object of type 'Led"
  "b4f59e8c864d23e3c9d2105fe9c83902")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Led>)))
  "Returns full string definition for message of type '<Led>"
  (cl:format cl:nil "bool greenOn~%uint8 numRedOn~%bool[4] yellowOn~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Led)))
  "Returns full string definition for message of type 'Led"
  (cl:format cl:nil "bool greenOn~%uint8 numRedOn~%bool[4] yellowOn~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Led>))
  (cl:+ 0
     1
     1
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'yellowOn) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Led>))
  "Converts a ROS message object to a list"
  (cl:list 'Led
    (cl:cons ':greenOn (greenOn msg))
    (cl:cons ':numRedOn (numRedOn msg))
    (cl:cons ':yellowOn (yellowOn msg))
))
