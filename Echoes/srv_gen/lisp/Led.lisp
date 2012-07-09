; Auto-generated. Do not edit!


(cl:in-package Echoes-srv)


;//! \htmlinclude Led-request.msg.html

(cl:defclass <Led-request> (roslisp-msg-protocol:ros-message)
  ((editGreen
    :reader editGreen
    :initarg :editGreen
    :type cl:boolean
    :initform cl:nil)
   (editYellow
    :reader editYellow
    :initarg :editYellow
    :type cl:boolean
    :initform cl:nil)
   (editRed
    :reader editRed
    :initarg :editRed
    :type cl:boolean
    :initform cl:nil)
   (greenIsOn
    :reader greenIsOn
    :initarg :greenIsOn
    :type cl:boolean
    :initform cl:nil)
   (redNumOn
    :reader redNumOn
    :initarg :redNumOn
    :type cl:fixnum
    :initform 0)
   (yellowIsOn
    :reader yellowIsOn
    :initarg :yellowIsOn
    :type (cl:vector cl:boolean)
   :initform (cl:make-array 4 :element-type 'cl:boolean :initial-element cl:nil)))
)

(cl:defclass Led-request (<Led-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Led-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Led-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name Echoes-srv:<Led-request> is deprecated: use Echoes-srv:Led-request instead.")))

(cl:ensure-generic-function 'editGreen-val :lambda-list '(m))
(cl:defmethod editGreen-val ((m <Led-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Echoes-srv:editGreen-val is deprecated.  Use Echoes-srv:editGreen instead.")
  (editGreen m))

(cl:ensure-generic-function 'editYellow-val :lambda-list '(m))
(cl:defmethod editYellow-val ((m <Led-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Echoes-srv:editYellow-val is deprecated.  Use Echoes-srv:editYellow instead.")
  (editYellow m))

(cl:ensure-generic-function 'editRed-val :lambda-list '(m))
(cl:defmethod editRed-val ((m <Led-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Echoes-srv:editRed-val is deprecated.  Use Echoes-srv:editRed instead.")
  (editRed m))

(cl:ensure-generic-function 'greenIsOn-val :lambda-list '(m))
(cl:defmethod greenIsOn-val ((m <Led-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Echoes-srv:greenIsOn-val is deprecated.  Use Echoes-srv:greenIsOn instead.")
  (greenIsOn m))

(cl:ensure-generic-function 'redNumOn-val :lambda-list '(m))
(cl:defmethod redNumOn-val ((m <Led-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Echoes-srv:redNumOn-val is deprecated.  Use Echoes-srv:redNumOn instead.")
  (redNumOn m))

(cl:ensure-generic-function 'yellowIsOn-val :lambda-list '(m))
(cl:defmethod yellowIsOn-val ((m <Led-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Echoes-srv:yellowIsOn-val is deprecated.  Use Echoes-srv:yellowIsOn instead.")
  (yellowIsOn m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Led-request>) ostream)
  "Serializes a message object of type '<Led-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'editGreen) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'editYellow) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'editRed) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'greenIsOn) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'redNumOn)) ostream)
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if ele 1 0)) ostream))
   (cl:slot-value msg 'yellowIsOn))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Led-request>) istream)
  "Deserializes a message object of type '<Led-request>"
    (cl:setf (cl:slot-value msg 'editGreen) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'editYellow) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'editRed) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'greenIsOn) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'redNumOn)) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'yellowIsOn) (cl:make-array 4))
  (cl:let ((vals (cl:slot-value msg 'yellowIsOn)))
    (cl:dotimes (i 4)
    (cl:setf (cl:aref vals i) (cl:not (cl:zerop (cl:read-byte istream))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Led-request>)))
  "Returns string type for a service object of type '<Led-request>"
  "Echoes/LedRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Led-request)))
  "Returns string type for a service object of type 'Led-request"
  "Echoes/LedRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Led-request>)))
  "Returns md5sum for a message object of type '<Led-request>"
  "123e812292460e49a24080052eae9c17")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Led-request)))
  "Returns md5sum for a message object of type 'Led-request"
  "123e812292460e49a24080052eae9c17")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Led-request>)))
  "Returns full string definition for message of type '<Led-request>"
  (cl:format cl:nil "bool editGreen~%bool editYellow~%bool editRed~%bool greenIsOn~%uint8 redNumOn~%bool[4] yellowIsOn~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Led-request)))
  "Returns full string definition for message of type 'Led-request"
  (cl:format cl:nil "bool editGreen~%bool editYellow~%bool editRed~%bool greenIsOn~%uint8 redNumOn~%bool[4] yellowIsOn~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Led-request>))
  (cl:+ 0
     1
     1
     1
     1
     1
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'yellowIsOn) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Led-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Led-request
    (cl:cons ':editGreen (editGreen msg))
    (cl:cons ':editYellow (editYellow msg))
    (cl:cons ':editRed (editRed msg))
    (cl:cons ':greenIsOn (greenIsOn msg))
    (cl:cons ':redNumOn (redNumOn msg))
    (cl:cons ':yellowIsOn (yellowIsOn msg))
))
;//! \htmlinclude Led-response.msg.html

(cl:defclass <Led-response> (roslisp-msg-protocol:ros-message)
  ((requestSuccessful
    :reader requestSuccessful
    :initarg :requestSuccessful
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass Led-response (<Led-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Led-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Led-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name Echoes-srv:<Led-response> is deprecated: use Echoes-srv:Led-response instead.")))

(cl:ensure-generic-function 'requestSuccessful-val :lambda-list '(m))
(cl:defmethod requestSuccessful-val ((m <Led-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Echoes-srv:requestSuccessful-val is deprecated.  Use Echoes-srv:requestSuccessful instead.")
  (requestSuccessful m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Led-response>) ostream)
  "Serializes a message object of type '<Led-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'requestSuccessful) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Led-response>) istream)
  "Deserializes a message object of type '<Led-response>"
    (cl:setf (cl:slot-value msg 'requestSuccessful) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Led-response>)))
  "Returns string type for a service object of type '<Led-response>"
  "Echoes/LedResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Led-response)))
  "Returns string type for a service object of type 'Led-response"
  "Echoes/LedResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Led-response>)))
  "Returns md5sum for a message object of type '<Led-response>"
  "123e812292460e49a24080052eae9c17")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Led-response)))
  "Returns md5sum for a message object of type 'Led-response"
  "123e812292460e49a24080052eae9c17")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Led-response>)))
  "Returns full string definition for message of type '<Led-response>"
  (cl:format cl:nil "bool requestSuccessful~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Led-response)))
  "Returns full string definition for message of type 'Led-response"
  (cl:format cl:nil "bool requestSuccessful~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Led-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Led-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Led-response
    (cl:cons ':requestSuccessful (requestSuccessful msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Led)))
  'Led-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Led)))
  'Led-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Led)))
  "Returns string type for a service object of type '<Led>"
  "Echoes/Led")