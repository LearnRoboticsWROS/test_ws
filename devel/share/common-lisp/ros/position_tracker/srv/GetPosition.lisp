; Auto-generated. Do not edit!


(cl:in-package position_tracker-srv)


;//! \htmlinclude GetPosition-request.msg.html

(cl:defclass <GetPosition-request> (roslisp-msg-protocol:ros-message)
  ((get_position
    :reader get_position
    :initarg :get_position
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass GetPosition-request (<GetPosition-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetPosition-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetPosition-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name position_tracker-srv:<GetPosition-request> is deprecated: use position_tracker-srv:GetPosition-request instead.")))

(cl:ensure-generic-function 'get_position-val :lambda-list '(m))
(cl:defmethod get_position-val ((m <GetPosition-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader position_tracker-srv:get_position-val is deprecated.  Use position_tracker-srv:get_position instead.")
  (get_position m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetPosition-request>) ostream)
  "Serializes a message object of type '<GetPosition-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'get_position) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetPosition-request>) istream)
  "Deserializes a message object of type '<GetPosition-request>"
    (cl:setf (cl:slot-value msg 'get_position) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetPosition-request>)))
  "Returns string type for a service object of type '<GetPosition-request>"
  "position_tracker/GetPositionRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetPosition-request)))
  "Returns string type for a service object of type 'GetPosition-request"
  "position_tracker/GetPositionRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetPosition-request>)))
  "Returns md5sum for a message object of type '<GetPosition-request>"
  "4931ed2ee82bda66c8bc53fd24c21c60")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetPosition-request)))
  "Returns md5sum for a message object of type 'GetPosition-request"
  "4931ed2ee82bda66c8bc53fd24c21c60")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetPosition-request>)))
  "Returns full string definition for message of type '<GetPosition-request>"
  (cl:format cl:nil "bool get_position~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetPosition-request)))
  "Returns full string definition for message of type 'GetPosition-request"
  (cl:format cl:nil "bool get_position~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetPosition-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetPosition-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetPosition-request
    (cl:cons ':get_position (get_position msg))
))
;//! \htmlinclude GetPosition-response.msg.html

(cl:defclass <GetPosition-response> (roslisp-msg-protocol:ros-message)
  ((x_position
    :reader x_position
    :initarg :x_position
    :type cl:float
    :initform 0.0)
   (y_position
    :reader y_position
    :initarg :y_position
    :type cl:float
    :initform 0.0)
   (z_position
    :reader z_position
    :initarg :z_position
    :type cl:float
    :initform 0.0))
)

(cl:defclass GetPosition-response (<GetPosition-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetPosition-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetPosition-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name position_tracker-srv:<GetPosition-response> is deprecated: use position_tracker-srv:GetPosition-response instead.")))

(cl:ensure-generic-function 'x_position-val :lambda-list '(m))
(cl:defmethod x_position-val ((m <GetPosition-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader position_tracker-srv:x_position-val is deprecated.  Use position_tracker-srv:x_position instead.")
  (x_position m))

(cl:ensure-generic-function 'y_position-val :lambda-list '(m))
(cl:defmethod y_position-val ((m <GetPosition-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader position_tracker-srv:y_position-val is deprecated.  Use position_tracker-srv:y_position instead.")
  (y_position m))

(cl:ensure-generic-function 'z_position-val :lambda-list '(m))
(cl:defmethod z_position-val ((m <GetPosition-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader position_tracker-srv:z_position-val is deprecated.  Use position_tracker-srv:z_position instead.")
  (z_position m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetPosition-response>) ostream)
  "Serializes a message object of type '<GetPosition-response>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'x_position))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'y_position))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'z_position))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetPosition-response>) istream)
  "Deserializes a message object of type '<GetPosition-response>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'x_position) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'y_position) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'z_position) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetPosition-response>)))
  "Returns string type for a service object of type '<GetPosition-response>"
  "position_tracker/GetPositionResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetPosition-response)))
  "Returns string type for a service object of type 'GetPosition-response"
  "position_tracker/GetPositionResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetPosition-response>)))
  "Returns md5sum for a message object of type '<GetPosition-response>"
  "4931ed2ee82bda66c8bc53fd24c21c60")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetPosition-response)))
  "Returns md5sum for a message object of type 'GetPosition-response"
  "4931ed2ee82bda66c8bc53fd24c21c60")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetPosition-response>)))
  "Returns full string definition for message of type '<GetPosition-response>"
  (cl:format cl:nil "float64 x_position~%float64 y_position~%float64 z_position~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetPosition-response)))
  "Returns full string definition for message of type 'GetPosition-response"
  (cl:format cl:nil "float64 x_position~%float64 y_position~%float64 z_position~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetPosition-response>))
  (cl:+ 0
     8
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetPosition-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetPosition-response
    (cl:cons ':x_position (x_position msg))
    (cl:cons ':y_position (y_position msg))
    (cl:cons ':z_position (z_position msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetPosition)))
  'GetPosition-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetPosition)))
  'GetPosition-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetPosition)))
  "Returns string type for a service object of type '<GetPosition>"
  "position_tracker/GetPosition")