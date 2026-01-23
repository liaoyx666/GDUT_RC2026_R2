; Auto-generated. Do not edit!


(cl:in-package wheel_leg-msg)


;//! \htmlinclude wl_control.msg.html

(cl:defclass <wl_control> (roslisp-msg-protocol:ros-message)
  ((linear_vel
    :reader linear_vel
    :initarg :linear_vel
    :type cl:float
    :initform 0.0)
   (angular_vel
    :reader angular_vel
    :initarg :angular_vel
    :type cl:float
    :initform 0.0)
   (leg_pos
    :reader leg_pos
    :initarg :leg_pos
    :type cl:float
    :initform 0.0))
)

(cl:defclass wl_control (<wl_control>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <wl_control>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'wl_control)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name wheel_leg-msg:<wl_control> is deprecated: use wheel_leg-msg:wl_control instead.")))

(cl:ensure-generic-function 'linear_vel-val :lambda-list '(m))
(cl:defmethod linear_vel-val ((m <wl_control>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader wheel_leg-msg:linear_vel-val is deprecated.  Use wheel_leg-msg:linear_vel instead.")
  (linear_vel m))

(cl:ensure-generic-function 'angular_vel-val :lambda-list '(m))
(cl:defmethod angular_vel-val ((m <wl_control>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader wheel_leg-msg:angular_vel-val is deprecated.  Use wheel_leg-msg:angular_vel instead.")
  (angular_vel m))

(cl:ensure-generic-function 'leg_pos-val :lambda-list '(m))
(cl:defmethod leg_pos-val ((m <wl_control>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader wheel_leg-msg:leg_pos-val is deprecated.  Use wheel_leg-msg:leg_pos instead.")
  (leg_pos m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <wl_control>) ostream)
  "Serializes a message object of type '<wl_control>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'linear_vel))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'angular_vel))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'leg_pos))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <wl_control>) istream)
  "Deserializes a message object of type '<wl_control>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'linear_vel) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'angular_vel) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'leg_pos) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<wl_control>)))
  "Returns string type for a message object of type '<wl_control>"
  "wheel_leg/wl_control")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'wl_control)))
  "Returns string type for a message object of type 'wl_control"
  "wheel_leg/wl_control")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<wl_control>)))
  "Returns md5sum for a message object of type '<wl_control>"
  "7fc1c80748cd92d4ec0e0172340cea1e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'wl_control)))
  "Returns md5sum for a message object of type 'wl_control"
  "7fc1c80748cd92d4ec0e0172340cea1e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<wl_control>)))
  "Returns full string definition for message of type '<wl_control>"
  (cl:format cl:nil "float64 linear_vel  # 线速度~%float64 angular_vel # 角速度~%float64 leg_pos  # 腿关节位置~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'wl_control)))
  "Returns full string definition for message of type 'wl_control"
  (cl:format cl:nil "float64 linear_vel  # 线速度~%float64 angular_vel # 角速度~%float64 leg_pos  # 腿关节位置~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <wl_control>))
  (cl:+ 0
     8
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <wl_control>))
  "Converts a ROS message object to a list"
  (cl:list 'wl_control
    (cl:cons ':linear_vel (linear_vel msg))
    (cl:cons ':angular_vel (angular_vel msg))
    (cl:cons ':leg_pos (leg_pos msg))
))
