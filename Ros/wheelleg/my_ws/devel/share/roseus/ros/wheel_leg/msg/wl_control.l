;; Auto-generated. Do not edit!


(when (boundp 'wheel_leg::wl_control)
  (if (not (find-package "WHEEL_LEG"))
    (make-package "WHEEL_LEG"))
  (shadow 'wl_control (find-package "WHEEL_LEG")))
(unless (find-package "WHEEL_LEG::WL_CONTROL")
  (make-package "WHEEL_LEG::WL_CONTROL"))

(in-package "ROS")
;;//! \htmlinclude wl_control.msg.html


(defclass wheel_leg::wl_control
  :super ros::object
  :slots (_linear_vel _angular_vel _leg_pos ))

(defmethod wheel_leg::wl_control
  (:init
   (&key
    ((:linear_vel __linear_vel) 0.0)
    ((:angular_vel __angular_vel) 0.0)
    ((:leg_pos __leg_pos) 0.0)
    )
   (send-super :init)
   (setq _linear_vel (float __linear_vel))
   (setq _angular_vel (float __angular_vel))
   (setq _leg_pos (float __leg_pos))
   self)
  (:linear_vel
   (&optional __linear_vel)
   (if __linear_vel (setq _linear_vel __linear_vel)) _linear_vel)
  (:angular_vel
   (&optional __angular_vel)
   (if __angular_vel (setq _angular_vel __angular_vel)) _angular_vel)
  (:leg_pos
   (&optional __leg_pos)
   (if __leg_pos (setq _leg_pos __leg_pos)) _leg_pos)
  (:serialization-length
   ()
   (+
    ;; float64 _linear_vel
    8
    ;; float64 _angular_vel
    8
    ;; float64 _leg_pos
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _linear_vel
       (sys::poke _linear_vel (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _angular_vel
       (sys::poke _angular_vel (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _leg_pos
       (sys::poke _leg_pos (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _linear_vel
     (setq _linear_vel (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _angular_vel
     (setq _angular_vel (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _leg_pos
     (setq _leg_pos (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get wheel_leg::wl_control :md5sum-) "7fc1c80748cd92d4ec0e0172340cea1e")
(setf (get wheel_leg::wl_control :datatype-) "wheel_leg/wl_control")
(setf (get wheel_leg::wl_control :definition-)
      "float64 linear_vel  # 线速度
float64 angular_vel # 角速度
float64 leg_pos  # 腿关节位置
")



(provide :wheel_leg/wl_control "7fc1c80748cd92d4ec0e0172340cea1e")


