;; Auto-generated. Do not edit!


(when (boundp 'position_tracker::GetPosition)
  (if (not (find-package "POSITION_TRACKER"))
    (make-package "POSITION_TRACKER"))
  (shadow 'GetPosition (find-package "POSITION_TRACKER")))
(unless (find-package "POSITION_TRACKER::GETPOSITION")
  (make-package "POSITION_TRACKER::GETPOSITION"))
(unless (find-package "POSITION_TRACKER::GETPOSITIONREQUEST")
  (make-package "POSITION_TRACKER::GETPOSITIONREQUEST"))
(unless (find-package "POSITION_TRACKER::GETPOSITIONRESPONSE")
  (make-package "POSITION_TRACKER::GETPOSITIONRESPONSE"))

(in-package "ROS")





(defclass position_tracker::GetPositionRequest
  :super ros::object
  :slots (_get_position ))

(defmethod position_tracker::GetPositionRequest
  (:init
   (&key
    ((:get_position __get_position) nil)
    )
   (send-super :init)
   (setq _get_position __get_position)
   self)
  (:get_position
   (&optional (__get_position :null))
   (if (not (eq __get_position :null)) (setq _get_position __get_position)) _get_position)
  (:serialization-length
   ()
   (+
    ;; bool _get_position
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _get_position
       (if _get_position (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _get_position
     (setq _get_position (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass position_tracker::GetPositionResponse
  :super ros::object
  :slots (_x_position _y_position _z_position ))

(defmethod position_tracker::GetPositionResponse
  (:init
   (&key
    ((:x_position __x_position) 0.0)
    ((:y_position __y_position) 0.0)
    ((:z_position __z_position) 0.0)
    )
   (send-super :init)
   (setq _x_position (float __x_position))
   (setq _y_position (float __y_position))
   (setq _z_position (float __z_position))
   self)
  (:x_position
   (&optional __x_position)
   (if __x_position (setq _x_position __x_position)) _x_position)
  (:y_position
   (&optional __y_position)
   (if __y_position (setq _y_position __y_position)) _y_position)
  (:z_position
   (&optional __z_position)
   (if __z_position (setq _z_position __z_position)) _z_position)
  (:serialization-length
   ()
   (+
    ;; float64 _x_position
    8
    ;; float64 _y_position
    8
    ;; float64 _z_position
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _x_position
       (sys::poke _x_position (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _y_position
       (sys::poke _y_position (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _z_position
       (sys::poke _z_position (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _x_position
     (setq _x_position (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _y_position
     (setq _y_position (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _z_position
     (setq _z_position (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(defclass position_tracker::GetPosition
  :super ros::object
  :slots ())

(setf (get position_tracker::GetPosition :md5sum-) "4931ed2ee82bda66c8bc53fd24c21c60")
(setf (get position_tracker::GetPosition :datatype-) "position_tracker/GetPosition")
(setf (get position_tracker::GetPosition :request) position_tracker::GetPositionRequest)
(setf (get position_tracker::GetPosition :response) position_tracker::GetPositionResponse)

(defmethod position_tracker::GetPositionRequest
  (:response () (instance position_tracker::GetPositionResponse :init)))

(setf (get position_tracker::GetPositionRequest :md5sum-) "4931ed2ee82bda66c8bc53fd24c21c60")
(setf (get position_tracker::GetPositionRequest :datatype-) "position_tracker/GetPositionRequest")
(setf (get position_tracker::GetPositionRequest :definition-)
      "bool get_position
---
float64 x_position
float64 y_position
float64 z_position
")

(setf (get position_tracker::GetPositionResponse :md5sum-) "4931ed2ee82bda66c8bc53fd24c21c60")
(setf (get position_tracker::GetPositionResponse :datatype-) "position_tracker/GetPositionResponse")
(setf (get position_tracker::GetPositionResponse :definition-)
      "bool get_position
---
float64 x_position
float64 y_position
float64 z_position
")



(provide :position_tracker/GetPosition "4931ed2ee82bda66c8bc53fd24c21c60")


