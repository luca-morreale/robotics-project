;; Auto-generated. Do not edit!


(when (boundp 'kobra_plugins::ptz_msg)
  (if (not (find-package "KOBRA_PLUGINS"))
    (make-package "KOBRA_PLUGINS"))
  (shadow 'ptz_msg (find-package "KOBRA_PLUGINS")))
(unless (find-package "KOBRA_PLUGINS::PTZ_MSG")
  (make-package "KOBRA_PLUGINS::PTZ_MSG"))

(in-package "ROS")
;;//! \htmlinclude ptz_msg.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass kobra_plugins::ptz_msg
  :super ros::object
  :slots (_header _pan _tilt _zoom ))

(defmethod kobra_plugins::ptz_msg
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:pan __pan) 0.0)
    ((:tilt __tilt) 0.0)
    ((:zoom __zoom) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _pan (float __pan))
   (setq _tilt (float __tilt))
   (setq _zoom (float __zoom))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:pan
   (&optional __pan)
   (if __pan (setq _pan __pan)) _pan)
  (:tilt
   (&optional __tilt)
   (if __tilt (setq _tilt __tilt)) _tilt)
  (:zoom
   (&optional __zoom)
   (if __zoom (setq _zoom __zoom)) _zoom)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float64 _pan
    8
    ;; float64 _tilt
    8
    ;; float64 _zoom
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float64 _pan
       (sys::poke _pan (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _tilt
       (sys::poke _tilt (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _zoom
       (sys::poke _zoom (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float64 _pan
     (setq _pan (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _tilt
     (setq _tilt (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _zoom
     (setq _zoom (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get kobra_plugins::ptz_msg :md5sum-) "6cdfd49c4536c9dd35c1b03a7330ea40")
(setf (get kobra_plugins::ptz_msg :datatype-) "kobra_plugins/ptz_msg")
(setf (get kobra_plugins::ptz_msg :definition-)
      "

Header header
float64 pan
float64 tilt
float64 zoom

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
# 0: no frame
# 1: global frame
string frame_id

")



(provide :kobra_plugins/ptz_msg "6cdfd49c4536c9dd35c1b03a7330ea40")


