;; Auto-generated. Do not edit!


(when (boundp 'vision::Markers)
  (if (not (find-package "VISION"))
    (make-package "VISION"))
  (shadow 'Markers (find-package "VISION")))
(unless (find-package "VISION::MARKERS")
  (make-package "VISION::MARKERS"))

(in-package "ROS")
;;//! \htmlinclude Markers.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass vision::Markers
  :super ros::object
  :slots (_header _markers_detected _markers_number _marker_id _marker_rotvec_x _marker_rotvec_y _marker_rotvec_z _marker_transvec_x _marker_transvec_y _marker_transvec_z ))

(defmethod vision::Markers
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:markers_detected __markers_detected) nil)
    ((:markers_number __markers_number) 0)
    ((:marker_id __marker_id) (make-array 0 :initial-element 0 :element-type :integer))
    ((:marker_rotvec_x __marker_rotvec_x) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:marker_rotvec_y __marker_rotvec_y) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:marker_rotvec_z __marker_rotvec_z) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:marker_transvec_x __marker_transvec_x) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:marker_transvec_y __marker_transvec_y) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:marker_transvec_z __marker_transvec_z) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _header __header)
   (setq _markers_detected __markers_detected)
   (setq _markers_number (round __markers_number))
   (setq _marker_id __marker_id)
   (setq _marker_rotvec_x __marker_rotvec_x)
   (setq _marker_rotvec_y __marker_rotvec_y)
   (setq _marker_rotvec_z __marker_rotvec_z)
   (setq _marker_transvec_x __marker_transvec_x)
   (setq _marker_transvec_y __marker_transvec_y)
   (setq _marker_transvec_z __marker_transvec_z)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:markers_detected
   (&optional __markers_detected)
   (if __markers_detected (setq _markers_detected __markers_detected)) _markers_detected)
  (:markers_number
   (&optional __markers_number)
   (if __markers_number (setq _markers_number __markers_number)) _markers_number)
  (:marker_id
   (&optional __marker_id)
   (if __marker_id (setq _marker_id __marker_id)) _marker_id)
  (:marker_rotvec_x
   (&optional __marker_rotvec_x)
   (if __marker_rotvec_x (setq _marker_rotvec_x __marker_rotvec_x)) _marker_rotvec_x)
  (:marker_rotvec_y
   (&optional __marker_rotvec_y)
   (if __marker_rotvec_y (setq _marker_rotvec_y __marker_rotvec_y)) _marker_rotvec_y)
  (:marker_rotvec_z
   (&optional __marker_rotvec_z)
   (if __marker_rotvec_z (setq _marker_rotvec_z __marker_rotvec_z)) _marker_rotvec_z)
  (:marker_transvec_x
   (&optional __marker_transvec_x)
   (if __marker_transvec_x (setq _marker_transvec_x __marker_transvec_x)) _marker_transvec_x)
  (:marker_transvec_y
   (&optional __marker_transvec_y)
   (if __marker_transvec_y (setq _marker_transvec_y __marker_transvec_y)) _marker_transvec_y)
  (:marker_transvec_z
   (&optional __marker_transvec_z)
   (if __marker_transvec_z (setq _marker_transvec_z __marker_transvec_z)) _marker_transvec_z)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; bool _markers_detected
    1
    ;; uint16 _markers_number
    2
    ;; int8[] _marker_id
    (* 1    (length _marker_id)) 4
    ;; float64[] _marker_rotvec_x
    (* 8    (length _marker_rotvec_x)) 4
    ;; float64[] _marker_rotvec_y
    (* 8    (length _marker_rotvec_y)) 4
    ;; float64[] _marker_rotvec_z
    (* 8    (length _marker_rotvec_z)) 4
    ;; float64[] _marker_transvec_x
    (* 8    (length _marker_transvec_x)) 4
    ;; float64[] _marker_transvec_y
    (* 8    (length _marker_transvec_y)) 4
    ;; float64[] _marker_transvec_z
    (* 8    (length _marker_transvec_z)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; bool _markers_detected
       (if _markers_detected (write-byte -1 s) (write-byte 0 s))
     ;; uint16 _markers_number
       (write-word _markers_number s)
     ;; int8[] _marker_id
     (write-long (length _marker_id) s)
     (dotimes (i (length _marker_id))
       (write-byte (elt _marker_id i) s)
       )
     ;; float64[] _marker_rotvec_x
     (write-long (length _marker_rotvec_x) s)
     (dotimes (i (length _marker_rotvec_x))
       (sys::poke (elt _marker_rotvec_x i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _marker_rotvec_y
     (write-long (length _marker_rotvec_y) s)
     (dotimes (i (length _marker_rotvec_y))
       (sys::poke (elt _marker_rotvec_y i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _marker_rotvec_z
     (write-long (length _marker_rotvec_z) s)
     (dotimes (i (length _marker_rotvec_z))
       (sys::poke (elt _marker_rotvec_z i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _marker_transvec_x
     (write-long (length _marker_transvec_x) s)
     (dotimes (i (length _marker_transvec_x))
       (sys::poke (elt _marker_transvec_x i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _marker_transvec_y
     (write-long (length _marker_transvec_y) s)
     (dotimes (i (length _marker_transvec_y))
       (sys::poke (elt _marker_transvec_y i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _marker_transvec_z
     (write-long (length _marker_transvec_z) s)
     (dotimes (i (length _marker_transvec_z))
       (sys::poke (elt _marker_transvec_z i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; bool _markers_detected
     (setq _markers_detected (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; uint16 _markers_number
     (setq _markers_number (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int8[] _marker_id
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _marker_id (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _marker_id i) (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> (elt _marker_id i) 127) (setf (elt _marker_id i) (- (elt _marker_id i) 256)))
     ))
   ;; float64[] _marker_rotvec_x
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _marker_rotvec_x (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _marker_rotvec_x i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _marker_rotvec_y
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _marker_rotvec_y (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _marker_rotvec_y i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _marker_rotvec_z
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _marker_rotvec_z (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _marker_rotvec_z i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _marker_transvec_x
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _marker_transvec_x (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _marker_transvec_x i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _marker_transvec_y
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _marker_transvec_y (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _marker_transvec_y i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _marker_transvec_z
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _marker_transvec_z (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _marker_transvec_z i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;;
   self)
  )

(setf (get vision::Markers :md5sum-) "1a85c45b00f3e0388fd3fe5a952433ae")
(setf (get vision::Markers :datatype-) "vision/Markers")
(setf (get vision::Markers :definition-)
      "Header header
bool markers_detected
uint16 markers_number  #all together, if more than two seen, placed from closer two to further
int8[] marker_id
float64[] marker_rotvec_x
float64[] marker_rotvec_y
float64[] marker_rotvec_z
float64[] marker_transvec_x
float64[] marker_transvec_y
float64[] marker_transvec_z

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



(provide :vision/Markers "1a85c45b00f3e0388fd3fe5a952433ae")


