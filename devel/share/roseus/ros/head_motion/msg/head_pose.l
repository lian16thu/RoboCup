;; Auto-generated. Do not edit!


(when (boundp 'head_motion::head_pose)
  (if (not (find-package "HEAD_MOTION"))
    (make-package "HEAD_MOTION"))
  (shadow 'head_pose (find-package "HEAD_MOTION")))
(unless (find-package "HEAD_MOTION::HEAD_POSE")
  (make-package "HEAD_MOTION::HEAD_POSE"))

(in-package "ROS")
;;//! \htmlinclude head_pose.msg.html


(defclass head_motion::head_pose
  :super ros::object
  :slots (_yaw _pitch ))

(defmethod head_motion::head_pose
  (:init
   (&key
    ((:yaw __yaw) 0)
    ((:pitch __pitch) 0)
    )
   (send-super :init)
   (setq _yaw (round __yaw))
   (setq _pitch (round __pitch))
   self)
  (:yaw
   (&optional __yaw)
   (if __yaw (setq _yaw __yaw)) _yaw)
  (:pitch
   (&optional __pitch)
   (if __pitch (setq _pitch __pitch)) _pitch)
  (:serialization-length
   ()
   (+
    ;; int64 _yaw
    8
    ;; int64 _pitch
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int64 _yaw
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _yaw (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _yaw) (= (length (_yaw . bv)) 2)) ;; bignum
              (write-long (ash (elt (_yaw . bv) 0) 0) s)
              (write-long (ash (elt (_yaw . bv) 1) -1) s))
             ((and (class _yaw) (= (length (_yaw . bv)) 1)) ;; big1
              (write-long (elt (_yaw . bv) 0) s)
              (write-long (if (>= _yaw 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _yaw s)(write-long (if (>= _yaw 0) 0 #xffffffff) s)))
     ;; int64 _pitch
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _pitch (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _pitch) (= (length (_pitch . bv)) 2)) ;; bignum
              (write-long (ash (elt (_pitch . bv) 0) 0) s)
              (write-long (ash (elt (_pitch . bv) 1) -1) s))
             ((and (class _pitch) (= (length (_pitch . bv)) 1)) ;; big1
              (write-long (elt (_pitch . bv) 0) s)
              (write-long (if (>= _pitch 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _pitch s)(write-long (if (>= _pitch 0) 0 #xffffffff) s)))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int64 _yaw
#+(or :alpha :irix6 :x86_64)
      (setf _yaw (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _yaw (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; int64 _pitch
#+(or :alpha :irix6 :x86_64)
      (setf _pitch (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _pitch (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;;
   self)
  )

(setf (get head_motion::head_pose :md5sum-) "a639ba30f63c6c2e9b511b1c6df08e3a")
(setf (get head_motion::head_pose :datatype-) "head_motion/head_pose")
(setf (get head_motion::head_pose :definition-)
      "int64 yaw
int64 pitch

")



(provide :head_motion/head_pose "a639ba30f63c6c2e9b511b1c6df08e3a")


