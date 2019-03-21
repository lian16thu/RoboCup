;; Auto-generated. Do not edit!


(when (boundp 'gamecontroller::gameControl)
  (if (not (find-package "GAMECONTROLLER"))
    (make-package "GAMECONTROLLER"))
  (shadow 'gameControl (find-package "GAMECONTROLLER")))
(unless (find-package "GAMECONTROLLER::GAMECONTROL")
  (make-package "GAMECONTROLLER::GAMECONTROL"))

(in-package "ROS")
;;//! \htmlinclude gameControl.msg.html


(defclass gamecontroller::gameControl
  :super ros::object
  :slots (_gameType _state _firstHalf _kickOffTeam _secondaryState _secondaryStateTeam _secondaryStateInfo _dropInTeam _dropInTime _secsRemaining _secondaryTime _score _penaltyShot _singleShots _penalty _secsTillUnpenalised _yellowCardCount _redCardCount ))

(defmethod gamecontroller::gameControl
  (:init
   (&key
    ((:gameType __gameType) 0)
    ((:state __state) 0)
    ((:firstHalf __firstHalf) 0)
    ((:kickOffTeam __kickOffTeam) 0)
    ((:secondaryState __secondaryState) 0)
    ((:secondaryStateTeam __secondaryStateTeam) 0)
    ((:secondaryStateInfo __secondaryStateInfo) 0)
    ((:dropInTeam __dropInTeam) 0)
    ((:dropInTime __dropInTime) 0)
    ((:secsRemaining __secsRemaining) 0)
    ((:secondaryTime __secondaryTime) 0)
    ((:score __score) 0)
    ((:penaltyShot __penaltyShot) 0)
    ((:singleShots __singleShots) 0)
    ((:penalty __penalty) 0)
    ((:secsTillUnpenalised __secsTillUnpenalised) 0)
    ((:yellowCardCount __yellowCardCount) 0)
    ((:redCardCount __redCardCount) 0)
    )
   (send-super :init)
   (setq _gameType (round __gameType))
   (setq _state (round __state))
   (setq _firstHalf (round __firstHalf))
   (setq _kickOffTeam (round __kickOffTeam))
   (setq _secondaryState (round __secondaryState))
   (setq _secondaryStateTeam (round __secondaryStateTeam))
   (setq _secondaryStateInfo (round __secondaryStateInfo))
   (setq _dropInTeam (round __dropInTeam))
   (setq _dropInTime (round __dropInTime))
   (setq _secsRemaining (round __secsRemaining))
   (setq _secondaryTime (round __secondaryTime))
   (setq _score (round __score))
   (setq _penaltyShot (round __penaltyShot))
   (setq _singleShots (round __singleShots))
   (setq _penalty (round __penalty))
   (setq _secsTillUnpenalised (round __secsTillUnpenalised))
   (setq _yellowCardCount (round __yellowCardCount))
   (setq _redCardCount (round __redCardCount))
   self)
  (:gameType
   (&optional __gameType)
   (if __gameType (setq _gameType __gameType)) _gameType)
  (:state
   (&optional __state)
   (if __state (setq _state __state)) _state)
  (:firstHalf
   (&optional __firstHalf)
   (if __firstHalf (setq _firstHalf __firstHalf)) _firstHalf)
  (:kickOffTeam
   (&optional __kickOffTeam)
   (if __kickOffTeam (setq _kickOffTeam __kickOffTeam)) _kickOffTeam)
  (:secondaryState
   (&optional __secondaryState)
   (if __secondaryState (setq _secondaryState __secondaryState)) _secondaryState)
  (:secondaryStateTeam
   (&optional __secondaryStateTeam)
   (if __secondaryStateTeam (setq _secondaryStateTeam __secondaryStateTeam)) _secondaryStateTeam)
  (:secondaryStateInfo
   (&optional __secondaryStateInfo)
   (if __secondaryStateInfo (setq _secondaryStateInfo __secondaryStateInfo)) _secondaryStateInfo)
  (:dropInTeam
   (&optional __dropInTeam)
   (if __dropInTeam (setq _dropInTeam __dropInTeam)) _dropInTeam)
  (:dropInTime
   (&optional __dropInTime)
   (if __dropInTime (setq _dropInTime __dropInTime)) _dropInTime)
  (:secsRemaining
   (&optional __secsRemaining)
   (if __secsRemaining (setq _secsRemaining __secsRemaining)) _secsRemaining)
  (:secondaryTime
   (&optional __secondaryTime)
   (if __secondaryTime (setq _secondaryTime __secondaryTime)) _secondaryTime)
  (:score
   (&optional __score)
   (if __score (setq _score __score)) _score)
  (:penaltyShot
   (&optional __penaltyShot)
   (if __penaltyShot (setq _penaltyShot __penaltyShot)) _penaltyShot)
  (:singleShots
   (&optional __singleShots)
   (if __singleShots (setq _singleShots __singleShots)) _singleShots)
  (:penalty
   (&optional __penalty)
   (if __penalty (setq _penalty __penalty)) _penalty)
  (:secsTillUnpenalised
   (&optional __secsTillUnpenalised)
   (if __secsTillUnpenalised (setq _secsTillUnpenalised __secsTillUnpenalised)) _secsTillUnpenalised)
  (:yellowCardCount
   (&optional __yellowCardCount)
   (if __yellowCardCount (setq _yellowCardCount __yellowCardCount)) _yellowCardCount)
  (:redCardCount
   (&optional __redCardCount)
   (if __redCardCount (setq _redCardCount __redCardCount)) _redCardCount)
  (:serialization-length
   ()
   (+
    ;; uint8 _gameType
    1
    ;; uint8 _state
    1
    ;; uint8 _firstHalf
    1
    ;; uint8 _kickOffTeam
    1
    ;; uint8 _secondaryState
    1
    ;; uint8 _secondaryStateTeam
    1
    ;; uint8 _secondaryStateInfo
    1
    ;; uint8 _dropInTeam
    1
    ;; uint16 _dropInTime
    2
    ;; uint16 _secsRemaining
    2
    ;; uint16 _secondaryTime
    2
    ;; uint8 _score
    1
    ;; uint8 _penaltyShot
    1
    ;; uint16 _singleShots
    2
    ;; uint8 _penalty
    1
    ;; uint8 _secsTillUnpenalised
    1
    ;; uint8 _yellowCardCount
    1
    ;; uint8 _redCardCount
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8 _gameType
       (write-byte _gameType s)
     ;; uint8 _state
       (write-byte _state s)
     ;; uint8 _firstHalf
       (write-byte _firstHalf s)
     ;; uint8 _kickOffTeam
       (write-byte _kickOffTeam s)
     ;; uint8 _secondaryState
       (write-byte _secondaryState s)
     ;; uint8 _secondaryStateTeam
       (write-byte _secondaryStateTeam s)
     ;; uint8 _secondaryStateInfo
       (write-byte _secondaryStateInfo s)
     ;; uint8 _dropInTeam
       (write-byte _dropInTeam s)
     ;; uint16 _dropInTime
       (write-word _dropInTime s)
     ;; uint16 _secsRemaining
       (write-word _secsRemaining s)
     ;; uint16 _secondaryTime
       (write-word _secondaryTime s)
     ;; uint8 _score
       (write-byte _score s)
     ;; uint8 _penaltyShot
       (write-byte _penaltyShot s)
     ;; uint16 _singleShots
       (write-word _singleShots s)
     ;; uint8 _penalty
       (write-byte _penalty s)
     ;; uint8 _secsTillUnpenalised
       (write-byte _secsTillUnpenalised s)
     ;; uint8 _yellowCardCount
       (write-byte _yellowCardCount s)
     ;; uint8 _redCardCount
       (write-byte _redCardCount s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8 _gameType
     (setq _gameType (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _state
     (setq _state (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _firstHalf
     (setq _firstHalf (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _kickOffTeam
     (setq _kickOffTeam (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _secondaryState
     (setq _secondaryState (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _secondaryStateTeam
     (setq _secondaryStateTeam (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _secondaryStateInfo
     (setq _secondaryStateInfo (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _dropInTeam
     (setq _dropInTeam (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint16 _dropInTime
     (setq _dropInTime (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint16 _secsRemaining
     (setq _secsRemaining (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint16 _secondaryTime
     (setq _secondaryTime (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint8 _score
     (setq _score (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _penaltyShot
     (setq _penaltyShot (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint16 _singleShots
     (setq _singleShots (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint8 _penalty
     (setq _penalty (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _secsTillUnpenalised
     (setq _secsTillUnpenalised (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _yellowCardCount
     (setq _yellowCardCount (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _redCardCount
     (setq _redCardCount (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(setf (get gamecontroller::gameControl :md5sum-) "8187bc9e4bf6fa1896498e321b213f47")
(setf (get gamecontroller::gameControl :datatype-) "gamecontroller/gameControl")
(setf (get gamecontroller::gameControl :definition-)
      "uint8 gameType
uint8 state
uint8 firstHalf
uint8 kickOffTeam
uint8 secondaryState
uint8 secondaryStateTeam
uint8 secondaryStateInfo
uint8 dropInTeam
uint16 dropInTime
uint16 secsRemaining
uint16 secondaryTime

uint8 score
uint8 penaltyShot
uint16 singleShots

uint8 penalty
uint8 secsTillUnpenalised
uint8 yellowCardCount
uint8 redCardCount
")



(provide :gamecontroller/gameControl "8187bc9e4bf6fa1896498e321b213f47")


