# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from decision/OutputData.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import geometry_msgs.msg
import std_msgs.msg

class OutputData(genpy.Message):
  _md5sum = "bd064faf37c8b9975f133c15e75ac7aa"
  _type = "decision/OutputData"
  _has_header = True #flag to mark the presence of a Header object
  _full_text = """#	data published by the localization node.

std_msgs/Header header 								# for time stamp
geometry_msgs/Pose2D robotPose        				# Pose of the robot according to the particle filter localization	
float32 robotPoseConfidence               			# confidence 0..1 for the robot_pose
bool bBallWasSeen                               	# boolean variable for ball detection
geometry_msgs/Point ballCenterInImage				# coordinates of ball center in the image
float32 ballDistance								# ball distance from robot as seen in the image
float32 ballAngle									# ball angle from center as seen in the image, [-90,90]
geometry_msgs/Point ballCenterOnField				# coordinates of ball center on field relative to robot localization	
bool bObstacleWasSeen                           	# boolean variable for obstacle detection
int32 iObstacleNumber                           	# index of obstacle, if 2 found this is 2
geometry_msgs/Point[] obstacleLeftEndInImage		# coordinates of obstacle left end point in the image
geometry_msgs/Point[] obstacleRightEndInImage		# coordinates of obstacle right end point in the image
float32[] obstacleDistance							# obstacle distance from robot as seen in the image
float32[] obstacleAngle								# obstacle angle from center as seen in the image, [-90,90]
geometry_msgs/Point[] obstacleCenterOnField			# coordinates of obstacle center on field relative to robot localization	
float32[] obstacleRadiusOnField						# estimated radius of obstacle on field
bool bKeeperWasSeen                             	# boolean variable for goalkeeper detection
geometry_msgs/Point keeperLeftStartOnField      	# coordinates of goalkeeper left leg start point on field
geometry_msgs/Point keeperRightEndOnField       	# coordinates of goalkeeper right leg end point on field

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

================================================================================
MSG: geometry_msgs/Pose2D
# Deprecated
# Please use the full 3D pose.

# In general our recommendation is to use a full 3D representation of everything and for 2D specific applications make the appropriate projections into the plane for their calculations but optimally will preserve the 3D information during processing.

# If we have parallel copies of 2D datatypes every UI and other pipeline will end up needing to have dual interfaces to plot everything. And you will end up with not being able to use 3D tools for 2D use cases even if they're completely valid, as you'd have to reimplement it with different inputs and outputs. It's not particularly hard to plot the 2D pose or compute the yaw error for the Pose message and there are already tools and libraries that can do this for you.


# This expresses a position and orientation on a 2D manifold.

float64 x
float64 y
float64 theta

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
"""
  __slots__ = ['header','robotPose','robotPoseConfidence','bBallWasSeen','ballCenterInImage','ballDistance','ballAngle','ballCenterOnField','bObstacleWasSeen','iObstacleNumber','obstacleLeftEndInImage','obstacleRightEndInImage','obstacleDistance','obstacleAngle','obstacleCenterOnField','obstacleRadiusOnField','bKeeperWasSeen','keeperLeftStartOnField','keeperRightEndOnField']
  _slot_types = ['std_msgs/Header','geometry_msgs/Pose2D','float32','bool','geometry_msgs/Point','float32','float32','geometry_msgs/Point','bool','int32','geometry_msgs/Point[]','geometry_msgs/Point[]','float32[]','float32[]','geometry_msgs/Point[]','float32[]','bool','geometry_msgs/Point','geometry_msgs/Point']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       header,robotPose,robotPoseConfidence,bBallWasSeen,ballCenterInImage,ballDistance,ballAngle,ballCenterOnField,bObstacleWasSeen,iObstacleNumber,obstacleLeftEndInImage,obstacleRightEndInImage,obstacleDistance,obstacleAngle,obstacleCenterOnField,obstacleRadiusOnField,bKeeperWasSeen,keeperLeftStartOnField,keeperRightEndOnField

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(OutputData, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.robotPose is None:
        self.robotPose = geometry_msgs.msg.Pose2D()
      if self.robotPoseConfidence is None:
        self.robotPoseConfidence = 0.
      if self.bBallWasSeen is None:
        self.bBallWasSeen = False
      if self.ballCenterInImage is None:
        self.ballCenterInImage = geometry_msgs.msg.Point()
      if self.ballDistance is None:
        self.ballDistance = 0.
      if self.ballAngle is None:
        self.ballAngle = 0.
      if self.ballCenterOnField is None:
        self.ballCenterOnField = geometry_msgs.msg.Point()
      if self.bObstacleWasSeen is None:
        self.bObstacleWasSeen = False
      if self.iObstacleNumber is None:
        self.iObstacleNumber = 0
      if self.obstacleLeftEndInImage is None:
        self.obstacleLeftEndInImage = []
      if self.obstacleRightEndInImage is None:
        self.obstacleRightEndInImage = []
      if self.obstacleDistance is None:
        self.obstacleDistance = []
      if self.obstacleAngle is None:
        self.obstacleAngle = []
      if self.obstacleCenterOnField is None:
        self.obstacleCenterOnField = []
      if self.obstacleRadiusOnField is None:
        self.obstacleRadiusOnField = []
      if self.bKeeperWasSeen is None:
        self.bKeeperWasSeen = False
      if self.keeperLeftStartOnField is None:
        self.keeperLeftStartOnField = geometry_msgs.msg.Point()
      if self.keeperRightEndOnField is None:
        self.keeperRightEndOnField = geometry_msgs.msg.Point()
    else:
      self.header = std_msgs.msg.Header()
      self.robotPose = geometry_msgs.msg.Pose2D()
      self.robotPoseConfidence = 0.
      self.bBallWasSeen = False
      self.ballCenterInImage = geometry_msgs.msg.Point()
      self.ballDistance = 0.
      self.ballAngle = 0.
      self.ballCenterOnField = geometry_msgs.msg.Point()
      self.bObstacleWasSeen = False
      self.iObstacleNumber = 0
      self.obstacleLeftEndInImage = []
      self.obstacleRightEndInImage = []
      self.obstacleDistance = []
      self.obstacleAngle = []
      self.obstacleCenterOnField = []
      self.obstacleRadiusOnField = []
      self.bKeeperWasSeen = False
      self.keeperLeftStartOnField = geometry_msgs.msg.Point()
      self.keeperRightEndOnField = geometry_msgs.msg.Point()

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    :param buff: buffer, ``StringIO``
    """
    try:
      _x = self
      buff.write(_get_struct_3I().pack(_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs))
      _x = self.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_get_struct_3dfB3d2f3dBi().pack(_x.robotPose.x, _x.robotPose.y, _x.robotPose.theta, _x.robotPoseConfidence, _x.bBallWasSeen, _x.ballCenterInImage.x, _x.ballCenterInImage.y, _x.ballCenterInImage.z, _x.ballDistance, _x.ballAngle, _x.ballCenterOnField.x, _x.ballCenterOnField.y, _x.ballCenterOnField.z, _x.bObstacleWasSeen, _x.iObstacleNumber))
      length = len(self.obstacleLeftEndInImage)
      buff.write(_struct_I.pack(length))
      for val1 in self.obstacleLeftEndInImage:
        _x = val1
        buff.write(_get_struct_3d().pack(_x.x, _x.y, _x.z))
      length = len(self.obstacleRightEndInImage)
      buff.write(_struct_I.pack(length))
      for val1 in self.obstacleRightEndInImage:
        _x = val1
        buff.write(_get_struct_3d().pack(_x.x, _x.y, _x.z))
      length = len(self.obstacleDistance)
      buff.write(_struct_I.pack(length))
      pattern = '<%sf'%length
      buff.write(struct.pack(pattern, *self.obstacleDistance))
      length = len(self.obstacleAngle)
      buff.write(_struct_I.pack(length))
      pattern = '<%sf'%length
      buff.write(struct.pack(pattern, *self.obstacleAngle))
      length = len(self.obstacleCenterOnField)
      buff.write(_struct_I.pack(length))
      for val1 in self.obstacleCenterOnField:
        _x = val1
        buff.write(_get_struct_3d().pack(_x.x, _x.y, _x.z))
      length = len(self.obstacleRadiusOnField)
      buff.write(_struct_I.pack(length))
      pattern = '<%sf'%length
      buff.write(struct.pack(pattern, *self.obstacleRadiusOnField))
      _x = self
      buff.write(_get_struct_B6d().pack(_x.bKeeperWasSeen, _x.keeperLeftStartOnField.x, _x.keeperLeftStartOnField.y, _x.keeperLeftStartOnField.z, _x.keeperRightEndOnField.x, _x.keeperRightEndOnField.y, _x.keeperRightEndOnField.z))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.robotPose is None:
        self.robotPose = geometry_msgs.msg.Pose2D()
      if self.ballCenterInImage is None:
        self.ballCenterInImage = geometry_msgs.msg.Point()
      if self.ballCenterOnField is None:
        self.ballCenterOnField = geometry_msgs.msg.Point()
      if self.obstacleLeftEndInImage is None:
        self.obstacleLeftEndInImage = None
      if self.obstacleRightEndInImage is None:
        self.obstacleRightEndInImage = None
      if self.obstacleCenterOnField is None:
        self.obstacleCenterOnField = None
      if self.keeperLeftStartOnField is None:
        self.keeperLeftStartOnField = geometry_msgs.msg.Point()
      if self.keeperRightEndOnField is None:
        self.keeperRightEndOnField = geometry_msgs.msg.Point()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs,) = _get_struct_3I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 90
      (_x.robotPose.x, _x.robotPose.y, _x.robotPose.theta, _x.robotPoseConfidence, _x.bBallWasSeen, _x.ballCenterInImage.x, _x.ballCenterInImage.y, _x.ballCenterInImage.z, _x.ballDistance, _x.ballAngle, _x.ballCenterOnField.x, _x.ballCenterOnField.y, _x.ballCenterOnField.z, _x.bObstacleWasSeen, _x.iObstacleNumber,) = _get_struct_3dfB3d2f3dBi().unpack(str[start:end])
      self.bBallWasSeen = bool(self.bBallWasSeen)
      self.bObstacleWasSeen = bool(self.bObstacleWasSeen)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.obstacleLeftEndInImage = []
      for i in range(0, length):
        val1 = geometry_msgs.msg.Point()
        _x = val1
        start = end
        end += 24
        (_x.x, _x.y, _x.z,) = _get_struct_3d().unpack(str[start:end])
        self.obstacleLeftEndInImage.append(val1)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.obstacleRightEndInImage = []
      for i in range(0, length):
        val1 = geometry_msgs.msg.Point()
        _x = val1
        start = end
        end += 24
        (_x.x, _x.y, _x.z,) = _get_struct_3d().unpack(str[start:end])
        self.obstacleRightEndInImage.append(val1)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sf'%length
      start = end
      end += struct.calcsize(pattern)
      self.obstacleDistance = struct.unpack(pattern, str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sf'%length
      start = end
      end += struct.calcsize(pattern)
      self.obstacleAngle = struct.unpack(pattern, str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.obstacleCenterOnField = []
      for i in range(0, length):
        val1 = geometry_msgs.msg.Point()
        _x = val1
        start = end
        end += 24
        (_x.x, _x.y, _x.z,) = _get_struct_3d().unpack(str[start:end])
        self.obstacleCenterOnField.append(val1)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sf'%length
      start = end
      end += struct.calcsize(pattern)
      self.obstacleRadiusOnField = struct.unpack(pattern, str[start:end])
      _x = self
      start = end
      end += 49
      (_x.bKeeperWasSeen, _x.keeperLeftStartOnField.x, _x.keeperLeftStartOnField.y, _x.keeperLeftStartOnField.z, _x.keeperRightEndOnField.x, _x.keeperRightEndOnField.y, _x.keeperRightEndOnField.z,) = _get_struct_B6d().unpack(str[start:end])
      self.bKeeperWasSeen = bool(self.bKeeperWasSeen)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self
      buff.write(_get_struct_3I().pack(_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs))
      _x = self.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_get_struct_3dfB3d2f3dBi().pack(_x.robotPose.x, _x.robotPose.y, _x.robotPose.theta, _x.robotPoseConfidence, _x.bBallWasSeen, _x.ballCenterInImage.x, _x.ballCenterInImage.y, _x.ballCenterInImage.z, _x.ballDistance, _x.ballAngle, _x.ballCenterOnField.x, _x.ballCenterOnField.y, _x.ballCenterOnField.z, _x.bObstacleWasSeen, _x.iObstacleNumber))
      length = len(self.obstacleLeftEndInImage)
      buff.write(_struct_I.pack(length))
      for val1 in self.obstacleLeftEndInImage:
        _x = val1
        buff.write(_get_struct_3d().pack(_x.x, _x.y, _x.z))
      length = len(self.obstacleRightEndInImage)
      buff.write(_struct_I.pack(length))
      for val1 in self.obstacleRightEndInImage:
        _x = val1
        buff.write(_get_struct_3d().pack(_x.x, _x.y, _x.z))
      length = len(self.obstacleDistance)
      buff.write(_struct_I.pack(length))
      pattern = '<%sf'%length
      buff.write(self.obstacleDistance.tostring())
      length = len(self.obstacleAngle)
      buff.write(_struct_I.pack(length))
      pattern = '<%sf'%length
      buff.write(self.obstacleAngle.tostring())
      length = len(self.obstacleCenterOnField)
      buff.write(_struct_I.pack(length))
      for val1 in self.obstacleCenterOnField:
        _x = val1
        buff.write(_get_struct_3d().pack(_x.x, _x.y, _x.z))
      length = len(self.obstacleRadiusOnField)
      buff.write(_struct_I.pack(length))
      pattern = '<%sf'%length
      buff.write(self.obstacleRadiusOnField.tostring())
      _x = self
      buff.write(_get_struct_B6d().pack(_x.bKeeperWasSeen, _x.keeperLeftStartOnField.x, _x.keeperLeftStartOnField.y, _x.keeperLeftStartOnField.z, _x.keeperRightEndOnField.x, _x.keeperRightEndOnField.y, _x.keeperRightEndOnField.z))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.robotPose is None:
        self.robotPose = geometry_msgs.msg.Pose2D()
      if self.ballCenterInImage is None:
        self.ballCenterInImage = geometry_msgs.msg.Point()
      if self.ballCenterOnField is None:
        self.ballCenterOnField = geometry_msgs.msg.Point()
      if self.obstacleLeftEndInImage is None:
        self.obstacleLeftEndInImage = None
      if self.obstacleRightEndInImage is None:
        self.obstacleRightEndInImage = None
      if self.obstacleCenterOnField is None:
        self.obstacleCenterOnField = None
      if self.keeperLeftStartOnField is None:
        self.keeperLeftStartOnField = geometry_msgs.msg.Point()
      if self.keeperRightEndOnField is None:
        self.keeperRightEndOnField = geometry_msgs.msg.Point()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs,) = _get_struct_3I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 90
      (_x.robotPose.x, _x.robotPose.y, _x.robotPose.theta, _x.robotPoseConfidence, _x.bBallWasSeen, _x.ballCenterInImage.x, _x.ballCenterInImage.y, _x.ballCenterInImage.z, _x.ballDistance, _x.ballAngle, _x.ballCenterOnField.x, _x.ballCenterOnField.y, _x.ballCenterOnField.z, _x.bObstacleWasSeen, _x.iObstacleNumber,) = _get_struct_3dfB3d2f3dBi().unpack(str[start:end])
      self.bBallWasSeen = bool(self.bBallWasSeen)
      self.bObstacleWasSeen = bool(self.bObstacleWasSeen)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.obstacleLeftEndInImage = []
      for i in range(0, length):
        val1 = geometry_msgs.msg.Point()
        _x = val1
        start = end
        end += 24
        (_x.x, _x.y, _x.z,) = _get_struct_3d().unpack(str[start:end])
        self.obstacleLeftEndInImage.append(val1)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.obstacleRightEndInImage = []
      for i in range(0, length):
        val1 = geometry_msgs.msg.Point()
        _x = val1
        start = end
        end += 24
        (_x.x, _x.y, _x.z,) = _get_struct_3d().unpack(str[start:end])
        self.obstacleRightEndInImage.append(val1)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sf'%length
      start = end
      end += struct.calcsize(pattern)
      self.obstacleDistance = numpy.frombuffer(str[start:end], dtype=numpy.float32, count=length)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sf'%length
      start = end
      end += struct.calcsize(pattern)
      self.obstacleAngle = numpy.frombuffer(str[start:end], dtype=numpy.float32, count=length)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.obstacleCenterOnField = []
      for i in range(0, length):
        val1 = geometry_msgs.msg.Point()
        _x = val1
        start = end
        end += 24
        (_x.x, _x.y, _x.z,) = _get_struct_3d().unpack(str[start:end])
        self.obstacleCenterOnField.append(val1)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sf'%length
      start = end
      end += struct.calcsize(pattern)
      self.obstacleRadiusOnField = numpy.frombuffer(str[start:end], dtype=numpy.float32, count=length)
      _x = self
      start = end
      end += 49
      (_x.bKeeperWasSeen, _x.keeperLeftStartOnField.x, _x.keeperLeftStartOnField.y, _x.keeperLeftStartOnField.z, _x.keeperRightEndOnField.x, _x.keeperRightEndOnField.y, _x.keeperRightEndOnField.z,) = _get_struct_B6d().unpack(str[start:end])
      self.bKeeperWasSeen = bool(self.bKeeperWasSeen)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_3I = None
def _get_struct_3I():
    global _struct_3I
    if _struct_3I is None:
        _struct_3I = struct.Struct("<3I")
    return _struct_3I
_struct_B6d = None
def _get_struct_B6d():
    global _struct_B6d
    if _struct_B6d is None:
        _struct_B6d = struct.Struct("<B6d")
    return _struct_B6d
_struct_3dfB3d2f3dBi = None
def _get_struct_3dfB3d2f3dBi():
    global _struct_3dfB3d2f3dBi
    if _struct_3dfB3d2f3dBi is None:
        _struct_3dfB3d2f3dBi = struct.Struct("<3dfB3d2f3dBi")
    return _struct_3dfB3d2f3dBi
_struct_3d = None
def _get_struct_3d():
    global _struct_3d
    if _struct_3d is None:
        _struct_3d = struct.Struct("<3d")
    return _struct_3d
