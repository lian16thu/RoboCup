# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from vision/LinesLandmarks.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import vision.msg
import geometry_msgs.msg

class LinesLandmarks(genpy.Message):
  _md5sum = "fb66f810489cc88ff8dd15c871c9425d"
  _type = "vision/LinesLandmarks"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """Line[] lines
Landmark[] landmarks

================================================================================
MSG: vision/Line
float32 x1 #line start
float32 y1
float32 x2 #line end
float32 y2

================================================================================
MSG: vision/Landmark
geometry_msgs/Pose2D pose        # Pose
uint8 type                       # Type (see localization::field_model::WorldObject::Type)
float32 confidence               # confidence 0..1

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
"""
  __slots__ = ['lines','landmarks']
  _slot_types = ['vision/Line[]','vision/Landmark[]']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       lines,landmarks

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(LinesLandmarks, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.lines is None:
        self.lines = []
      if self.landmarks is None:
        self.landmarks = []
    else:
      self.lines = []
      self.landmarks = []

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
      length = len(self.lines)
      buff.write(_struct_I.pack(length))
      for val1 in self.lines:
        _x = val1
        buff.write(_get_struct_4f().pack(_x.x1, _x.y1, _x.x2, _x.y2))
      length = len(self.landmarks)
      buff.write(_struct_I.pack(length))
      for val1 in self.landmarks:
        _v1 = val1.pose
        _x = _v1
        buff.write(_get_struct_3d().pack(_x.x, _x.y, _x.theta))
        _x = val1
        buff.write(_get_struct_Bf().pack(_x.type, _x.confidence))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      if self.lines is None:
        self.lines = None
      if self.landmarks is None:
        self.landmarks = None
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.lines = []
      for i in range(0, length):
        val1 = vision.msg.Line()
        _x = val1
        start = end
        end += 16
        (_x.x1, _x.y1, _x.x2, _x.y2,) = _get_struct_4f().unpack(str[start:end])
        self.lines.append(val1)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.landmarks = []
      for i in range(0, length):
        val1 = vision.msg.Landmark()
        _v2 = val1.pose
        _x = _v2
        start = end
        end += 24
        (_x.x, _x.y, _x.theta,) = _get_struct_3d().unpack(str[start:end])
        _x = val1
        start = end
        end += 5
        (_x.type, _x.confidence,) = _get_struct_Bf().unpack(str[start:end])
        self.landmarks.append(val1)
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
      length = len(self.lines)
      buff.write(_struct_I.pack(length))
      for val1 in self.lines:
        _x = val1
        buff.write(_get_struct_4f().pack(_x.x1, _x.y1, _x.x2, _x.y2))
      length = len(self.landmarks)
      buff.write(_struct_I.pack(length))
      for val1 in self.landmarks:
        _v3 = val1.pose
        _x = _v3
        buff.write(_get_struct_3d().pack(_x.x, _x.y, _x.theta))
        _x = val1
        buff.write(_get_struct_Bf().pack(_x.type, _x.confidence))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      if self.lines is None:
        self.lines = None
      if self.landmarks is None:
        self.landmarks = None
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.lines = []
      for i in range(0, length):
        val1 = vision.msg.Line()
        _x = val1
        start = end
        end += 16
        (_x.x1, _x.y1, _x.x2, _x.y2,) = _get_struct_4f().unpack(str[start:end])
        self.lines.append(val1)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.landmarks = []
      for i in range(0, length):
        val1 = vision.msg.Landmark()
        _v4 = val1.pose
        _x = _v4
        start = end
        end += 24
        (_x.x, _x.y, _x.theta,) = _get_struct_3d().unpack(str[start:end])
        _x = val1
        start = end
        end += 5
        (_x.type, _x.confidence,) = _get_struct_Bf().unpack(str[start:end])
        self.landmarks.append(val1)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_4f = None
def _get_struct_4f():
    global _struct_4f
    if _struct_4f is None:
        _struct_4f = struct.Struct("<4f")
    return _struct_4f
_struct_Bf = None
def _get_struct_Bf():
    global _struct_Bf
    if _struct_Bf is None:
        _struct_Bf = struct.Struct("<Bf")
    return _struct_Bf
_struct_3d = None
def _get_struct_3d():
    global _struct_3d
    if _struct_3d is None:
        _struct_3d = struct.Struct("<3d")
    return _struct_3d