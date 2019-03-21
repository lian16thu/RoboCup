# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from vision/Landmark.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import geometry_msgs.msg

class Landmark(genpy.Message):
  _md5sum = "45117ab335e578bb32c6e4a315f61f4d"
  _type = "vision/Landmark"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """geometry_msgs/Pose2D pose        # Pose
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
  __slots__ = ['pose','type','confidence']
  _slot_types = ['geometry_msgs/Pose2D','uint8','float32']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       pose,type,confidence

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(Landmark, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.pose is None:
        self.pose = geometry_msgs.msg.Pose2D()
      if self.type is None:
        self.type = 0
      if self.confidence is None:
        self.confidence = 0.
    else:
      self.pose = geometry_msgs.msg.Pose2D()
      self.type = 0
      self.confidence = 0.

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
      buff.write(_get_struct_3dBf().pack(_x.pose.x, _x.pose.y, _x.pose.theta, _x.type, _x.confidence))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      if self.pose is None:
        self.pose = geometry_msgs.msg.Pose2D()
      end = 0
      _x = self
      start = end
      end += 29
      (_x.pose.x, _x.pose.y, _x.pose.theta, _x.type, _x.confidence,) = _get_struct_3dBf().unpack(str[start:end])
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
      buff.write(_get_struct_3dBf().pack(_x.pose.x, _x.pose.y, _x.pose.theta, _x.type, _x.confidence))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      if self.pose is None:
        self.pose = geometry_msgs.msg.Pose2D()
      end = 0
      _x = self
      start = end
      end += 29
      (_x.pose.x, _x.pose.y, _x.pose.theta, _x.type, _x.confidence,) = _get_struct_3dBf().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_3dBf = None
def _get_struct_3dBf():
    global _struct_3dBf
    if _struct_3dBf is None:
        _struct_3dBf = struct.Struct("<3dBf")
    return _struct_3dBf