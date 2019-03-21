# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from vision/Goalpost.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import std_msgs.msg

class Goalpost(genpy.Message):
  _md5sum = "c7e85911d22877f743fec7cea6c072ce"
  _type = "vision/Goalpost"
  _has_header = True #flag to mark the presence of a Header object
  _full_text = """Header header
bool goalpost_detected
int8 goalpost_number   # 3 if only see the middle bar of goalpost. then send out the left and right point as left goalpost and right goalpost
float64 goalpost_left_range
float64 goalpost_left_bearing
float64 goalpost_right_range
float64 goalpost_right_bearing
float64 goalpost_center_range
float64 goalpost_center_bearing

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
"""
  __slots__ = ['header','goalpost_detected','goalpost_number','goalpost_left_range','goalpost_left_bearing','goalpost_right_range','goalpost_right_bearing','goalpost_center_range','goalpost_center_bearing']
  _slot_types = ['std_msgs/Header','bool','int8','float64','float64','float64','float64','float64','float64']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       header,goalpost_detected,goalpost_number,goalpost_left_range,goalpost_left_bearing,goalpost_right_range,goalpost_right_bearing,goalpost_center_range,goalpost_center_bearing

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(Goalpost, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.goalpost_detected is None:
        self.goalpost_detected = False
      if self.goalpost_number is None:
        self.goalpost_number = 0
      if self.goalpost_left_range is None:
        self.goalpost_left_range = 0.
      if self.goalpost_left_bearing is None:
        self.goalpost_left_bearing = 0.
      if self.goalpost_right_range is None:
        self.goalpost_right_range = 0.
      if self.goalpost_right_bearing is None:
        self.goalpost_right_bearing = 0.
      if self.goalpost_center_range is None:
        self.goalpost_center_range = 0.
      if self.goalpost_center_bearing is None:
        self.goalpost_center_bearing = 0.
    else:
      self.header = std_msgs.msg.Header()
      self.goalpost_detected = False
      self.goalpost_number = 0
      self.goalpost_left_range = 0.
      self.goalpost_left_bearing = 0.
      self.goalpost_right_range = 0.
      self.goalpost_right_bearing = 0.
      self.goalpost_center_range = 0.
      self.goalpost_center_bearing = 0.

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
      buff.write(_get_struct_Bb6d().pack(_x.goalpost_detected, _x.goalpost_number, _x.goalpost_left_range, _x.goalpost_left_bearing, _x.goalpost_right_range, _x.goalpost_right_bearing, _x.goalpost_center_range, _x.goalpost_center_bearing))
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
      end += 50
      (_x.goalpost_detected, _x.goalpost_number, _x.goalpost_left_range, _x.goalpost_left_bearing, _x.goalpost_right_range, _x.goalpost_right_bearing, _x.goalpost_center_range, _x.goalpost_center_bearing,) = _get_struct_Bb6d().unpack(str[start:end])
      self.goalpost_detected = bool(self.goalpost_detected)
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
      buff.write(_get_struct_Bb6d().pack(_x.goalpost_detected, _x.goalpost_number, _x.goalpost_left_range, _x.goalpost_left_bearing, _x.goalpost_right_range, _x.goalpost_right_bearing, _x.goalpost_center_range, _x.goalpost_center_bearing))
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
      end += 50
      (_x.goalpost_detected, _x.goalpost_number, _x.goalpost_left_range, _x.goalpost_left_bearing, _x.goalpost_right_range, _x.goalpost_right_bearing, _x.goalpost_center_range, _x.goalpost_center_bearing,) = _get_struct_Bb6d().unpack(str[start:end])
      self.goalpost_detected = bool(self.goalpost_detected)
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
_struct_Bb6d = None
def _get_struct_Bb6d():
    global _struct_Bb6d
    if _struct_Bb6d is None:
        _struct_Bb6d = struct.Struct("<Bb6d")
    return _struct_Bb6d
