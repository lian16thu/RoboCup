# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from vision/Landmarks.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import std_msgs.msg

class Landmarks(genpy.Message):
  _md5sum = "6b999f1904ab908fbb4ce607a9071d78"
  _type = "vision/Landmarks"
  _has_header = True #flag to mark the presence of a Header object
  _full_text = """Header header
uint8 landmark_number  
uint8[20] landmark_type     	# 1 is X, 2 is T, 3 is L, 4 is unknown, 5 is penaltypoint
float64[20] landmark_range                 
float64[20] landmark_bearing               
float32[20] landmark_confidence               # confidence 0..1

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
  __slots__ = ['header','landmark_number','landmark_type','landmark_range','landmark_bearing','landmark_confidence']
  _slot_types = ['std_msgs/Header','uint8','uint8[20]','float64[20]','float64[20]','float32[20]']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       header,landmark_number,landmark_type,landmark_range,landmark_bearing,landmark_confidence

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(Landmarks, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.landmark_number is None:
        self.landmark_number = 0
      if self.landmark_type is None:
        self.landmark_type = b'\0'*20
      if self.landmark_range is None:
        self.landmark_range = [0.] * 20
      if self.landmark_bearing is None:
        self.landmark_bearing = [0.] * 20
      if self.landmark_confidence is None:
        self.landmark_confidence = [0.] * 20
    else:
      self.header = std_msgs.msg.Header()
      self.landmark_number = 0
      self.landmark_type = b'\0'*20
      self.landmark_range = [0.] * 20
      self.landmark_bearing = [0.] * 20
      self.landmark_confidence = [0.] * 20

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
      buff.write(_get_struct_B().pack(self.landmark_number))
      _x = self.landmark_type
      # - if encoded as a list instead, serialize as bytes instead of string
      if type(_x) in [list, tuple]:
        buff.write(_get_struct_20B().pack(*_x))
      else:
        buff.write(_get_struct_20s().pack(_x))
      buff.write(_get_struct_20d().pack(*self.landmark_range))
      buff.write(_get_struct_20d().pack(*self.landmark_bearing))
      buff.write(_get_struct_20f().pack(*self.landmark_confidence))
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
      start = end
      end += 1
      (self.landmark_number,) = _get_struct_B().unpack(str[start:end])
      start = end
      end += 20
      self.landmark_type = str[start:end]
      start = end
      end += 160
      self.landmark_range = _get_struct_20d().unpack(str[start:end])
      start = end
      end += 160
      self.landmark_bearing = _get_struct_20d().unpack(str[start:end])
      start = end
      end += 80
      self.landmark_confidence = _get_struct_20f().unpack(str[start:end])
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
      buff.write(_get_struct_B().pack(self.landmark_number))
      _x = self.landmark_type
      # - if encoded as a list instead, serialize as bytes instead of string
      if type(_x) in [list, tuple]:
        buff.write(_get_struct_20B().pack(*_x))
      else:
        buff.write(_get_struct_20s().pack(_x))
      buff.write(self.landmark_range.tostring())
      buff.write(self.landmark_bearing.tostring())
      buff.write(self.landmark_confidence.tostring())
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
      start = end
      end += 1
      (self.landmark_number,) = _get_struct_B().unpack(str[start:end])
      start = end
      end += 20
      self.landmark_type = str[start:end]
      start = end
      end += 160
      self.landmark_range = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=20)
      start = end
      end += 160
      self.landmark_bearing = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=20)
      start = end
      end += 80
      self.landmark_confidence = numpy.frombuffer(str[start:end], dtype=numpy.float32, count=20)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_20s = None
def _get_struct_20s():
    global _struct_20s
    if _struct_20s is None:
        _struct_20s = struct.Struct("<20s")
    return _struct_20s
_struct_B = None
def _get_struct_B():
    global _struct_B
    if _struct_B is None:
        _struct_B = struct.Struct("<B")
    return _struct_B
_struct_3I = None
def _get_struct_3I():
    global _struct_3I
    if _struct_3I is None:
        _struct_3I = struct.Struct("<3I")
    return _struct_3I
_struct_20B = None
def _get_struct_20B():
    global _struct_20B
    if _struct_20B is None:
        _struct_20B = struct.Struct("<20B")
    return _struct_20B
_struct_20d = None
def _get_struct_20d():
    global _struct_20d
    if _struct_20d is None:
        _struct_20d = struct.Struct("<20d")
    return _struct_20d
_struct_20f = None
def _get_struct_20f():
    global _struct_20f
    if _struct_20f is None:
        _struct_20f = struct.Struct("<20f")
    return _struct_20f
