# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from vision/Obstacle.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class Obstacle(genpy.Message):
  _md5sum = "b7ab1788ead7712018d8f4217af3a9ee"
  _type = "vision/Obstacle"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """uint8 bObstacleWasSeen

uint16 iLeftEdgeInImageX
uint16 iLeftEdgeInImageY
uint16 iRightEdgeInImageX
uint16 iRightEdgeInImageY
uint16 iHeightInImage

uint16 inumber # apart from first obstacle, if there were 2 obstacles found, this is 1
uint16[] iOthersLeftEdgeInImageX
uint16[] iOthersRightEdgeInImageX
uint16[] iOthersInImageY  # the same Y for both left and right
uint16[] iOthersHeightInImage
"""
  __slots__ = ['bObstacleWasSeen','iLeftEdgeInImageX','iLeftEdgeInImageY','iRightEdgeInImageX','iRightEdgeInImageY','iHeightInImage','inumber','iOthersLeftEdgeInImageX','iOthersRightEdgeInImageX','iOthersInImageY','iOthersHeightInImage']
  _slot_types = ['uint8','uint16','uint16','uint16','uint16','uint16','uint16','uint16[]','uint16[]','uint16[]','uint16[]']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       bObstacleWasSeen,iLeftEdgeInImageX,iLeftEdgeInImageY,iRightEdgeInImageX,iRightEdgeInImageY,iHeightInImage,inumber,iOthersLeftEdgeInImageX,iOthersRightEdgeInImageX,iOthersInImageY,iOthersHeightInImage

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(Obstacle, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.bObstacleWasSeen is None:
        self.bObstacleWasSeen = 0
      if self.iLeftEdgeInImageX is None:
        self.iLeftEdgeInImageX = 0
      if self.iLeftEdgeInImageY is None:
        self.iLeftEdgeInImageY = 0
      if self.iRightEdgeInImageX is None:
        self.iRightEdgeInImageX = 0
      if self.iRightEdgeInImageY is None:
        self.iRightEdgeInImageY = 0
      if self.iHeightInImage is None:
        self.iHeightInImage = 0
      if self.inumber is None:
        self.inumber = 0
      if self.iOthersLeftEdgeInImageX is None:
        self.iOthersLeftEdgeInImageX = []
      if self.iOthersRightEdgeInImageX is None:
        self.iOthersRightEdgeInImageX = []
      if self.iOthersInImageY is None:
        self.iOthersInImageY = []
      if self.iOthersHeightInImage is None:
        self.iOthersHeightInImage = []
    else:
      self.bObstacleWasSeen = 0
      self.iLeftEdgeInImageX = 0
      self.iLeftEdgeInImageY = 0
      self.iRightEdgeInImageX = 0
      self.iRightEdgeInImageY = 0
      self.iHeightInImage = 0
      self.inumber = 0
      self.iOthersLeftEdgeInImageX = []
      self.iOthersRightEdgeInImageX = []
      self.iOthersInImageY = []
      self.iOthersHeightInImage = []

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
      buff.write(_get_struct_B6H().pack(_x.bObstacleWasSeen, _x.iLeftEdgeInImageX, _x.iLeftEdgeInImageY, _x.iRightEdgeInImageX, _x.iRightEdgeInImageY, _x.iHeightInImage, _x.inumber))
      length = len(self.iOthersLeftEdgeInImageX)
      buff.write(_struct_I.pack(length))
      pattern = '<%sH'%length
      buff.write(struct.pack(pattern, *self.iOthersLeftEdgeInImageX))
      length = len(self.iOthersRightEdgeInImageX)
      buff.write(_struct_I.pack(length))
      pattern = '<%sH'%length
      buff.write(struct.pack(pattern, *self.iOthersRightEdgeInImageX))
      length = len(self.iOthersInImageY)
      buff.write(_struct_I.pack(length))
      pattern = '<%sH'%length
      buff.write(struct.pack(pattern, *self.iOthersInImageY))
      length = len(self.iOthersHeightInImage)
      buff.write(_struct_I.pack(length))
      pattern = '<%sH'%length
      buff.write(struct.pack(pattern, *self.iOthersHeightInImage))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      end = 0
      _x = self
      start = end
      end += 13
      (_x.bObstacleWasSeen, _x.iLeftEdgeInImageX, _x.iLeftEdgeInImageY, _x.iRightEdgeInImageX, _x.iRightEdgeInImageY, _x.iHeightInImage, _x.inumber,) = _get_struct_B6H().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sH'%length
      start = end
      end += struct.calcsize(pattern)
      self.iOthersLeftEdgeInImageX = struct.unpack(pattern, str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sH'%length
      start = end
      end += struct.calcsize(pattern)
      self.iOthersRightEdgeInImageX = struct.unpack(pattern, str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sH'%length
      start = end
      end += struct.calcsize(pattern)
      self.iOthersInImageY = struct.unpack(pattern, str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sH'%length
      start = end
      end += struct.calcsize(pattern)
      self.iOthersHeightInImage = struct.unpack(pattern, str[start:end])
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
      buff.write(_get_struct_B6H().pack(_x.bObstacleWasSeen, _x.iLeftEdgeInImageX, _x.iLeftEdgeInImageY, _x.iRightEdgeInImageX, _x.iRightEdgeInImageY, _x.iHeightInImage, _x.inumber))
      length = len(self.iOthersLeftEdgeInImageX)
      buff.write(_struct_I.pack(length))
      pattern = '<%sH'%length
      buff.write(self.iOthersLeftEdgeInImageX.tostring())
      length = len(self.iOthersRightEdgeInImageX)
      buff.write(_struct_I.pack(length))
      pattern = '<%sH'%length
      buff.write(self.iOthersRightEdgeInImageX.tostring())
      length = len(self.iOthersInImageY)
      buff.write(_struct_I.pack(length))
      pattern = '<%sH'%length
      buff.write(self.iOthersInImageY.tostring())
      length = len(self.iOthersHeightInImage)
      buff.write(_struct_I.pack(length))
      pattern = '<%sH'%length
      buff.write(self.iOthersHeightInImage.tostring())
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      end = 0
      _x = self
      start = end
      end += 13
      (_x.bObstacleWasSeen, _x.iLeftEdgeInImageX, _x.iLeftEdgeInImageY, _x.iRightEdgeInImageX, _x.iRightEdgeInImageY, _x.iHeightInImage, _x.inumber,) = _get_struct_B6H().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sH'%length
      start = end
      end += struct.calcsize(pattern)
      self.iOthersLeftEdgeInImageX = numpy.frombuffer(str[start:end], dtype=numpy.uint16, count=length)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sH'%length
      start = end
      end += struct.calcsize(pattern)
      self.iOthersRightEdgeInImageX = numpy.frombuffer(str[start:end], dtype=numpy.uint16, count=length)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sH'%length
      start = end
      end += struct.calcsize(pattern)
      self.iOthersInImageY = numpy.frombuffer(str[start:end], dtype=numpy.uint16, count=length)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sH'%length
      start = end
      end += struct.calcsize(pattern)
      self.iOthersHeightInImage = numpy.frombuffer(str[start:end], dtype=numpy.uint16, count=length)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_B6H = None
def _get_struct_B6H():
    global _struct_B6H
    if _struct_B6H is None:
        _struct_B6H = struct.Struct("<B6H")
    return _struct_B6H
