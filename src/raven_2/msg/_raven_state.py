"""autogenerated by genpy from raven_2/raven_state.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import genpy
import std_msgs.msg

class raven_state(genpy.Message):
  _md5sum = "b35c52beea71553432c57abc395ea0fc"
  _type = "raven_2/raven_state"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """Header      hdr
int32       runlevel
int32       sublevel
int32       last_seq
int32[2]    type
int32[6]    pos
float32[18]   ori
float32[18]   ori_d
int32[6]    pos_d
duration    dt
int32[16]   encVals
float32[16] tau
float32[16] mpos
float32[16] jpos
float32[16] mvel
float32[16] mvel_d
float32[16] jvel
float32[16] mpos_d
float32[16] jpos_d
float32[2]  grasp_d
float32[16] encoffsets
int32[16] current_cmd
char[256] err_msg

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
  __slots__ = ['hdr','runlevel','sublevel','last_seq','type','pos','ori','ori_d','pos_d','dt','encVals','tau','mpos','jpos','mvel','mvel_d','jvel','mpos_d','jpos_d','grasp_d','encoffsets','current_cmd','err_msg']
  _slot_types = ['std_msgs/Header','int32','int32','int32','int32[2]','int32[6]','float32[18]','float32[18]','int32[6]','duration','int32[16]','float32[16]','float32[16]','float32[16]','float32[16]','float32[16]','float32[16]','float32[16]','float32[16]','float32[2]','float32[16]','int32[16]','char[256]']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       hdr,runlevel,sublevel,last_seq,type,pos,ori,ori_d,pos_d,dt,encVals,tau,mpos,jpos,mvel,mvel_d,jvel,mpos_d,jpos_d,grasp_d,encoffsets,current_cmd,err_msg

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(raven_state, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.hdr is None:
        self.hdr = std_msgs.msg.Header()
      if self.runlevel is None:
        self.runlevel = 0
      if self.sublevel is None:
        self.sublevel = 0
      if self.last_seq is None:
        self.last_seq = 0
      if self.type is None:
        self.type = [0,0]
      if self.pos is None:
        self.pos = [0,0,0,0,0,0]
      if self.ori is None:
        self.ori = [0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.]
      if self.ori_d is None:
        self.ori_d = [0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.]
      if self.pos_d is None:
        self.pos_d = [0,0,0,0,0,0]
      if self.dt is None:
        self.dt = genpy.Duration()
      if self.encVals is None:
        self.encVals = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
      if self.tau is None:
        self.tau = [0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.]
      if self.mpos is None:
        self.mpos = [0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.]
      if self.jpos is None:
        self.jpos = [0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.]
      if self.mvel is None:
        self.mvel = [0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.]
      if self.mvel_d is None:
        self.mvel_d = [0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.]
      if self.jvel is None:
        self.jvel = [0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.]
      if self.mpos_d is None:
        self.mpos_d = [0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.]
      if self.jpos_d is None:
        self.jpos_d = [0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.]
      if self.grasp_d is None:
        self.grasp_d = [0.,0.]
      if self.encoffsets is None:
        self.encoffsets = [0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.]
      if self.current_cmd is None:
        self.current_cmd = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
      if self.err_msg is None:
        self.err_msg = chr(0)*256
    else:
      self.hdr = std_msgs.msg.Header()
      self.runlevel = 0
      self.sublevel = 0
      self.last_seq = 0
      self.type = [0,0]
      self.pos = [0,0,0,0,0,0]
      self.ori = [0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.]
      self.ori_d = [0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.]
      self.pos_d = [0,0,0,0,0,0]
      self.dt = genpy.Duration()
      self.encVals = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
      self.tau = [0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.]
      self.mpos = [0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.]
      self.jpos = [0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.]
      self.mvel = [0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.]
      self.mvel_d = [0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.]
      self.jvel = [0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.]
      self.mpos_d = [0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.]
      self.jpos_d = [0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.]
      self.grasp_d = [0.,0.]
      self.encoffsets = [0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.]
      self.current_cmd = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
      self.err_msg = chr(0)*256

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
      buff.write(_struct_3I.pack(_x.hdr.seq, _x.hdr.stamp.secs, _x.hdr.stamp.nsecs))
      _x = self.hdr.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      if python3:
        buff.write(struct.pack('<I%sB'%length, length, *_x))
      else:
        buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_3i.pack(_x.runlevel, _x.sublevel, _x.last_seq))
      buff.write(_struct_2i.pack(*self.type))
      buff.write(_struct_6i.pack(*self.pos))
      buff.write(_struct_18f.pack(*self.ori))
      buff.write(_struct_18f.pack(*self.ori_d))
      buff.write(_struct_6i.pack(*self.pos_d))
      _x = self
      buff.write(_struct_2i.pack(_x.dt.secs, _x.dt.nsecs))
      buff.write(_struct_16i.pack(*self.encVals))
      buff.write(_struct_16f.pack(*self.tau))
      buff.write(_struct_16f.pack(*self.mpos))
      buff.write(_struct_16f.pack(*self.jpos))
      buff.write(_struct_16f.pack(*self.mvel))
      buff.write(_struct_16f.pack(*self.mvel_d))
      buff.write(_struct_16f.pack(*self.jvel))
      buff.write(_struct_16f.pack(*self.mpos_d))
      buff.write(_struct_16f.pack(*self.jpos_d))
      buff.write(_struct_2f.pack(*self.grasp_d))
      buff.write(_struct_16f.pack(*self.encoffsets))
      buff.write(_struct_16i.pack(*self.current_cmd))
      _x = self.err_msg
      # - if encoded as a list instead, serialize as bytes instead of string
      if type(_x) in [list, tuple]:
        buff.write(_struct_256B.pack(*_x))
      else:
        buff.write(_struct_256s.pack(_x))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(_x))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(_x))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      if self.hdr is None:
        self.hdr = std_msgs.msg.Header()
      if self.dt is None:
        self.dt = genpy.Duration()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.hdr.seq, _x.hdr.stamp.secs, _x.hdr.stamp.nsecs,) = _struct_3I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.hdr.frame_id = str[start:end].decode('utf-8')
      else:
        self.hdr.frame_id = str[start:end]
      _x = self
      start = end
      end += 12
      (_x.runlevel, _x.sublevel, _x.last_seq,) = _struct_3i.unpack(str[start:end])
      start = end
      end += 8
      self.type = _struct_2i.unpack(str[start:end])
      start = end
      end += 24
      self.pos = _struct_6i.unpack(str[start:end])
      start = end
      end += 72
      self.ori = _struct_18f.unpack(str[start:end])
      start = end
      end += 72
      self.ori_d = _struct_18f.unpack(str[start:end])
      start = end
      end += 24
      self.pos_d = _struct_6i.unpack(str[start:end])
      _x = self
      start = end
      end += 8
      (_x.dt.secs, _x.dt.nsecs,) = _struct_2i.unpack(str[start:end])
      start = end
      end += 64
      self.encVals = _struct_16i.unpack(str[start:end])
      start = end
      end += 64
      self.tau = _struct_16f.unpack(str[start:end])
      start = end
      end += 64
      self.mpos = _struct_16f.unpack(str[start:end])
      start = end
      end += 64
      self.jpos = _struct_16f.unpack(str[start:end])
      start = end
      end += 64
      self.mvel = _struct_16f.unpack(str[start:end])
      start = end
      end += 64
      self.mvel_d = _struct_16f.unpack(str[start:end])
      start = end
      end += 64
      self.jvel = _struct_16f.unpack(str[start:end])
      start = end
      end += 64
      self.mpos_d = _struct_16f.unpack(str[start:end])
      start = end
      end += 64
      self.jpos_d = _struct_16f.unpack(str[start:end])
      start = end
      end += 8
      self.grasp_d = _struct_2f.unpack(str[start:end])
      start = end
      end += 64
      self.encoffsets = _struct_16f.unpack(str[start:end])
      start = end
      end += 64
      self.current_cmd = _struct_16i.unpack(str[start:end])
      start = end
      end += 256
      self.err_msg = str[start:end]
      self.dt.canon()
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
      buff.write(_struct_3I.pack(_x.hdr.seq, _x.hdr.stamp.secs, _x.hdr.stamp.nsecs))
      _x = self.hdr.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      if python3:
        buff.write(struct.pack('<I%sB'%length, length, *_x))
      else:
        buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_3i.pack(_x.runlevel, _x.sublevel, _x.last_seq))
      buff.write(self.type.tostring())
      buff.write(self.pos.tostring())
      buff.write(self.ori.tostring())
      buff.write(self.ori_d.tostring())
      buff.write(self.pos_d.tostring())
      _x = self
      buff.write(_struct_2i.pack(_x.dt.secs, _x.dt.nsecs))
      buff.write(self.encVals.tostring())
      buff.write(self.tau.tostring())
      buff.write(self.mpos.tostring())
      buff.write(self.jpos.tostring())
      buff.write(self.mvel.tostring())
      buff.write(self.mvel_d.tostring())
      buff.write(self.jvel.tostring())
      buff.write(self.mpos_d.tostring())
      buff.write(self.jpos_d.tostring())
      buff.write(self.grasp_d.tostring())
      buff.write(self.encoffsets.tostring())
      buff.write(self.current_cmd.tostring())
      _x = self.err_msg
      # - if encoded as a list instead, serialize as bytes instead of string
      if type(_x) in [list, tuple]:
        buff.write(_struct_256B.pack(*_x))
      else:
        buff.write(_struct_256s.pack(_x))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(_x))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(_x))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      if self.hdr is None:
        self.hdr = std_msgs.msg.Header()
      if self.dt is None:
        self.dt = genpy.Duration()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.hdr.seq, _x.hdr.stamp.secs, _x.hdr.stamp.nsecs,) = _struct_3I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.hdr.frame_id = str[start:end].decode('utf-8')
      else:
        self.hdr.frame_id = str[start:end]
      _x = self
      start = end
      end += 12
      (_x.runlevel, _x.sublevel, _x.last_seq,) = _struct_3i.unpack(str[start:end])
      start = end
      end += 8
      self.type = numpy.frombuffer(str[start:end], dtype=numpy.int32, count=2)
      start = end
      end += 24
      self.pos = numpy.frombuffer(str[start:end], dtype=numpy.int32, count=6)
      start = end
      end += 72
      self.ori = numpy.frombuffer(str[start:end], dtype=numpy.float32, count=18)
      start = end
      end += 72
      self.ori_d = numpy.frombuffer(str[start:end], dtype=numpy.float32, count=18)
      start = end
      end += 24
      self.pos_d = numpy.frombuffer(str[start:end], dtype=numpy.int32, count=6)
      _x = self
      start = end
      end += 8
      (_x.dt.secs, _x.dt.nsecs,) = _struct_2i.unpack(str[start:end])
      start = end
      end += 64
      self.encVals = numpy.frombuffer(str[start:end], dtype=numpy.int32, count=16)
      start = end
      end += 64
      self.tau = numpy.frombuffer(str[start:end], dtype=numpy.float32, count=16)
      start = end
      end += 64
      self.mpos = numpy.frombuffer(str[start:end], dtype=numpy.float32, count=16)
      start = end
      end += 64
      self.jpos = numpy.frombuffer(str[start:end], dtype=numpy.float32, count=16)
      start = end
      end += 64
      self.mvel = numpy.frombuffer(str[start:end], dtype=numpy.float32, count=16)
      start = end
      end += 64
      self.mvel_d = numpy.frombuffer(str[start:end], dtype=numpy.float32, count=16)
      start = end
      end += 64
      self.jvel = numpy.frombuffer(str[start:end], dtype=numpy.float32, count=16)
      start = end
      end += 64
      self.mpos_d = numpy.frombuffer(str[start:end], dtype=numpy.float32, count=16)
      start = end
      end += 64
      self.jpos_d = numpy.frombuffer(str[start:end], dtype=numpy.float32, count=16)
      start = end
      end += 8
      self.grasp_d = numpy.frombuffer(str[start:end], dtype=numpy.float32, count=2)
      start = end
      end += 64
      self.encoffsets = numpy.frombuffer(str[start:end], dtype=numpy.float32, count=16)
      start = end
      end += 64
      self.current_cmd = numpy.frombuffer(str[start:end], dtype=numpy.int32, count=16)
      start = end
      end += 256
      self.err_msg = str[start:end]
      self.dt.canon()
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_18f = struct.Struct("<18f")
_struct_16i = struct.Struct("<16i")
_struct_256s = struct.Struct("<256s")
_struct_6i = struct.Struct("<6i")
_struct_16f = struct.Struct("<16f")
_struct_3i = struct.Struct("<3i")
_struct_3I = struct.Struct("<3I")
_struct_2f = struct.Struct("<2f")
_struct_256B = struct.Struct("<256B")
_struct_2i = struct.Struct("<2i")
