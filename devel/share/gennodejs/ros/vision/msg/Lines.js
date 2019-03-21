// Auto-generated. Do not edit!

// (in-package vision.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class Lines {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.lines_number = null;
      this.line_start_x = null;
      this.line_start_y = null;
      this.line_end_x = null;
      this.line_end_y = null;
      this.line_confidence = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('lines_number')) {
        this.lines_number = initObj.lines_number
      }
      else {
        this.lines_number = 0;
      }
      if (initObj.hasOwnProperty('line_start_x')) {
        this.line_start_x = initObj.line_start_x
      }
      else {
        this.line_start_x = new Array(20).fill(0);
      }
      if (initObj.hasOwnProperty('line_start_y')) {
        this.line_start_y = initObj.line_start_y
      }
      else {
        this.line_start_y = new Array(20).fill(0);
      }
      if (initObj.hasOwnProperty('line_end_x')) {
        this.line_end_x = initObj.line_end_x
      }
      else {
        this.line_end_x = new Array(20).fill(0);
      }
      if (initObj.hasOwnProperty('line_end_y')) {
        this.line_end_y = initObj.line_end_y
      }
      else {
        this.line_end_y = new Array(20).fill(0);
      }
      if (initObj.hasOwnProperty('line_confidence')) {
        this.line_confidence = initObj.line_confidence
      }
      else {
        this.line_confidence = new Array(20).fill(0);
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Lines
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [lines_number]
    bufferOffset = _serializer.uint8(obj.lines_number, buffer, bufferOffset);
    // Check that the constant length array field [line_start_x] has the right length
    if (obj.line_start_x.length !== 20) {
      throw new Error('Unable to serialize array field line_start_x - length must be 20')
    }
    // Serialize message field [line_start_x]
    bufferOffset = _arraySerializer.float64(obj.line_start_x, buffer, bufferOffset, 20);
    // Check that the constant length array field [line_start_y] has the right length
    if (obj.line_start_y.length !== 20) {
      throw new Error('Unable to serialize array field line_start_y - length must be 20')
    }
    // Serialize message field [line_start_y]
    bufferOffset = _arraySerializer.float64(obj.line_start_y, buffer, bufferOffset, 20);
    // Check that the constant length array field [line_end_x] has the right length
    if (obj.line_end_x.length !== 20) {
      throw new Error('Unable to serialize array field line_end_x - length must be 20')
    }
    // Serialize message field [line_end_x]
    bufferOffset = _arraySerializer.float64(obj.line_end_x, buffer, bufferOffset, 20);
    // Check that the constant length array field [line_end_y] has the right length
    if (obj.line_end_y.length !== 20) {
      throw new Error('Unable to serialize array field line_end_y - length must be 20')
    }
    // Serialize message field [line_end_y]
    bufferOffset = _arraySerializer.float64(obj.line_end_y, buffer, bufferOffset, 20);
    // Check that the constant length array field [line_confidence] has the right length
    if (obj.line_confidence.length !== 20) {
      throw new Error('Unable to serialize array field line_confidence - length must be 20')
    }
    // Serialize message field [line_confidence]
    bufferOffset = _arraySerializer.float32(obj.line_confidence, buffer, bufferOffset, 20);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Lines
    let len;
    let data = new Lines(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [lines_number]
    data.lines_number = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [line_start_x]
    data.line_start_x = _arrayDeserializer.float64(buffer, bufferOffset, 20)
    // Deserialize message field [line_start_y]
    data.line_start_y = _arrayDeserializer.float64(buffer, bufferOffset, 20)
    // Deserialize message field [line_end_x]
    data.line_end_x = _arrayDeserializer.float64(buffer, bufferOffset, 20)
    // Deserialize message field [line_end_y]
    data.line_end_y = _arrayDeserializer.float64(buffer, bufferOffset, 20)
    // Deserialize message field [line_confidence]
    data.line_confidence = _arrayDeserializer.float32(buffer, bufferOffset, 20)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 721;
  }

  static datatype() {
    // Returns string type for a message object
    return 'vision/Lines';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '26fb3619336498992b94003d734b3dd9';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    uint8 lines_number  
    float64[20] line_start_x                 
    float64[20] line_start_y 
    float64[20] line_end_x                 
    float64[20] line_end_y               
    float32[20] line_confidence               # confidence 0..1
    
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
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Lines(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.lines_number !== undefined) {
      resolved.lines_number = msg.lines_number;
    }
    else {
      resolved.lines_number = 0
    }

    if (msg.line_start_x !== undefined) {
      resolved.line_start_x = msg.line_start_x;
    }
    else {
      resolved.line_start_x = new Array(20).fill(0)
    }

    if (msg.line_start_y !== undefined) {
      resolved.line_start_y = msg.line_start_y;
    }
    else {
      resolved.line_start_y = new Array(20).fill(0)
    }

    if (msg.line_end_x !== undefined) {
      resolved.line_end_x = msg.line_end_x;
    }
    else {
      resolved.line_end_x = new Array(20).fill(0)
    }

    if (msg.line_end_y !== undefined) {
      resolved.line_end_y = msg.line_end_y;
    }
    else {
      resolved.line_end_y = new Array(20).fill(0)
    }

    if (msg.line_confidence !== undefined) {
      resolved.line_confidence = msg.line_confidence;
    }
    else {
      resolved.line_confidence = new Array(20).fill(0)
    }

    return resolved;
    }
};

module.exports = Lines;
