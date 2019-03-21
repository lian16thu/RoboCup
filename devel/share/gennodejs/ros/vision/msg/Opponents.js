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

class Opponents {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.opponent_detected = null;
      this.opponent_number = null;
      this.opponent_range = null;
      this.opponent_bearing = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('opponent_detected')) {
        this.opponent_detected = initObj.opponent_detected
      }
      else {
        this.opponent_detected = false;
      }
      if (initObj.hasOwnProperty('opponent_number')) {
        this.opponent_number = initObj.opponent_number
      }
      else {
        this.opponent_number = 0;
      }
      if (initObj.hasOwnProperty('opponent_range')) {
        this.opponent_range = initObj.opponent_range
      }
      else {
        this.opponent_range = new Array(5).fill(0);
      }
      if (initObj.hasOwnProperty('opponent_bearing')) {
        this.opponent_bearing = initObj.opponent_bearing
      }
      else {
        this.opponent_bearing = new Array(5).fill(0);
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Opponents
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [opponent_detected]
    bufferOffset = _serializer.bool(obj.opponent_detected, buffer, bufferOffset);
    // Serialize message field [opponent_number]
    bufferOffset = _serializer.uint8(obj.opponent_number, buffer, bufferOffset);
    // Check that the constant length array field [opponent_range] has the right length
    if (obj.opponent_range.length !== 5) {
      throw new Error('Unable to serialize array field opponent_range - length must be 5')
    }
    // Serialize message field [opponent_range]
    bufferOffset = _arraySerializer.float64(obj.opponent_range, buffer, bufferOffset, 5);
    // Check that the constant length array field [opponent_bearing] has the right length
    if (obj.opponent_bearing.length !== 5) {
      throw new Error('Unable to serialize array field opponent_bearing - length must be 5')
    }
    // Serialize message field [opponent_bearing]
    bufferOffset = _arraySerializer.float64(obj.opponent_bearing, buffer, bufferOffset, 5);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Opponents
    let len;
    let data = new Opponents(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [opponent_detected]
    data.opponent_detected = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [opponent_number]
    data.opponent_number = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [opponent_range]
    data.opponent_range = _arrayDeserializer.float64(buffer, bufferOffset, 5)
    // Deserialize message field [opponent_bearing]
    data.opponent_bearing = _arrayDeserializer.float64(buffer, bufferOffset, 5)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 82;
  }

  static datatype() {
    // Returns string type for a message object
    return 'vision/Opponents';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'ef41a5e204617f0c2b713c70ec7cf155';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    bool opponent_detected
    uint8 opponent_number
    float64[5] opponent_range
    float64[5] opponent_bearing
    
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
    const resolved = new Opponents(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.opponent_detected !== undefined) {
      resolved.opponent_detected = msg.opponent_detected;
    }
    else {
      resolved.opponent_detected = false
    }

    if (msg.opponent_number !== undefined) {
      resolved.opponent_number = msg.opponent_number;
    }
    else {
      resolved.opponent_number = 0
    }

    if (msg.opponent_range !== undefined) {
      resolved.opponent_range = msg.opponent_range;
    }
    else {
      resolved.opponent_range = new Array(5).fill(0)
    }

    if (msg.opponent_bearing !== undefined) {
      resolved.opponent_bearing = msg.opponent_bearing;
    }
    else {
      resolved.opponent_bearing = new Array(5).fill(0)
    }

    return resolved;
    }
};

module.exports = Opponents;
