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

class Landmarks {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.landmark_number = null;
      this.landmark_type = null;
      this.landmark_range = null;
      this.landmark_bearing = null;
      this.landmark_confidence = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('landmark_number')) {
        this.landmark_number = initObj.landmark_number
      }
      else {
        this.landmark_number = 0;
      }
      if (initObj.hasOwnProperty('landmark_type')) {
        this.landmark_type = initObj.landmark_type
      }
      else {
        this.landmark_type = new Array(20).fill(0);
      }
      if (initObj.hasOwnProperty('landmark_range')) {
        this.landmark_range = initObj.landmark_range
      }
      else {
        this.landmark_range = new Array(20).fill(0);
      }
      if (initObj.hasOwnProperty('landmark_bearing')) {
        this.landmark_bearing = initObj.landmark_bearing
      }
      else {
        this.landmark_bearing = new Array(20).fill(0);
      }
      if (initObj.hasOwnProperty('landmark_confidence')) {
        this.landmark_confidence = initObj.landmark_confidence
      }
      else {
        this.landmark_confidence = new Array(20).fill(0);
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Landmarks
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [landmark_number]
    bufferOffset = _serializer.uint8(obj.landmark_number, buffer, bufferOffset);
    // Check that the constant length array field [landmark_type] has the right length
    if (obj.landmark_type.length !== 20) {
      throw new Error('Unable to serialize array field landmark_type - length must be 20')
    }
    // Serialize message field [landmark_type]
    bufferOffset = _arraySerializer.uint8(obj.landmark_type, buffer, bufferOffset, 20);
    // Check that the constant length array field [landmark_range] has the right length
    if (obj.landmark_range.length !== 20) {
      throw new Error('Unable to serialize array field landmark_range - length must be 20')
    }
    // Serialize message field [landmark_range]
    bufferOffset = _arraySerializer.float64(obj.landmark_range, buffer, bufferOffset, 20);
    // Check that the constant length array field [landmark_bearing] has the right length
    if (obj.landmark_bearing.length !== 20) {
      throw new Error('Unable to serialize array field landmark_bearing - length must be 20')
    }
    // Serialize message field [landmark_bearing]
    bufferOffset = _arraySerializer.float64(obj.landmark_bearing, buffer, bufferOffset, 20);
    // Check that the constant length array field [landmark_confidence] has the right length
    if (obj.landmark_confidence.length !== 20) {
      throw new Error('Unable to serialize array field landmark_confidence - length must be 20')
    }
    // Serialize message field [landmark_confidence]
    bufferOffset = _arraySerializer.float32(obj.landmark_confidence, buffer, bufferOffset, 20);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Landmarks
    let len;
    let data = new Landmarks(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [landmark_number]
    data.landmark_number = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [landmark_type]
    data.landmark_type = _arrayDeserializer.uint8(buffer, bufferOffset, 20)
    // Deserialize message field [landmark_range]
    data.landmark_range = _arrayDeserializer.float64(buffer, bufferOffset, 20)
    // Deserialize message field [landmark_bearing]
    data.landmark_bearing = _arrayDeserializer.float64(buffer, bufferOffset, 20)
    // Deserialize message field [landmark_confidence]
    data.landmark_confidence = _arrayDeserializer.float32(buffer, bufferOffset, 20)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 421;
  }

  static datatype() {
    // Returns string type for a message object
    return 'vision/Landmarks';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '6b999f1904ab908fbb4ce607a9071d78';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
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
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Landmarks(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.landmark_number !== undefined) {
      resolved.landmark_number = msg.landmark_number;
    }
    else {
      resolved.landmark_number = 0
    }

    if (msg.landmark_type !== undefined) {
      resolved.landmark_type = msg.landmark_type;
    }
    else {
      resolved.landmark_type = new Array(20).fill(0)
    }

    if (msg.landmark_range !== undefined) {
      resolved.landmark_range = msg.landmark_range;
    }
    else {
      resolved.landmark_range = new Array(20).fill(0)
    }

    if (msg.landmark_bearing !== undefined) {
      resolved.landmark_bearing = msg.landmark_bearing;
    }
    else {
      resolved.landmark_bearing = new Array(20).fill(0)
    }

    if (msg.landmark_confidence !== undefined) {
      resolved.landmark_confidence = msg.landmark_confidence;
    }
    else {
      resolved.landmark_confidence = new Array(20).fill(0)
    }

    return resolved;
    }
};

module.exports = Landmarks;
