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

class Goalpost {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.goalpost_detected = null;
      this.goalpost_number = null;
      this.goalpost_left_range = null;
      this.goalpost_left_bearing = null;
      this.goalpost_right_range = null;
      this.goalpost_right_bearing = null;
      this.goalpost_center_range = null;
      this.goalpost_center_bearing = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('goalpost_detected')) {
        this.goalpost_detected = initObj.goalpost_detected
      }
      else {
        this.goalpost_detected = false;
      }
      if (initObj.hasOwnProperty('goalpost_number')) {
        this.goalpost_number = initObj.goalpost_number
      }
      else {
        this.goalpost_number = 0;
      }
      if (initObj.hasOwnProperty('goalpost_left_range')) {
        this.goalpost_left_range = initObj.goalpost_left_range
      }
      else {
        this.goalpost_left_range = 0.0;
      }
      if (initObj.hasOwnProperty('goalpost_left_bearing')) {
        this.goalpost_left_bearing = initObj.goalpost_left_bearing
      }
      else {
        this.goalpost_left_bearing = 0.0;
      }
      if (initObj.hasOwnProperty('goalpost_right_range')) {
        this.goalpost_right_range = initObj.goalpost_right_range
      }
      else {
        this.goalpost_right_range = 0.0;
      }
      if (initObj.hasOwnProperty('goalpost_right_bearing')) {
        this.goalpost_right_bearing = initObj.goalpost_right_bearing
      }
      else {
        this.goalpost_right_bearing = 0.0;
      }
      if (initObj.hasOwnProperty('goalpost_center_range')) {
        this.goalpost_center_range = initObj.goalpost_center_range
      }
      else {
        this.goalpost_center_range = 0.0;
      }
      if (initObj.hasOwnProperty('goalpost_center_bearing')) {
        this.goalpost_center_bearing = initObj.goalpost_center_bearing
      }
      else {
        this.goalpost_center_bearing = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Goalpost
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [goalpost_detected]
    bufferOffset = _serializer.bool(obj.goalpost_detected, buffer, bufferOffset);
    // Serialize message field [goalpost_number]
    bufferOffset = _serializer.int8(obj.goalpost_number, buffer, bufferOffset);
    // Serialize message field [goalpost_left_range]
    bufferOffset = _serializer.float64(obj.goalpost_left_range, buffer, bufferOffset);
    // Serialize message field [goalpost_left_bearing]
    bufferOffset = _serializer.float64(obj.goalpost_left_bearing, buffer, bufferOffset);
    // Serialize message field [goalpost_right_range]
    bufferOffset = _serializer.float64(obj.goalpost_right_range, buffer, bufferOffset);
    // Serialize message field [goalpost_right_bearing]
    bufferOffset = _serializer.float64(obj.goalpost_right_bearing, buffer, bufferOffset);
    // Serialize message field [goalpost_center_range]
    bufferOffset = _serializer.float64(obj.goalpost_center_range, buffer, bufferOffset);
    // Serialize message field [goalpost_center_bearing]
    bufferOffset = _serializer.float64(obj.goalpost_center_bearing, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Goalpost
    let len;
    let data = new Goalpost(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [goalpost_detected]
    data.goalpost_detected = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [goalpost_number]
    data.goalpost_number = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [goalpost_left_range]
    data.goalpost_left_range = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [goalpost_left_bearing]
    data.goalpost_left_bearing = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [goalpost_right_range]
    data.goalpost_right_range = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [goalpost_right_bearing]
    data.goalpost_right_bearing = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [goalpost_center_range]
    data.goalpost_center_range = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [goalpost_center_bearing]
    data.goalpost_center_bearing = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 50;
  }

  static datatype() {
    // Returns string type for a message object
    return 'vision/Goalpost';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'c7e85911d22877f743fec7cea6c072ce';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
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
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Goalpost(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.goalpost_detected !== undefined) {
      resolved.goalpost_detected = msg.goalpost_detected;
    }
    else {
      resolved.goalpost_detected = false
    }

    if (msg.goalpost_number !== undefined) {
      resolved.goalpost_number = msg.goalpost_number;
    }
    else {
      resolved.goalpost_number = 0
    }

    if (msg.goalpost_left_range !== undefined) {
      resolved.goalpost_left_range = msg.goalpost_left_range;
    }
    else {
      resolved.goalpost_left_range = 0.0
    }

    if (msg.goalpost_left_bearing !== undefined) {
      resolved.goalpost_left_bearing = msg.goalpost_left_bearing;
    }
    else {
      resolved.goalpost_left_bearing = 0.0
    }

    if (msg.goalpost_right_range !== undefined) {
      resolved.goalpost_right_range = msg.goalpost_right_range;
    }
    else {
      resolved.goalpost_right_range = 0.0
    }

    if (msg.goalpost_right_bearing !== undefined) {
      resolved.goalpost_right_bearing = msg.goalpost_right_bearing;
    }
    else {
      resolved.goalpost_right_bearing = 0.0
    }

    if (msg.goalpost_center_range !== undefined) {
      resolved.goalpost_center_range = msg.goalpost_center_range;
    }
    else {
      resolved.goalpost_center_range = 0.0
    }

    if (msg.goalpost_center_bearing !== undefined) {
      resolved.goalpost_center_bearing = msg.goalpost_center_bearing;
    }
    else {
      resolved.goalpost_center_bearing = 0.0
    }

    return resolved;
    }
};

module.exports = Goalpost;
