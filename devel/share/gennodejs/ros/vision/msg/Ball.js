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

class Ball {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.ball_detected = null;
      this.ball_range = null;
      this.ball_bearing = null;
      this.ball_radius = null;
      this.ball_center_x = null;
      this.ball_center_y = null;
      this.kick_time = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('ball_detected')) {
        this.ball_detected = initObj.ball_detected
      }
      else {
        this.ball_detected = false;
      }
      if (initObj.hasOwnProperty('ball_range')) {
        this.ball_range = initObj.ball_range
      }
      else {
        this.ball_range = 0.0;
      }
      if (initObj.hasOwnProperty('ball_bearing')) {
        this.ball_bearing = initObj.ball_bearing
      }
      else {
        this.ball_bearing = 0.0;
      }
      if (initObj.hasOwnProperty('ball_radius')) {
        this.ball_radius = initObj.ball_radius
      }
      else {
        this.ball_radius = 0.0;
      }
      if (initObj.hasOwnProperty('ball_center_x')) {
        this.ball_center_x = initObj.ball_center_x
      }
      else {
        this.ball_center_x = 0;
      }
      if (initObj.hasOwnProperty('ball_center_y')) {
        this.ball_center_y = initObj.ball_center_y
      }
      else {
        this.ball_center_y = 0;
      }
      if (initObj.hasOwnProperty('kick_time')) {
        this.kick_time = initObj.kick_time
      }
      else {
        this.kick_time = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Ball
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [ball_detected]
    bufferOffset = _serializer.bool(obj.ball_detected, buffer, bufferOffset);
    // Serialize message field [ball_range]
    bufferOffset = _serializer.float64(obj.ball_range, buffer, bufferOffset);
    // Serialize message field [ball_bearing]
    bufferOffset = _serializer.float64(obj.ball_bearing, buffer, bufferOffset);
    // Serialize message field [ball_radius]
    bufferOffset = _serializer.float64(obj.ball_radius, buffer, bufferOffset);
    // Serialize message field [ball_center_x]
    bufferOffset = _serializer.int64(obj.ball_center_x, buffer, bufferOffset);
    // Serialize message field [ball_center_y]
    bufferOffset = _serializer.int64(obj.ball_center_y, buffer, bufferOffset);
    // Serialize message field [kick_time]
    bufferOffset = _serializer.float64(obj.kick_time, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Ball
    let len;
    let data = new Ball(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [ball_detected]
    data.ball_detected = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [ball_range]
    data.ball_range = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [ball_bearing]
    data.ball_bearing = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [ball_radius]
    data.ball_radius = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [ball_center_x]
    data.ball_center_x = _deserializer.int64(buffer, bufferOffset);
    // Deserialize message field [ball_center_y]
    data.ball_center_y = _deserializer.int64(buffer, bufferOffset);
    // Deserialize message field [kick_time]
    data.kick_time = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 49;
  }

  static datatype() {
    // Returns string type for a message object
    return 'vision/Ball';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '07dd74c62bc00b3a12c5e1cd0e3a4159';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    bool ball_detected
    float64 ball_range
    float64 ball_bearing
    float64 ball_radius
    int64 ball_center_x
    int64 ball_center_y
    float64 kick_time
    
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
    const resolved = new Ball(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.ball_detected !== undefined) {
      resolved.ball_detected = msg.ball_detected;
    }
    else {
      resolved.ball_detected = false
    }

    if (msg.ball_range !== undefined) {
      resolved.ball_range = msg.ball_range;
    }
    else {
      resolved.ball_range = 0.0
    }

    if (msg.ball_bearing !== undefined) {
      resolved.ball_bearing = msg.ball_bearing;
    }
    else {
      resolved.ball_bearing = 0.0
    }

    if (msg.ball_radius !== undefined) {
      resolved.ball_radius = msg.ball_radius;
    }
    else {
      resolved.ball_radius = 0.0
    }

    if (msg.ball_center_x !== undefined) {
      resolved.ball_center_x = msg.ball_center_x;
    }
    else {
      resolved.ball_center_x = 0
    }

    if (msg.ball_center_y !== undefined) {
      resolved.ball_center_y = msg.ball_center_y;
    }
    else {
      resolved.ball_center_y = 0
    }

    if (msg.kick_time !== undefined) {
      resolved.kick_time = msg.kick_time;
    }
    else {
      resolved.kick_time = 0.0
    }

    return resolved;
    }
};

module.exports = Ball;
