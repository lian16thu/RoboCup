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

class Markers {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.markers_detected = null;
      this.markers_number = null;
      this.marker_id = null;
      this.marker_rotvec_x = null;
      this.marker_rotvec_y = null;
      this.marker_rotvec_z = null;
      this.marker_transvec_x = null;
      this.marker_transvec_y = null;
      this.marker_transvec_z = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('markers_detected')) {
        this.markers_detected = initObj.markers_detected
      }
      else {
        this.markers_detected = false;
      }
      if (initObj.hasOwnProperty('markers_number')) {
        this.markers_number = initObj.markers_number
      }
      else {
        this.markers_number = 0;
      }
      if (initObj.hasOwnProperty('marker_id')) {
        this.marker_id = initObj.marker_id
      }
      else {
        this.marker_id = [];
      }
      if (initObj.hasOwnProperty('marker_rotvec_x')) {
        this.marker_rotvec_x = initObj.marker_rotvec_x
      }
      else {
        this.marker_rotvec_x = [];
      }
      if (initObj.hasOwnProperty('marker_rotvec_y')) {
        this.marker_rotvec_y = initObj.marker_rotvec_y
      }
      else {
        this.marker_rotvec_y = [];
      }
      if (initObj.hasOwnProperty('marker_rotvec_z')) {
        this.marker_rotvec_z = initObj.marker_rotvec_z
      }
      else {
        this.marker_rotvec_z = [];
      }
      if (initObj.hasOwnProperty('marker_transvec_x')) {
        this.marker_transvec_x = initObj.marker_transvec_x
      }
      else {
        this.marker_transvec_x = [];
      }
      if (initObj.hasOwnProperty('marker_transvec_y')) {
        this.marker_transvec_y = initObj.marker_transvec_y
      }
      else {
        this.marker_transvec_y = [];
      }
      if (initObj.hasOwnProperty('marker_transvec_z')) {
        this.marker_transvec_z = initObj.marker_transvec_z
      }
      else {
        this.marker_transvec_z = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Markers
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [markers_detected]
    bufferOffset = _serializer.bool(obj.markers_detected, buffer, bufferOffset);
    // Serialize message field [markers_number]
    bufferOffset = _serializer.uint16(obj.markers_number, buffer, bufferOffset);
    // Serialize message field [marker_id]
    bufferOffset = _arraySerializer.int8(obj.marker_id, buffer, bufferOffset, null);
    // Serialize message field [marker_rotvec_x]
    bufferOffset = _arraySerializer.float64(obj.marker_rotvec_x, buffer, bufferOffset, null);
    // Serialize message field [marker_rotvec_y]
    bufferOffset = _arraySerializer.float64(obj.marker_rotvec_y, buffer, bufferOffset, null);
    // Serialize message field [marker_rotvec_z]
    bufferOffset = _arraySerializer.float64(obj.marker_rotvec_z, buffer, bufferOffset, null);
    // Serialize message field [marker_transvec_x]
    bufferOffset = _arraySerializer.float64(obj.marker_transvec_x, buffer, bufferOffset, null);
    // Serialize message field [marker_transvec_y]
    bufferOffset = _arraySerializer.float64(obj.marker_transvec_y, buffer, bufferOffset, null);
    // Serialize message field [marker_transvec_z]
    bufferOffset = _arraySerializer.float64(obj.marker_transvec_z, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Markers
    let len;
    let data = new Markers(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [markers_detected]
    data.markers_detected = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [markers_number]
    data.markers_number = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [marker_id]
    data.marker_id = _arrayDeserializer.int8(buffer, bufferOffset, null)
    // Deserialize message field [marker_rotvec_x]
    data.marker_rotvec_x = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [marker_rotvec_y]
    data.marker_rotvec_y = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [marker_rotvec_z]
    data.marker_rotvec_z = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [marker_transvec_x]
    data.marker_transvec_x = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [marker_transvec_y]
    data.marker_transvec_y = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [marker_transvec_z]
    data.marker_transvec_z = _arrayDeserializer.float64(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += object.marker_id.length;
    length += 8 * object.marker_rotvec_x.length;
    length += 8 * object.marker_rotvec_y.length;
    length += 8 * object.marker_rotvec_z.length;
    length += 8 * object.marker_transvec_x.length;
    length += 8 * object.marker_transvec_y.length;
    length += 8 * object.marker_transvec_z.length;
    return length + 31;
  }

  static datatype() {
    // Returns string type for a message object
    return 'vision/Markers';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '1a85c45b00f3e0388fd3fe5a952433ae';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    bool markers_detected
    uint16 markers_number  #all together, if more than two seen, placed from closer two to further
    int8[] marker_id
    float64[] marker_rotvec_x
    float64[] marker_rotvec_y
    float64[] marker_rotvec_z
    float64[] marker_transvec_x
    float64[] marker_transvec_y
    float64[] marker_transvec_z
    
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
    const resolved = new Markers(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.markers_detected !== undefined) {
      resolved.markers_detected = msg.markers_detected;
    }
    else {
      resolved.markers_detected = false
    }

    if (msg.markers_number !== undefined) {
      resolved.markers_number = msg.markers_number;
    }
    else {
      resolved.markers_number = 0
    }

    if (msg.marker_id !== undefined) {
      resolved.marker_id = msg.marker_id;
    }
    else {
      resolved.marker_id = []
    }

    if (msg.marker_rotvec_x !== undefined) {
      resolved.marker_rotvec_x = msg.marker_rotvec_x;
    }
    else {
      resolved.marker_rotvec_x = []
    }

    if (msg.marker_rotvec_y !== undefined) {
      resolved.marker_rotvec_y = msg.marker_rotvec_y;
    }
    else {
      resolved.marker_rotvec_y = []
    }

    if (msg.marker_rotvec_z !== undefined) {
      resolved.marker_rotvec_z = msg.marker_rotvec_z;
    }
    else {
      resolved.marker_rotvec_z = []
    }

    if (msg.marker_transvec_x !== undefined) {
      resolved.marker_transvec_x = msg.marker_transvec_x;
    }
    else {
      resolved.marker_transvec_x = []
    }

    if (msg.marker_transvec_y !== undefined) {
      resolved.marker_transvec_y = msg.marker_transvec_y;
    }
    else {
      resolved.marker_transvec_y = []
    }

    if (msg.marker_transvec_z !== undefined) {
      resolved.marker_transvec_z = msg.marker_transvec_z;
    }
    else {
      resolved.marker_transvec_z = []
    }

    return resolved;
    }
};

module.exports = Markers;
