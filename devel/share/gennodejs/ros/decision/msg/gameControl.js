// Auto-generated. Do not edit!

// (in-package decision.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class gameControl {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.state = null;
      this.secsRemaining = null;
    }
    else {
      if (initObj.hasOwnProperty('state')) {
        this.state = initObj.state
      }
      else {
        this.state = 0;
      }
      if (initObj.hasOwnProperty('secsRemaining')) {
        this.secsRemaining = initObj.secsRemaining
      }
      else {
        this.secsRemaining = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type gameControl
    // Serialize message field [state]
    bufferOffset = _serializer.int32(obj.state, buffer, bufferOffset);
    // Serialize message field [secsRemaining]
    bufferOffset = _serializer.int32(obj.secsRemaining, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type gameControl
    let len;
    let data = new gameControl(null);
    // Deserialize message field [state]
    data.state = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [secsRemaining]
    data.secsRemaining = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'decision/gameControl';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '4f8fd4e28d3da76570da7ab09212fb90';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int32 state
    int32 secsRemaining
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new gameControl(null);
    if (msg.state !== undefined) {
      resolved.state = msg.state;
    }
    else {
      resolved.state = 0
    }

    if (msg.secsRemaining !== undefined) {
      resolved.secsRemaining = msg.secsRemaining;
    }
    else {
      resolved.secsRemaining = 0
    }

    return resolved;
    }
};

module.exports = gameControl;
