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

class GoalData {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.goal = null;
      this.leftx = null;
      this.lefty = null;
      this.rightx = null;
      this.righty = null;
    }
    else {
      if (initObj.hasOwnProperty('goal')) {
        this.goal = initObj.goal
      }
      else {
        this.goal = 0;
      }
      if (initObj.hasOwnProperty('leftx')) {
        this.leftx = initObj.leftx
      }
      else {
        this.leftx = 0.0;
      }
      if (initObj.hasOwnProperty('lefty')) {
        this.lefty = initObj.lefty
      }
      else {
        this.lefty = 0.0;
      }
      if (initObj.hasOwnProperty('rightx')) {
        this.rightx = initObj.rightx
      }
      else {
        this.rightx = 0.0;
      }
      if (initObj.hasOwnProperty('righty')) {
        this.righty = initObj.righty
      }
      else {
        this.righty = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GoalData
    // Serialize message field [goal]
    bufferOffset = _serializer.uint8(obj.goal, buffer, bufferOffset);
    // Serialize message field [leftx]
    bufferOffset = _serializer.float32(obj.leftx, buffer, bufferOffset);
    // Serialize message field [lefty]
    bufferOffset = _serializer.float32(obj.lefty, buffer, bufferOffset);
    // Serialize message field [rightx]
    bufferOffset = _serializer.float32(obj.rightx, buffer, bufferOffset);
    // Serialize message field [righty]
    bufferOffset = _serializer.float32(obj.righty, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GoalData
    let len;
    let data = new GoalData(null);
    // Deserialize message field [goal]
    data.goal = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [leftx]
    data.leftx = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [lefty]
    data.lefty = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [rightx]
    data.rightx = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [righty]
    data.righty = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 17;
  }

  static datatype() {
    // Returns string type for a message object
    return 'decision/GoalData';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '657b4b89e12eaea48f7336974eb25b11';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    uint8 goal
    float32 leftx
    float32 lefty
    float32 rightx
    float32 righty
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GoalData(null);
    if (msg.goal !== undefined) {
      resolved.goal = msg.goal;
    }
    else {
      resolved.goal = 0
    }

    if (msg.leftx !== undefined) {
      resolved.leftx = msg.leftx;
    }
    else {
      resolved.leftx = 0.0
    }

    if (msg.lefty !== undefined) {
      resolved.lefty = msg.lefty;
    }
    else {
      resolved.lefty = 0.0
    }

    if (msg.rightx !== undefined) {
      resolved.rightx = msg.rightx;
    }
    else {
      resolved.rightx = 0.0
    }

    if (msg.righty !== undefined) {
      resolved.righty = msg.righty;
    }
    else {
      resolved.righty = 0.0
    }

    return resolved;
    }
};

module.exports = GoalData;
