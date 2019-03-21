// Auto-generated. Do not edit!

// (in-package decision.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class walkRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.type = null;
      this.speed_x = null;
      this.speed_y = null;
      this.rotation_speed = null;
      this.step = null;
    }
    else {
      if (initObj.hasOwnProperty('type')) {
        this.type = initObj.type
      }
      else {
        this.type = 0;
      }
      if (initObj.hasOwnProperty('speed_x')) {
        this.speed_x = initObj.speed_x
      }
      else {
        this.speed_x = 0.0;
      }
      if (initObj.hasOwnProperty('speed_y')) {
        this.speed_y = initObj.speed_y
      }
      else {
        this.speed_y = 0.0;
      }
      if (initObj.hasOwnProperty('rotation_speed')) {
        this.rotation_speed = initObj.rotation_speed
      }
      else {
        this.rotation_speed = 0.0;
      }
      if (initObj.hasOwnProperty('step')) {
        this.step = initObj.step
      }
      else {
        this.step = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type walkRequest
    // Serialize message field [type]
    bufferOffset = _serializer.int32(obj.type, buffer, bufferOffset);
    // Serialize message field [speed_x]
    bufferOffset = _serializer.float64(obj.speed_x, buffer, bufferOffset);
    // Serialize message field [speed_y]
    bufferOffset = _serializer.float64(obj.speed_y, buffer, bufferOffset);
    // Serialize message field [rotation_speed]
    bufferOffset = _serializer.float64(obj.rotation_speed, buffer, bufferOffset);
    // Serialize message field [step]
    bufferOffset = _serializer.int32(obj.step, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type walkRequest
    let len;
    let data = new walkRequest(null);
    // Deserialize message field [type]
    data.type = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [speed_x]
    data.speed_x = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [speed_y]
    data.speed_y = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [rotation_speed]
    data.rotation_speed = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [step]
    data.step = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 32;
  }

  static datatype() {
    // Returns string type for a service object
    return 'decision/walkRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '69b51b917edc240405fe819ca92a881d';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int32 type
    float64 speed_x
    float64 speed_y
    float64 rotation_speed
    int32 step
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new walkRequest(null);
    if (msg.type !== undefined) {
      resolved.type = msg.type;
    }
    else {
      resolved.type = 0
    }

    if (msg.speed_x !== undefined) {
      resolved.speed_x = msg.speed_x;
    }
    else {
      resolved.speed_x = 0.0
    }

    if (msg.speed_y !== undefined) {
      resolved.speed_y = msg.speed_y;
    }
    else {
      resolved.speed_y = 0.0
    }

    if (msg.rotation_speed !== undefined) {
      resolved.rotation_speed = msg.rotation_speed;
    }
    else {
      resolved.rotation_speed = 0.0
    }

    if (msg.step !== undefined) {
      resolved.step = msg.step;
    }
    else {
      resolved.step = 0
    }

    return resolved;
    }
};

class walkResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.results = null;
    }
    else {
      if (initObj.hasOwnProperty('results')) {
        this.results = initObj.results
      }
      else {
        this.results = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type walkResponse
    // Serialize message field [results]
    bufferOffset = _serializer.int32(obj.results, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type walkResponse
    let len;
    let data = new walkResponse(null);
    // Deserialize message field [results]
    data.results = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'decision/walkResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '197ce17ed92dd2e138792386b1f2fbad';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int32 results
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new walkResponse(null);
    if (msg.results !== undefined) {
      resolved.results = msg.results;
    }
    else {
      resolved.results = 0
    }

    return resolved;
    }
};

module.exports = {
  Request: walkRequest,
  Response: walkResponse,
  md5sum() { return 'abed49acefdf43a65d0e6ffd2bf04f9b'; },
  datatype() { return 'decision/walk'; }
};
