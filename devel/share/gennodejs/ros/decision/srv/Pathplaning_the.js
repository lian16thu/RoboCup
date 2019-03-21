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

class Pathplaning_theRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.state = null;
    }
    else {
      if (initObj.hasOwnProperty('state')) {
        this.state = initObj.state
      }
      else {
        this.state = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Pathplaning_theRequest
    // Serialize message field [state]
    bufferOffset = _serializer.bool(obj.state, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Pathplaning_theRequest
    let len;
    let data = new Pathplaning_theRequest(null);
    // Deserialize message field [state]
    data.state = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'decision/Pathplaning_theRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '001fde3cab9e313a150416ff09c08ee4';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool state
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Pathplaning_theRequest(null);
    if (msg.state !== undefined) {
      resolved.state = msg.state;
    }
    else {
      resolved.state = false
    }

    return resolved;
    }
};

class Pathplaning_theResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.theta_result = null;
    }
    else {
      if (initObj.hasOwnProperty('theta_result')) {
        this.theta_result = initObj.theta_result
      }
      else {
        this.theta_result = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Pathplaning_theResponse
    // Serialize message field [theta_result]
    bufferOffset = _serializer.float32(obj.theta_result, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Pathplaning_theResponse
    let len;
    let data = new Pathplaning_theResponse(null);
    // Deserialize message field [theta_result]
    data.theta_result = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'decision/Pathplaning_theResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '38f246c33d90c8333d784465d938cba6';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float32 theta_result
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Pathplaning_theResponse(null);
    if (msg.theta_result !== undefined) {
      resolved.theta_result = msg.theta_result;
    }
    else {
      resolved.theta_result = 0.0
    }

    return resolved;
    }
};

module.exports = {
  Request: Pathplaning_theRequest,
  Response: Pathplaning_theResponse,
  md5sum() { return '743721b9c6ac4f45f0566da711cade92'; },
  datatype() { return 'decision/Pathplaning_the'; }
};
