// Auto-generated. Do not edit!

// (in-package vision.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class DepthRequestRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.u = null;
      this.v = null;
    }
    else {
      if (initObj.hasOwnProperty('u')) {
        this.u = initObj.u
      }
      else {
        this.u = 0;
      }
      if (initObj.hasOwnProperty('v')) {
        this.v = initObj.v
      }
      else {
        this.v = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type DepthRequestRequest
    // Serialize message field [u]
    bufferOffset = _serializer.int32(obj.u, buffer, bufferOffset);
    // Serialize message field [v]
    bufferOffset = _serializer.int32(obj.v, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type DepthRequestRequest
    let len;
    let data = new DepthRequestRequest(null);
    // Deserialize message field [u]
    data.u = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [v]
    data.v = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 8;
  }

  static datatype() {
    // Returns string type for a service object
    return 'vision/DepthRequestRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'f06a7e8dd345ffb826d71a489782114b';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    
    int32 u
    int32 v
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new DepthRequestRequest(null);
    if (msg.u !== undefined) {
      resolved.u = msg.u;
    }
    else {
      resolved.u = 0
    }

    if (msg.v !== undefined) {
      resolved.v = msg.v;
    }
    else {
      resolved.v = 0
    }

    return resolved;
    }
};

class DepthRequestResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.depth = null;
    }
    else {
      if (initObj.hasOwnProperty('depth')) {
        this.depth = initObj.depth
      }
      else {
        this.depth = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type DepthRequestResponse
    // Serialize message field [depth]
    bufferOffset = _serializer.float64(obj.depth, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type DepthRequestResponse
    let len;
    let data = new DepthRequestResponse(null);
    // Deserialize message field [depth]
    data.depth = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 8;
  }

  static datatype() {
    // Returns string type for a service object
    return 'vision/DepthRequestResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '3493bcd175ed22d87965370b75a0d961';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64 depth
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new DepthRequestResponse(null);
    if (msg.depth !== undefined) {
      resolved.depth = msg.depth;
    }
    else {
      resolved.depth = 0.0
    }

    return resolved;
    }
};

module.exports = {
  Request: DepthRequestRequest,
  Response: DepthRequestResponse,
  md5sum() { return '1036fa6d5d2b46f7aa7aaecd35620560'; },
  datatype() { return 'vision/DepthRequest'; }
};
