// Auto-generated. Do not edit!

// (in-package vision.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class ObjectOnImage {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.object_type = null;
      this.u = null;
      this.v = null;
    }
    else {
      if (initObj.hasOwnProperty('object_type')) {
        this.object_type = initObj.object_type
      }
      else {
        this.object_type = 0;
      }
      if (initObj.hasOwnProperty('u')) {
        this.u = initObj.u
      }
      else {
        this.u = [];
      }
      if (initObj.hasOwnProperty('v')) {
        this.v = initObj.v
      }
      else {
        this.v = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ObjectOnImage
    // Serialize message field [object_type]
    bufferOffset = _serializer.uint8(obj.object_type, buffer, bufferOffset);
    // Serialize message field [u]
    bufferOffset = _arraySerializer.uint8(obj.u, buffer, bufferOffset, null);
    // Serialize message field [v]
    bufferOffset = _arraySerializer.uint8(obj.v, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ObjectOnImage
    let len;
    let data = new ObjectOnImage(null);
    // Deserialize message field [object_type]
    data.object_type = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [u]
    data.u = _arrayDeserializer.uint8(buffer, bufferOffset, null)
    // Deserialize message field [v]
    data.v = _arrayDeserializer.uint8(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.u.length;
    length += object.v.length;
    return length + 9;
  }

  static datatype() {
    // Returns string type for a message object
    return 'vision/ObjectOnImage';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd8b2eee0990ad6385c56c5984bb6dea1';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    uint8 object_type
    uint8[] u
    uint8[] v
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ObjectOnImage(null);
    if (msg.object_type !== undefined) {
      resolved.object_type = msg.object_type;
    }
    else {
      resolved.object_type = 0
    }

    if (msg.u !== undefined) {
      resolved.u = msg.u;
    }
    else {
      resolved.u = []
    }

    if (msg.v !== undefined) {
      resolved.v = msg.v;
    }
    else {
      resolved.v = []
    }

    return resolved;
    }
};

module.exports = ObjectOnImage;
