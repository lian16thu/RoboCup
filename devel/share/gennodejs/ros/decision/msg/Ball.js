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

class Ball {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.bBallWasSeen = null;
      this.iCenterInImageX = null;
      this.iCenterInImageY = null;
      this.iRadiusInImage = null;
      this.fDistance = null;
      this.fAngle = null;
    }
    else {
      if (initObj.hasOwnProperty('bBallWasSeen')) {
        this.bBallWasSeen = initObj.bBallWasSeen
      }
      else {
        this.bBallWasSeen = 0;
      }
      if (initObj.hasOwnProperty('iCenterInImageX')) {
        this.iCenterInImageX = initObj.iCenterInImageX
      }
      else {
        this.iCenterInImageX = 0;
      }
      if (initObj.hasOwnProperty('iCenterInImageY')) {
        this.iCenterInImageY = initObj.iCenterInImageY
      }
      else {
        this.iCenterInImageY = 0;
      }
      if (initObj.hasOwnProperty('iRadiusInImage')) {
        this.iRadiusInImage = initObj.iRadiusInImage
      }
      else {
        this.iRadiusInImage = 0;
      }
      if (initObj.hasOwnProperty('fDistance')) {
        this.fDistance = initObj.fDistance
      }
      else {
        this.fDistance = 0.0;
      }
      if (initObj.hasOwnProperty('fAngle')) {
        this.fAngle = initObj.fAngle
      }
      else {
        this.fAngle = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Ball
    // Serialize message field [bBallWasSeen]
    bufferOffset = _serializer.uint8(obj.bBallWasSeen, buffer, bufferOffset);
    // Serialize message field [iCenterInImageX]
    bufferOffset = _serializer.uint16(obj.iCenterInImageX, buffer, bufferOffset);
    // Serialize message field [iCenterInImageY]
    bufferOffset = _serializer.uint16(obj.iCenterInImageY, buffer, bufferOffset);
    // Serialize message field [iRadiusInImage]
    bufferOffset = _serializer.uint16(obj.iRadiusInImage, buffer, bufferOffset);
    // Serialize message field [fDistance]
    bufferOffset = _serializer.float32(obj.fDistance, buffer, bufferOffset);
    // Serialize message field [fAngle]
    bufferOffset = _serializer.float32(obj.fAngle, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Ball
    let len;
    let data = new Ball(null);
    // Deserialize message field [bBallWasSeen]
    data.bBallWasSeen = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [iCenterInImageX]
    data.iCenterInImageX = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [iCenterInImageY]
    data.iCenterInImageY = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [iRadiusInImage]
    data.iRadiusInImage = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [fDistance]
    data.fDistance = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [fAngle]
    data.fAngle = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 15;
  }

  static datatype() {
    // Returns string type for a message object
    return 'decision/Ball';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'b1776e77d61a18b8d498a831f6c9807a';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    uint8 bBallWasSeen
    uint16 iCenterInImageX
    uint16 iCenterInImageY
    uint16 iRadiusInImage
    float32 fDistance
    float32 fAngle
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Ball(null);
    if (msg.bBallWasSeen !== undefined) {
      resolved.bBallWasSeen = msg.bBallWasSeen;
    }
    else {
      resolved.bBallWasSeen = 0
    }

    if (msg.iCenterInImageX !== undefined) {
      resolved.iCenterInImageX = msg.iCenterInImageX;
    }
    else {
      resolved.iCenterInImageX = 0
    }

    if (msg.iCenterInImageY !== undefined) {
      resolved.iCenterInImageY = msg.iCenterInImageY;
    }
    else {
      resolved.iCenterInImageY = 0
    }

    if (msg.iRadiusInImage !== undefined) {
      resolved.iRadiusInImage = msg.iRadiusInImage;
    }
    else {
      resolved.iRadiusInImage = 0
    }

    if (msg.fDistance !== undefined) {
      resolved.fDistance = msg.fDistance;
    }
    else {
      resolved.fDistance = 0.0
    }

    if (msg.fAngle !== undefined) {
      resolved.fAngle = msg.fAngle;
    }
    else {
      resolved.fAngle = 0.0
    }

    return resolved;
    }
};

module.exports = Ball;
