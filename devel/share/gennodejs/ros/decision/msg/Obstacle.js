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

class Obstacle {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.bObstacleWasSeen = null;
      this.iObstacleNumber = null;
      this.iLeftEdgeInImageX = null;
      this.iLeftEdgeInImageY = null;
      this.iRightEdgeInImageX = null;
      this.iRightEdgeInImageY = null;
      this.iHeightInImage = null;
      this.fDistance = null;
      this.fAngle = null;
    }
    else {
      if (initObj.hasOwnProperty('bObstacleWasSeen')) {
        this.bObstacleWasSeen = initObj.bObstacleWasSeen
      }
      else {
        this.bObstacleWasSeen = 0;
      }
      if (initObj.hasOwnProperty('iObstacleNumber')) {
        this.iObstacleNumber = initObj.iObstacleNumber
      }
      else {
        this.iObstacleNumber = 0;
      }
      if (initObj.hasOwnProperty('iLeftEdgeInImageX')) {
        this.iLeftEdgeInImageX = initObj.iLeftEdgeInImageX
      }
      else {
        this.iLeftEdgeInImageX = [];
      }
      if (initObj.hasOwnProperty('iLeftEdgeInImageY')) {
        this.iLeftEdgeInImageY = initObj.iLeftEdgeInImageY
      }
      else {
        this.iLeftEdgeInImageY = [];
      }
      if (initObj.hasOwnProperty('iRightEdgeInImageX')) {
        this.iRightEdgeInImageX = initObj.iRightEdgeInImageX
      }
      else {
        this.iRightEdgeInImageX = [];
      }
      if (initObj.hasOwnProperty('iRightEdgeInImageY')) {
        this.iRightEdgeInImageY = initObj.iRightEdgeInImageY
      }
      else {
        this.iRightEdgeInImageY = [];
      }
      if (initObj.hasOwnProperty('iHeightInImage')) {
        this.iHeightInImage = initObj.iHeightInImage
      }
      else {
        this.iHeightInImage = [];
      }
      if (initObj.hasOwnProperty('fDistance')) {
        this.fDistance = initObj.fDistance
      }
      else {
        this.fDistance = [];
      }
      if (initObj.hasOwnProperty('fAngle')) {
        this.fAngle = initObj.fAngle
      }
      else {
        this.fAngle = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Obstacle
    // Serialize message field [bObstacleWasSeen]
    bufferOffset = _serializer.uint8(obj.bObstacleWasSeen, buffer, bufferOffset);
    // Serialize message field [iObstacleNumber]
    bufferOffset = _serializer.uint16(obj.iObstacleNumber, buffer, bufferOffset);
    // Serialize message field [iLeftEdgeInImageX]
    bufferOffset = _arraySerializer.uint16(obj.iLeftEdgeInImageX, buffer, bufferOffset, null);
    // Serialize message field [iLeftEdgeInImageY]
    bufferOffset = _arraySerializer.uint16(obj.iLeftEdgeInImageY, buffer, bufferOffset, null);
    // Serialize message field [iRightEdgeInImageX]
    bufferOffset = _arraySerializer.uint16(obj.iRightEdgeInImageX, buffer, bufferOffset, null);
    // Serialize message field [iRightEdgeInImageY]
    bufferOffset = _arraySerializer.uint16(obj.iRightEdgeInImageY, buffer, bufferOffset, null);
    // Serialize message field [iHeightInImage]
    bufferOffset = _arraySerializer.uint16(obj.iHeightInImage, buffer, bufferOffset, null);
    // Serialize message field [fDistance]
    bufferOffset = _arraySerializer.float32(obj.fDistance, buffer, bufferOffset, null);
    // Serialize message field [fAngle]
    bufferOffset = _arraySerializer.float32(obj.fAngle, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Obstacle
    let len;
    let data = new Obstacle(null);
    // Deserialize message field [bObstacleWasSeen]
    data.bObstacleWasSeen = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [iObstacleNumber]
    data.iObstacleNumber = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [iLeftEdgeInImageX]
    data.iLeftEdgeInImageX = _arrayDeserializer.uint16(buffer, bufferOffset, null)
    // Deserialize message field [iLeftEdgeInImageY]
    data.iLeftEdgeInImageY = _arrayDeserializer.uint16(buffer, bufferOffset, null)
    // Deserialize message field [iRightEdgeInImageX]
    data.iRightEdgeInImageX = _arrayDeserializer.uint16(buffer, bufferOffset, null)
    // Deserialize message field [iRightEdgeInImageY]
    data.iRightEdgeInImageY = _arrayDeserializer.uint16(buffer, bufferOffset, null)
    // Deserialize message field [iHeightInImage]
    data.iHeightInImage = _arrayDeserializer.uint16(buffer, bufferOffset, null)
    // Deserialize message field [fDistance]
    data.fDistance = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [fAngle]
    data.fAngle = _arrayDeserializer.float32(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 2 * object.iLeftEdgeInImageX.length;
    length += 2 * object.iLeftEdgeInImageY.length;
    length += 2 * object.iRightEdgeInImageX.length;
    length += 2 * object.iRightEdgeInImageY.length;
    length += 2 * object.iHeightInImage.length;
    length += 4 * object.fDistance.length;
    length += 4 * object.fAngle.length;
    return length + 31;
  }

  static datatype() {
    // Returns string type for a message object
    return 'decision/Obstacle';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'da6170297c4a034fb40d276bbabe829c';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    uint8 bObstacleWasSeen
    
    uint16 iObstacleNumber  #all together, if 2 obstacles seen, it's 2
    uint16[] iLeftEdgeInImageX
    uint16[] iLeftEdgeInImageY
    uint16[] iRightEdgeInImageX
    uint16[] iRightEdgeInImageY
    uint16[] iHeightInImage
    float32[] fDistance
    float32[] fAngle
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Obstacle(null);
    if (msg.bObstacleWasSeen !== undefined) {
      resolved.bObstacleWasSeen = msg.bObstacleWasSeen;
    }
    else {
      resolved.bObstacleWasSeen = 0
    }

    if (msg.iObstacleNumber !== undefined) {
      resolved.iObstacleNumber = msg.iObstacleNumber;
    }
    else {
      resolved.iObstacleNumber = 0
    }

    if (msg.iLeftEdgeInImageX !== undefined) {
      resolved.iLeftEdgeInImageX = msg.iLeftEdgeInImageX;
    }
    else {
      resolved.iLeftEdgeInImageX = []
    }

    if (msg.iLeftEdgeInImageY !== undefined) {
      resolved.iLeftEdgeInImageY = msg.iLeftEdgeInImageY;
    }
    else {
      resolved.iLeftEdgeInImageY = []
    }

    if (msg.iRightEdgeInImageX !== undefined) {
      resolved.iRightEdgeInImageX = msg.iRightEdgeInImageX;
    }
    else {
      resolved.iRightEdgeInImageX = []
    }

    if (msg.iRightEdgeInImageY !== undefined) {
      resolved.iRightEdgeInImageY = msg.iRightEdgeInImageY;
    }
    else {
      resolved.iRightEdgeInImageY = []
    }

    if (msg.iHeightInImage !== undefined) {
      resolved.iHeightInImage = msg.iHeightInImage;
    }
    else {
      resolved.iHeightInImage = []
    }

    if (msg.fDistance !== undefined) {
      resolved.fDistance = msg.fDistance;
    }
    else {
      resolved.fDistance = []
    }

    if (msg.fAngle !== undefined) {
      resolved.fAngle = msg.fAngle;
    }
    else {
      resolved.fAngle = []
    }

    return resolved;
    }
};

module.exports = Obstacle;
