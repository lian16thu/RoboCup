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

class Obstacle {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.bObstacleWasSeen = null;
      this.iLeftEdgeInImageX = null;
      this.iLeftEdgeInImageY = null;
      this.iRightEdgeInImageX = null;
      this.iRightEdgeInImageY = null;
      this.iHeightInImage = null;
      this.inumber = null;
      this.iOthersLeftEdgeInImageX = null;
      this.iOthersRightEdgeInImageX = null;
      this.iOthersInImageY = null;
      this.iOthersHeightInImage = null;
    }
    else {
      if (initObj.hasOwnProperty('bObstacleWasSeen')) {
        this.bObstacleWasSeen = initObj.bObstacleWasSeen
      }
      else {
        this.bObstacleWasSeen = 0;
      }
      if (initObj.hasOwnProperty('iLeftEdgeInImageX')) {
        this.iLeftEdgeInImageX = initObj.iLeftEdgeInImageX
      }
      else {
        this.iLeftEdgeInImageX = 0;
      }
      if (initObj.hasOwnProperty('iLeftEdgeInImageY')) {
        this.iLeftEdgeInImageY = initObj.iLeftEdgeInImageY
      }
      else {
        this.iLeftEdgeInImageY = 0;
      }
      if (initObj.hasOwnProperty('iRightEdgeInImageX')) {
        this.iRightEdgeInImageX = initObj.iRightEdgeInImageX
      }
      else {
        this.iRightEdgeInImageX = 0;
      }
      if (initObj.hasOwnProperty('iRightEdgeInImageY')) {
        this.iRightEdgeInImageY = initObj.iRightEdgeInImageY
      }
      else {
        this.iRightEdgeInImageY = 0;
      }
      if (initObj.hasOwnProperty('iHeightInImage')) {
        this.iHeightInImage = initObj.iHeightInImage
      }
      else {
        this.iHeightInImage = 0;
      }
      if (initObj.hasOwnProperty('inumber')) {
        this.inumber = initObj.inumber
      }
      else {
        this.inumber = 0;
      }
      if (initObj.hasOwnProperty('iOthersLeftEdgeInImageX')) {
        this.iOthersLeftEdgeInImageX = initObj.iOthersLeftEdgeInImageX
      }
      else {
        this.iOthersLeftEdgeInImageX = [];
      }
      if (initObj.hasOwnProperty('iOthersRightEdgeInImageX')) {
        this.iOthersRightEdgeInImageX = initObj.iOthersRightEdgeInImageX
      }
      else {
        this.iOthersRightEdgeInImageX = [];
      }
      if (initObj.hasOwnProperty('iOthersInImageY')) {
        this.iOthersInImageY = initObj.iOthersInImageY
      }
      else {
        this.iOthersInImageY = [];
      }
      if (initObj.hasOwnProperty('iOthersHeightInImage')) {
        this.iOthersHeightInImage = initObj.iOthersHeightInImage
      }
      else {
        this.iOthersHeightInImage = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Obstacle
    // Serialize message field [bObstacleWasSeen]
    bufferOffset = _serializer.uint8(obj.bObstacleWasSeen, buffer, bufferOffset);
    // Serialize message field [iLeftEdgeInImageX]
    bufferOffset = _serializer.uint16(obj.iLeftEdgeInImageX, buffer, bufferOffset);
    // Serialize message field [iLeftEdgeInImageY]
    bufferOffset = _serializer.uint16(obj.iLeftEdgeInImageY, buffer, bufferOffset);
    // Serialize message field [iRightEdgeInImageX]
    bufferOffset = _serializer.uint16(obj.iRightEdgeInImageX, buffer, bufferOffset);
    // Serialize message field [iRightEdgeInImageY]
    bufferOffset = _serializer.uint16(obj.iRightEdgeInImageY, buffer, bufferOffset);
    // Serialize message field [iHeightInImage]
    bufferOffset = _serializer.uint16(obj.iHeightInImage, buffer, bufferOffset);
    // Serialize message field [inumber]
    bufferOffset = _serializer.uint16(obj.inumber, buffer, bufferOffset);
    // Serialize message field [iOthersLeftEdgeInImageX]
    bufferOffset = _arraySerializer.uint16(obj.iOthersLeftEdgeInImageX, buffer, bufferOffset, null);
    // Serialize message field [iOthersRightEdgeInImageX]
    bufferOffset = _arraySerializer.uint16(obj.iOthersRightEdgeInImageX, buffer, bufferOffset, null);
    // Serialize message field [iOthersInImageY]
    bufferOffset = _arraySerializer.uint16(obj.iOthersInImageY, buffer, bufferOffset, null);
    // Serialize message field [iOthersHeightInImage]
    bufferOffset = _arraySerializer.uint16(obj.iOthersHeightInImage, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Obstacle
    let len;
    let data = new Obstacle(null);
    // Deserialize message field [bObstacleWasSeen]
    data.bObstacleWasSeen = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [iLeftEdgeInImageX]
    data.iLeftEdgeInImageX = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [iLeftEdgeInImageY]
    data.iLeftEdgeInImageY = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [iRightEdgeInImageX]
    data.iRightEdgeInImageX = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [iRightEdgeInImageY]
    data.iRightEdgeInImageY = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [iHeightInImage]
    data.iHeightInImage = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [inumber]
    data.inumber = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [iOthersLeftEdgeInImageX]
    data.iOthersLeftEdgeInImageX = _arrayDeserializer.uint16(buffer, bufferOffset, null)
    // Deserialize message field [iOthersRightEdgeInImageX]
    data.iOthersRightEdgeInImageX = _arrayDeserializer.uint16(buffer, bufferOffset, null)
    // Deserialize message field [iOthersInImageY]
    data.iOthersInImageY = _arrayDeserializer.uint16(buffer, bufferOffset, null)
    // Deserialize message field [iOthersHeightInImage]
    data.iOthersHeightInImage = _arrayDeserializer.uint16(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 2 * object.iOthersLeftEdgeInImageX.length;
    length += 2 * object.iOthersRightEdgeInImageX.length;
    length += 2 * object.iOthersInImageY.length;
    length += 2 * object.iOthersHeightInImage.length;
    return length + 29;
  }

  static datatype() {
    // Returns string type for a message object
    return 'vision/Obstacle';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'b7ab1788ead7712018d8f4217af3a9ee';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    uint8 bObstacleWasSeen
    
    uint16 iLeftEdgeInImageX
    uint16 iLeftEdgeInImageY
    uint16 iRightEdgeInImageX
    uint16 iRightEdgeInImageY
    uint16 iHeightInImage
    
    uint16 inumber # apart from first obstacle, if there were 2 obstacles found, this is 1
    uint16[] iOthersLeftEdgeInImageX
    uint16[] iOthersRightEdgeInImageX
    uint16[] iOthersInImageY  # the same Y for both left and right
    uint16[] iOthersHeightInImage
    
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

    if (msg.iLeftEdgeInImageX !== undefined) {
      resolved.iLeftEdgeInImageX = msg.iLeftEdgeInImageX;
    }
    else {
      resolved.iLeftEdgeInImageX = 0
    }

    if (msg.iLeftEdgeInImageY !== undefined) {
      resolved.iLeftEdgeInImageY = msg.iLeftEdgeInImageY;
    }
    else {
      resolved.iLeftEdgeInImageY = 0
    }

    if (msg.iRightEdgeInImageX !== undefined) {
      resolved.iRightEdgeInImageX = msg.iRightEdgeInImageX;
    }
    else {
      resolved.iRightEdgeInImageX = 0
    }

    if (msg.iRightEdgeInImageY !== undefined) {
      resolved.iRightEdgeInImageY = msg.iRightEdgeInImageY;
    }
    else {
      resolved.iRightEdgeInImageY = 0
    }

    if (msg.iHeightInImage !== undefined) {
      resolved.iHeightInImage = msg.iHeightInImage;
    }
    else {
      resolved.iHeightInImage = 0
    }

    if (msg.inumber !== undefined) {
      resolved.inumber = msg.inumber;
    }
    else {
      resolved.inumber = 0
    }

    if (msg.iOthersLeftEdgeInImageX !== undefined) {
      resolved.iOthersLeftEdgeInImageX = msg.iOthersLeftEdgeInImageX;
    }
    else {
      resolved.iOthersLeftEdgeInImageX = []
    }

    if (msg.iOthersRightEdgeInImageX !== undefined) {
      resolved.iOthersRightEdgeInImageX = msg.iOthersRightEdgeInImageX;
    }
    else {
      resolved.iOthersRightEdgeInImageX = []
    }

    if (msg.iOthersInImageY !== undefined) {
      resolved.iOthersInImageY = msg.iOthersInImageY;
    }
    else {
      resolved.iOthersInImageY = []
    }

    if (msg.iOthersHeightInImage !== undefined) {
      resolved.iOthersHeightInImage = msg.iOthersHeightInImage;
    }
    else {
      resolved.iOthersHeightInImage = []
    }

    return resolved;
    }
};

module.exports = Obstacle;
