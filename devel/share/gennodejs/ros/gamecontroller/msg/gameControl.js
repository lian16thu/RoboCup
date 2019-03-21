// Auto-generated. Do not edit!

// (in-package gamecontroller.msg)


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
      this.gameType = null;
      this.state = null;
      this.firstHalf = null;
      this.kickOffTeam = null;
      this.secondaryState = null;
      this.secondaryStateTeam = null;
      this.secondaryStateInfo = null;
      this.dropInTeam = null;
      this.dropInTime = null;
      this.secsRemaining = null;
      this.secondaryTime = null;
      this.score = null;
      this.penaltyShot = null;
      this.singleShots = null;
      this.penalty = null;
      this.secsTillUnpenalised = null;
      this.yellowCardCount = null;
      this.redCardCount = null;
    }
    else {
      if (initObj.hasOwnProperty('gameType')) {
        this.gameType = initObj.gameType
      }
      else {
        this.gameType = 0;
      }
      if (initObj.hasOwnProperty('state')) {
        this.state = initObj.state
      }
      else {
        this.state = 0;
      }
      if (initObj.hasOwnProperty('firstHalf')) {
        this.firstHalf = initObj.firstHalf
      }
      else {
        this.firstHalf = 0;
      }
      if (initObj.hasOwnProperty('kickOffTeam')) {
        this.kickOffTeam = initObj.kickOffTeam
      }
      else {
        this.kickOffTeam = 0;
      }
      if (initObj.hasOwnProperty('secondaryState')) {
        this.secondaryState = initObj.secondaryState
      }
      else {
        this.secondaryState = 0;
      }
      if (initObj.hasOwnProperty('secondaryStateTeam')) {
        this.secondaryStateTeam = initObj.secondaryStateTeam
      }
      else {
        this.secondaryStateTeam = 0;
      }
      if (initObj.hasOwnProperty('secondaryStateInfo')) {
        this.secondaryStateInfo = initObj.secondaryStateInfo
      }
      else {
        this.secondaryStateInfo = 0;
      }
      if (initObj.hasOwnProperty('dropInTeam')) {
        this.dropInTeam = initObj.dropInTeam
      }
      else {
        this.dropInTeam = 0;
      }
      if (initObj.hasOwnProperty('dropInTime')) {
        this.dropInTime = initObj.dropInTime
      }
      else {
        this.dropInTime = 0;
      }
      if (initObj.hasOwnProperty('secsRemaining')) {
        this.secsRemaining = initObj.secsRemaining
      }
      else {
        this.secsRemaining = 0;
      }
      if (initObj.hasOwnProperty('secondaryTime')) {
        this.secondaryTime = initObj.secondaryTime
      }
      else {
        this.secondaryTime = 0;
      }
      if (initObj.hasOwnProperty('score')) {
        this.score = initObj.score
      }
      else {
        this.score = 0;
      }
      if (initObj.hasOwnProperty('penaltyShot')) {
        this.penaltyShot = initObj.penaltyShot
      }
      else {
        this.penaltyShot = 0;
      }
      if (initObj.hasOwnProperty('singleShots')) {
        this.singleShots = initObj.singleShots
      }
      else {
        this.singleShots = 0;
      }
      if (initObj.hasOwnProperty('penalty')) {
        this.penalty = initObj.penalty
      }
      else {
        this.penalty = 0;
      }
      if (initObj.hasOwnProperty('secsTillUnpenalised')) {
        this.secsTillUnpenalised = initObj.secsTillUnpenalised
      }
      else {
        this.secsTillUnpenalised = 0;
      }
      if (initObj.hasOwnProperty('yellowCardCount')) {
        this.yellowCardCount = initObj.yellowCardCount
      }
      else {
        this.yellowCardCount = 0;
      }
      if (initObj.hasOwnProperty('redCardCount')) {
        this.redCardCount = initObj.redCardCount
      }
      else {
        this.redCardCount = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type gameControl
    // Serialize message field [gameType]
    bufferOffset = _serializer.uint8(obj.gameType, buffer, bufferOffset);
    // Serialize message field [state]
    bufferOffset = _serializer.uint8(obj.state, buffer, bufferOffset);
    // Serialize message field [firstHalf]
    bufferOffset = _serializer.uint8(obj.firstHalf, buffer, bufferOffset);
    // Serialize message field [kickOffTeam]
    bufferOffset = _serializer.uint8(obj.kickOffTeam, buffer, bufferOffset);
    // Serialize message field [secondaryState]
    bufferOffset = _serializer.uint8(obj.secondaryState, buffer, bufferOffset);
    // Serialize message field [secondaryStateTeam]
    bufferOffset = _serializer.uint8(obj.secondaryStateTeam, buffer, bufferOffset);
    // Serialize message field [secondaryStateInfo]
    bufferOffset = _serializer.uint8(obj.secondaryStateInfo, buffer, bufferOffset);
    // Serialize message field [dropInTeam]
    bufferOffset = _serializer.uint8(obj.dropInTeam, buffer, bufferOffset);
    // Serialize message field [dropInTime]
    bufferOffset = _serializer.uint16(obj.dropInTime, buffer, bufferOffset);
    // Serialize message field [secsRemaining]
    bufferOffset = _serializer.uint16(obj.secsRemaining, buffer, bufferOffset);
    // Serialize message field [secondaryTime]
    bufferOffset = _serializer.uint16(obj.secondaryTime, buffer, bufferOffset);
    // Serialize message field [score]
    bufferOffset = _serializer.uint8(obj.score, buffer, bufferOffset);
    // Serialize message field [penaltyShot]
    bufferOffset = _serializer.uint8(obj.penaltyShot, buffer, bufferOffset);
    // Serialize message field [singleShots]
    bufferOffset = _serializer.uint16(obj.singleShots, buffer, bufferOffset);
    // Serialize message field [penalty]
    bufferOffset = _serializer.uint8(obj.penalty, buffer, bufferOffset);
    // Serialize message field [secsTillUnpenalised]
    bufferOffset = _serializer.uint8(obj.secsTillUnpenalised, buffer, bufferOffset);
    // Serialize message field [yellowCardCount]
    bufferOffset = _serializer.uint8(obj.yellowCardCount, buffer, bufferOffset);
    // Serialize message field [redCardCount]
    bufferOffset = _serializer.uint8(obj.redCardCount, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type gameControl
    let len;
    let data = new gameControl(null);
    // Deserialize message field [gameType]
    data.gameType = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [state]
    data.state = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [firstHalf]
    data.firstHalf = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [kickOffTeam]
    data.kickOffTeam = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [secondaryState]
    data.secondaryState = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [secondaryStateTeam]
    data.secondaryStateTeam = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [secondaryStateInfo]
    data.secondaryStateInfo = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [dropInTeam]
    data.dropInTeam = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [dropInTime]
    data.dropInTime = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [secsRemaining]
    data.secsRemaining = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [secondaryTime]
    data.secondaryTime = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [score]
    data.score = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [penaltyShot]
    data.penaltyShot = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [singleShots]
    data.singleShots = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [penalty]
    data.penalty = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [secsTillUnpenalised]
    data.secsTillUnpenalised = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [yellowCardCount]
    data.yellowCardCount = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [redCardCount]
    data.redCardCount = _deserializer.uint8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 22;
  }

  static datatype() {
    // Returns string type for a message object
    return 'gamecontroller/gameControl';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '8187bc9e4bf6fa1896498e321b213f47';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    uint8 gameType
    uint8 state
    uint8 firstHalf
    uint8 kickOffTeam
    uint8 secondaryState
    uint8 secondaryStateTeam
    uint8 secondaryStateInfo
    uint8 dropInTeam
    uint16 dropInTime
    uint16 secsRemaining
    uint16 secondaryTime
    
    uint8 score
    uint8 penaltyShot
    uint16 singleShots
    
    uint8 penalty
    uint8 secsTillUnpenalised
    uint8 yellowCardCount
    uint8 redCardCount
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new gameControl(null);
    if (msg.gameType !== undefined) {
      resolved.gameType = msg.gameType;
    }
    else {
      resolved.gameType = 0
    }

    if (msg.state !== undefined) {
      resolved.state = msg.state;
    }
    else {
      resolved.state = 0
    }

    if (msg.firstHalf !== undefined) {
      resolved.firstHalf = msg.firstHalf;
    }
    else {
      resolved.firstHalf = 0
    }

    if (msg.kickOffTeam !== undefined) {
      resolved.kickOffTeam = msg.kickOffTeam;
    }
    else {
      resolved.kickOffTeam = 0
    }

    if (msg.secondaryState !== undefined) {
      resolved.secondaryState = msg.secondaryState;
    }
    else {
      resolved.secondaryState = 0
    }

    if (msg.secondaryStateTeam !== undefined) {
      resolved.secondaryStateTeam = msg.secondaryStateTeam;
    }
    else {
      resolved.secondaryStateTeam = 0
    }

    if (msg.secondaryStateInfo !== undefined) {
      resolved.secondaryStateInfo = msg.secondaryStateInfo;
    }
    else {
      resolved.secondaryStateInfo = 0
    }

    if (msg.dropInTeam !== undefined) {
      resolved.dropInTeam = msg.dropInTeam;
    }
    else {
      resolved.dropInTeam = 0
    }

    if (msg.dropInTime !== undefined) {
      resolved.dropInTime = msg.dropInTime;
    }
    else {
      resolved.dropInTime = 0
    }

    if (msg.secsRemaining !== undefined) {
      resolved.secsRemaining = msg.secsRemaining;
    }
    else {
      resolved.secsRemaining = 0
    }

    if (msg.secondaryTime !== undefined) {
      resolved.secondaryTime = msg.secondaryTime;
    }
    else {
      resolved.secondaryTime = 0
    }

    if (msg.score !== undefined) {
      resolved.score = msg.score;
    }
    else {
      resolved.score = 0
    }

    if (msg.penaltyShot !== undefined) {
      resolved.penaltyShot = msg.penaltyShot;
    }
    else {
      resolved.penaltyShot = 0
    }

    if (msg.singleShots !== undefined) {
      resolved.singleShots = msg.singleShots;
    }
    else {
      resolved.singleShots = 0
    }

    if (msg.penalty !== undefined) {
      resolved.penalty = msg.penalty;
    }
    else {
      resolved.penalty = 0
    }

    if (msg.secsTillUnpenalised !== undefined) {
      resolved.secsTillUnpenalised = msg.secsTillUnpenalised;
    }
    else {
      resolved.secsTillUnpenalised = 0
    }

    if (msg.yellowCardCount !== undefined) {
      resolved.yellowCardCount = msg.yellowCardCount;
    }
    else {
      resolved.yellowCardCount = 0
    }

    if (msg.redCardCount !== undefined) {
      resolved.redCardCount = msg.redCardCount;
    }
    else {
      resolved.redCardCount = 0
    }

    return resolved;
    }
};

module.exports = gameControl;
