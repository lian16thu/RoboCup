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

class head_angle {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.angle_head_pitch = null;
      this.angle_head_yaw = null;
    }
    else {
      if (initObj.hasOwnProperty('angle_head_pitch')) {
        this.angle_head_pitch = initObj.angle_head_pitch
      }
      else {
        this.angle_head_pitch = 0.0;
      }
      if (initObj.hasOwnProperty('angle_head_yaw')) {
        this.angle_head_yaw = initObj.angle_head_yaw
      }
      else {
        this.angle_head_yaw = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type head_angle
    // Serialize message field [angle_head_pitch]
    bufferOffset = _serializer.float64(obj.angle_head_pitch, buffer, bufferOffset);
    // Serialize message field [angle_head_yaw]
    bufferOffset = _serializer.float64(obj.angle_head_yaw, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type head_angle
    let len;
    let data = new head_angle(null);
    // Deserialize message field [angle_head_pitch]
    data.angle_head_pitch = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [angle_head_yaw]
    data.angle_head_yaw = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 16;
  }

  static datatype() {
    // Returns string type for a message object
    return 'decision/head_angle';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '13464c771052649b3693335e0dc785b9';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64 angle_head_pitch	#pitch
    float64 angle_head_yaw  	#yaw
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new head_angle(null);
    if (msg.angle_head_pitch !== undefined) {
      resolved.angle_head_pitch = msg.angle_head_pitch;
    }
    else {
      resolved.angle_head_pitch = 0.0
    }

    if (msg.angle_head_yaw !== undefined) {
      resolved.angle_head_yaw = msg.angle_head_yaw;
    }
    else {
      resolved.angle_head_yaw = 0.0
    }

    return resolved;
    }
};

module.exports = head_angle;
