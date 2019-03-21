// Auto-generated. Do not edit!

// (in-package decision.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------

class gyro_euler {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.euler_angle = null;
    }
    else {
      if (initObj.hasOwnProperty('euler_angle')) {
        this.euler_angle = initObj.euler_angle
      }
      else {
        this.euler_angle = new geometry_msgs.msg.Vector3();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type gyro_euler
    // Serialize message field [euler_angle]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.euler_angle, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type gyro_euler
    let len;
    let data = new gyro_euler(null);
    // Deserialize message field [euler_angle]
    data.euler_angle = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 24;
  }

  static datatype() {
    // Returns string type for a message object
    return 'decision/gyro_euler';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'a0ed725f607be591af507cbf7836e5ed';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    geometry_msgs/Vector3 euler_angle
    
    ================================================================================
    MSG: geometry_msgs/Vector3
    # This represents a vector in free space. 
    # It is only meant to represent a direction. Therefore, it does not
    # make sense to apply a translation to it (e.g., when applying a 
    # generic rigid transformation to a Vector3, tf2 will only apply the
    # rotation). If you want your data to be translatable too, use the
    # geometry_msgs/Point message instead.
    
    float64 x
    float64 y
    float64 z
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new gyro_euler(null);
    if (msg.euler_angle !== undefined) {
      resolved.euler_angle = geometry_msgs.msg.Vector3.Resolve(msg.euler_angle)
    }
    else {
      resolved.euler_angle = new geometry_msgs.msg.Vector3()
    }

    return resolved;
    }
};

module.exports = gyro_euler;
