// Auto-generated. Do not edit!

// (in-package localization.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let geometry_msgs = _finder('geometry_msgs');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class MeanPoseConfStamped {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.robotPose = null;
      this.robotPoseConfidence = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('robotPose')) {
        this.robotPose = initObj.robotPose
      }
      else {
        this.robotPose = new geometry_msgs.msg.Pose2D();
      }
      if (initObj.hasOwnProperty('robotPoseConfidence')) {
        this.robotPoseConfidence = initObj.robotPoseConfidence
      }
      else {
        this.robotPoseConfidence = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type MeanPoseConfStamped
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [robotPose]
    bufferOffset = geometry_msgs.msg.Pose2D.serialize(obj.robotPose, buffer, bufferOffset);
    // Serialize message field [robotPoseConfidence]
    bufferOffset = _serializer.float32(obj.robotPoseConfidence, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type MeanPoseConfStamped
    let len;
    let data = new MeanPoseConfStamped(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [robotPose]
    data.robotPose = geometry_msgs.msg.Pose2D.deserialize(buffer, bufferOffset);
    // Deserialize message field [robotPoseConfidence]
    data.robotPoseConfidence = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 28;
  }

  static datatype() {
    // Returns string type for a message object
    return 'localization/MeanPoseConfStamped';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '87d282eb9ec7ff5b36abcb25698c079d';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header 				# for time stamp
    geometry_msgs/Pose2D robotPose        		# Pose of the robot according to the particle filter localization
    float32 robotPoseConfidence
    
    ================================================================================
    MSG: std_msgs/Header
    # Standard metadata for higher-level stamped data types.
    # This is generally used to communicate timestamped data 
    # in a particular coordinate frame.
    # 
    # sequence ID: consecutively increasing ID 
    uint32 seq
    #Two-integer timestamp that is expressed as:
    # * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
    # * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
    # time-handling sugar is provided by the client library
    time stamp
    #Frame this data is associated with
    # 0: no frame
    # 1: global frame
    string frame_id
    
    ================================================================================
    MSG: geometry_msgs/Pose2D
    # Deprecated
    # Please use the full 3D pose.
    
    # In general our recommendation is to use a full 3D representation of everything and for 2D specific applications make the appropriate projections into the plane for their calculations but optimally will preserve the 3D information during processing.
    
    # If we have parallel copies of 2D datatypes every UI and other pipeline will end up needing to have dual interfaces to plot everything. And you will end up with not being able to use 3D tools for 2D use cases even if they're completely valid, as you'd have to reimplement it with different inputs and outputs. It's not particularly hard to plot the 2D pose or compute the yaw error for the Pose message and there are already tools and libraries that can do this for you.
    
    
    # This expresses a position and orientation on a 2D manifold.
    
    float64 x
    float64 y
    float64 theta
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new MeanPoseConfStamped(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.robotPose !== undefined) {
      resolved.robotPose = geometry_msgs.msg.Pose2D.Resolve(msg.robotPose)
    }
    else {
      resolved.robotPose = new geometry_msgs.msg.Pose2D()
    }

    if (msg.robotPoseConfidence !== undefined) {
      resolved.robotPoseConfidence = msg.robotPoseConfidence;
    }
    else {
      resolved.robotPoseConfidence = 0.0
    }

    return resolved;
    }
};

module.exports = MeanPoseConfStamped;
