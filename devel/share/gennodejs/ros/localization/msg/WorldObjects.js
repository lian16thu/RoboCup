// Auto-generated. Do not edit!

// (in-package localization.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let ObjectsDetected = require('./ObjectsDetected.js');
let GoalpostsDetected = require('./GoalpostsDetected.js');
let LinesDetected = require('./LinesDetected.js');
let ObstaclesDetected = require('./ObstaclesDetected.js');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class WorldObjects {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.objects = null;
      this.goalposts = null;
      this.lines = null;
      this.obstacles = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('objects')) {
        this.objects = initObj.objects
      }
      else {
        this.objects = [];
      }
      if (initObj.hasOwnProperty('goalposts')) {
        this.goalposts = initObj.goalposts
      }
      else {
        this.goalposts = [];
      }
      if (initObj.hasOwnProperty('lines')) {
        this.lines = initObj.lines
      }
      else {
        this.lines = [];
      }
      if (initObj.hasOwnProperty('obstacles')) {
        this.obstacles = initObj.obstacles
      }
      else {
        this.obstacles = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type WorldObjects
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [objects]
    // Serialize the length for message field [objects]
    bufferOffset = _serializer.uint32(obj.objects.length, buffer, bufferOffset);
    obj.objects.forEach((val) => {
      bufferOffset = ObjectsDetected.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [goalposts]
    // Serialize the length for message field [goalposts]
    bufferOffset = _serializer.uint32(obj.goalposts.length, buffer, bufferOffset);
    obj.goalposts.forEach((val) => {
      bufferOffset = GoalpostsDetected.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [lines]
    // Serialize the length for message field [lines]
    bufferOffset = _serializer.uint32(obj.lines.length, buffer, bufferOffset);
    obj.lines.forEach((val) => {
      bufferOffset = LinesDetected.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [obstacles]
    // Serialize the length for message field [obstacles]
    bufferOffset = _serializer.uint32(obj.obstacles.length, buffer, bufferOffset);
    obj.obstacles.forEach((val) => {
      bufferOffset = ObstaclesDetected.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type WorldObjects
    let len;
    let data = new WorldObjects(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [objects]
    // Deserialize array length for message field [objects]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.objects = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.objects[i] = ObjectsDetected.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [goalposts]
    // Deserialize array length for message field [goalposts]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.goalposts = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.goalposts[i] = GoalpostsDetected.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [lines]
    // Deserialize array length for message field [lines]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.lines = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.lines[i] = LinesDetected.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [obstacles]
    // Deserialize array length for message field [obstacles]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.obstacles = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.obstacles[i] = ObstaclesDetected.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += 29 * object.objects.length;
    length += 29 * object.goalposts.length;
    length += 16 * object.lines.length;
    length += 29 * object.obstacles.length;
    return length + 16;
  }

  static datatype() {
    // Returns string type for a message object
    return 'localization/WorldObjects';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'b3b402e0a736a3c111358cf72848e29c';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    #add more for the lines and such, like in detections
    ObjectsDetected[] objects
    GoalpostsDetected[] goalposts
    LinesDetected[] lines
    ObstaclesDetected[] obstacles
    
    
    
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
    MSG: localization/ObjectsDetected
    geometry_msgs/Pose2D pose        # Pose
    uint8 type                       # Type (see field_model::WorldObject::Type)
    float32 confidence               # confidence 0..1
    
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
    
    ================================================================================
    MSG: localization/GoalpostsDetected
    geometry_msgs/Pose2D pose        # Pose
    uint8 type                       # Type (see field_model::WorldObject::Type)
    float32 confidence               # confidence 0..1
    
    ================================================================================
    MSG: localization/LinesDetected
    float32 x1
    float32 y1
    float32 x2
    float32 y2
    
    ================================================================================
    MSG: localization/ObstaclesDetected
    geometry_msgs/Pose2D pose        # Pose
    uint8 type                       # Type 0 for opponent, 1 for other obstacle
    float32 confidence               # confidence 0..1
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new WorldObjects(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.objects !== undefined) {
      resolved.objects = new Array(msg.objects.length);
      for (let i = 0; i < resolved.objects.length; ++i) {
        resolved.objects[i] = ObjectsDetected.Resolve(msg.objects[i]);
      }
    }
    else {
      resolved.objects = []
    }

    if (msg.goalposts !== undefined) {
      resolved.goalposts = new Array(msg.goalposts.length);
      for (let i = 0; i < resolved.goalposts.length; ++i) {
        resolved.goalposts[i] = GoalpostsDetected.Resolve(msg.goalposts[i]);
      }
    }
    else {
      resolved.goalposts = []
    }

    if (msg.lines !== undefined) {
      resolved.lines = new Array(msg.lines.length);
      for (let i = 0; i < resolved.lines.length; ++i) {
        resolved.lines[i] = LinesDetected.Resolve(msg.lines[i]);
      }
    }
    else {
      resolved.lines = []
    }

    if (msg.obstacles !== undefined) {
      resolved.obstacles = new Array(msg.obstacles.length);
      for (let i = 0; i < resolved.obstacles.length; ++i) {
        resolved.obstacles[i] = ObstaclesDetected.Resolve(msg.obstacles[i]);
      }
    }
    else {
      resolved.obstacles = []
    }

    return resolved;
    }
};

module.exports = WorldObjects;
