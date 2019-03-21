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
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class OutputData {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.robotPose = null;
      this.robotPoseConfidence = null;
      this.bBallWasSeen = null;
      this.ballCenterInImage = null;
      this.ballDistance = null;
      this.ballAngle = null;
      this.ballCenterOnField = null;
      this.bObstacleWasSeen = null;
      this.iObstacleNumber = null;
      this.obstacleLeftEndInImage = null;
      this.obstacleRightEndInImage = null;
      this.obstacleDistance = null;
      this.obstacleAngle = null;
      this.obstacleCenterOnField = null;
      this.obstacleRadiusOnField = null;
      this.bKeeperWasSeen = null;
      this.keeperLeftStartOnField = null;
      this.keeperRightEndOnField = null;
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
      if (initObj.hasOwnProperty('bBallWasSeen')) {
        this.bBallWasSeen = initObj.bBallWasSeen
      }
      else {
        this.bBallWasSeen = false;
      }
      if (initObj.hasOwnProperty('ballCenterInImage')) {
        this.ballCenterInImage = initObj.ballCenterInImage
      }
      else {
        this.ballCenterInImage = new geometry_msgs.msg.Point();
      }
      if (initObj.hasOwnProperty('ballDistance')) {
        this.ballDistance = initObj.ballDistance
      }
      else {
        this.ballDistance = 0.0;
      }
      if (initObj.hasOwnProperty('ballAngle')) {
        this.ballAngle = initObj.ballAngle
      }
      else {
        this.ballAngle = 0.0;
      }
      if (initObj.hasOwnProperty('ballCenterOnField')) {
        this.ballCenterOnField = initObj.ballCenterOnField
      }
      else {
        this.ballCenterOnField = new geometry_msgs.msg.Point();
      }
      if (initObj.hasOwnProperty('bObstacleWasSeen')) {
        this.bObstacleWasSeen = initObj.bObstacleWasSeen
      }
      else {
        this.bObstacleWasSeen = false;
      }
      if (initObj.hasOwnProperty('iObstacleNumber')) {
        this.iObstacleNumber = initObj.iObstacleNumber
      }
      else {
        this.iObstacleNumber = 0;
      }
      if (initObj.hasOwnProperty('obstacleLeftEndInImage')) {
        this.obstacleLeftEndInImage = initObj.obstacleLeftEndInImage
      }
      else {
        this.obstacleLeftEndInImage = [];
      }
      if (initObj.hasOwnProperty('obstacleRightEndInImage')) {
        this.obstacleRightEndInImage = initObj.obstacleRightEndInImage
      }
      else {
        this.obstacleRightEndInImage = [];
      }
      if (initObj.hasOwnProperty('obstacleDistance')) {
        this.obstacleDistance = initObj.obstacleDistance
      }
      else {
        this.obstacleDistance = [];
      }
      if (initObj.hasOwnProperty('obstacleAngle')) {
        this.obstacleAngle = initObj.obstacleAngle
      }
      else {
        this.obstacleAngle = [];
      }
      if (initObj.hasOwnProperty('obstacleCenterOnField')) {
        this.obstacleCenterOnField = initObj.obstacleCenterOnField
      }
      else {
        this.obstacleCenterOnField = [];
      }
      if (initObj.hasOwnProperty('obstacleRadiusOnField')) {
        this.obstacleRadiusOnField = initObj.obstacleRadiusOnField
      }
      else {
        this.obstacleRadiusOnField = [];
      }
      if (initObj.hasOwnProperty('bKeeperWasSeen')) {
        this.bKeeperWasSeen = initObj.bKeeperWasSeen
      }
      else {
        this.bKeeperWasSeen = false;
      }
      if (initObj.hasOwnProperty('keeperLeftStartOnField')) {
        this.keeperLeftStartOnField = initObj.keeperLeftStartOnField
      }
      else {
        this.keeperLeftStartOnField = new geometry_msgs.msg.Point();
      }
      if (initObj.hasOwnProperty('keeperRightEndOnField')) {
        this.keeperRightEndOnField = initObj.keeperRightEndOnField
      }
      else {
        this.keeperRightEndOnField = new geometry_msgs.msg.Point();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type OutputData
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [robotPose]
    bufferOffset = geometry_msgs.msg.Pose2D.serialize(obj.robotPose, buffer, bufferOffset);
    // Serialize message field [robotPoseConfidence]
    bufferOffset = _serializer.float32(obj.robotPoseConfidence, buffer, bufferOffset);
    // Serialize message field [bBallWasSeen]
    bufferOffset = _serializer.bool(obj.bBallWasSeen, buffer, bufferOffset);
    // Serialize message field [ballCenterInImage]
    bufferOffset = geometry_msgs.msg.Point.serialize(obj.ballCenterInImage, buffer, bufferOffset);
    // Serialize message field [ballDistance]
    bufferOffset = _serializer.float32(obj.ballDistance, buffer, bufferOffset);
    // Serialize message field [ballAngle]
    bufferOffset = _serializer.float32(obj.ballAngle, buffer, bufferOffset);
    // Serialize message field [ballCenterOnField]
    bufferOffset = geometry_msgs.msg.Point.serialize(obj.ballCenterOnField, buffer, bufferOffset);
    // Serialize message field [bObstacleWasSeen]
    bufferOffset = _serializer.bool(obj.bObstacleWasSeen, buffer, bufferOffset);
    // Serialize message field [iObstacleNumber]
    bufferOffset = _serializer.int32(obj.iObstacleNumber, buffer, bufferOffset);
    // Serialize message field [obstacleLeftEndInImage]
    // Serialize the length for message field [obstacleLeftEndInImage]
    bufferOffset = _serializer.uint32(obj.obstacleLeftEndInImage.length, buffer, bufferOffset);
    obj.obstacleLeftEndInImage.forEach((val) => {
      bufferOffset = geometry_msgs.msg.Point.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [obstacleRightEndInImage]
    // Serialize the length for message field [obstacleRightEndInImage]
    bufferOffset = _serializer.uint32(obj.obstacleRightEndInImage.length, buffer, bufferOffset);
    obj.obstacleRightEndInImage.forEach((val) => {
      bufferOffset = geometry_msgs.msg.Point.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [obstacleDistance]
    bufferOffset = _arraySerializer.float32(obj.obstacleDistance, buffer, bufferOffset, null);
    // Serialize message field [obstacleAngle]
    bufferOffset = _arraySerializer.float32(obj.obstacleAngle, buffer, bufferOffset, null);
    // Serialize message field [obstacleCenterOnField]
    // Serialize the length for message field [obstacleCenterOnField]
    bufferOffset = _serializer.uint32(obj.obstacleCenterOnField.length, buffer, bufferOffset);
    obj.obstacleCenterOnField.forEach((val) => {
      bufferOffset = geometry_msgs.msg.Point.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [obstacleRadiusOnField]
    bufferOffset = _arraySerializer.float32(obj.obstacleRadiusOnField, buffer, bufferOffset, null);
    // Serialize message field [bKeeperWasSeen]
    bufferOffset = _serializer.bool(obj.bKeeperWasSeen, buffer, bufferOffset);
    // Serialize message field [keeperLeftStartOnField]
    bufferOffset = geometry_msgs.msg.Point.serialize(obj.keeperLeftStartOnField, buffer, bufferOffset);
    // Serialize message field [keeperRightEndOnField]
    bufferOffset = geometry_msgs.msg.Point.serialize(obj.keeperRightEndOnField, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type OutputData
    let len;
    let data = new OutputData(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [robotPose]
    data.robotPose = geometry_msgs.msg.Pose2D.deserialize(buffer, bufferOffset);
    // Deserialize message field [robotPoseConfidence]
    data.robotPoseConfidence = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [bBallWasSeen]
    data.bBallWasSeen = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [ballCenterInImage]
    data.ballCenterInImage = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset);
    // Deserialize message field [ballDistance]
    data.ballDistance = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [ballAngle]
    data.ballAngle = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [ballCenterOnField]
    data.ballCenterOnField = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset);
    // Deserialize message field [bObstacleWasSeen]
    data.bObstacleWasSeen = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [iObstacleNumber]
    data.iObstacleNumber = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [obstacleLeftEndInImage]
    // Deserialize array length for message field [obstacleLeftEndInImage]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.obstacleLeftEndInImage = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.obstacleLeftEndInImage[i] = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [obstacleRightEndInImage]
    // Deserialize array length for message field [obstacleRightEndInImage]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.obstacleRightEndInImage = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.obstacleRightEndInImage[i] = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [obstacleDistance]
    data.obstacleDistance = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [obstacleAngle]
    data.obstacleAngle = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [obstacleCenterOnField]
    // Deserialize array length for message field [obstacleCenterOnField]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.obstacleCenterOnField = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.obstacleCenterOnField[i] = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [obstacleRadiusOnField]
    data.obstacleRadiusOnField = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [bKeeperWasSeen]
    data.bKeeperWasSeen = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [keeperLeftStartOnField]
    data.keeperLeftStartOnField = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset);
    // Deserialize message field [keeperRightEndOnField]
    data.keeperRightEndOnField = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += 24 * object.obstacleLeftEndInImage.length;
    length += 24 * object.obstacleRightEndInImage.length;
    length += 4 * object.obstacleDistance.length;
    length += 4 * object.obstacleAngle.length;
    length += 24 * object.obstacleCenterOnField.length;
    length += 4 * object.obstacleRadiusOnField.length;
    return length + 163;
  }

  static datatype() {
    // Returns string type for a message object
    return 'decision/OutputData';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'bd064faf37c8b9975f133c15e75ac7aa';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    #	data published by the localization node.
    
    std_msgs/Header header 								# for time stamp
    geometry_msgs/Pose2D robotPose        				# Pose of the robot according to the particle filter localization	
    float32 robotPoseConfidence               			# confidence 0..1 for the robot_pose
    bool bBallWasSeen                               	# boolean variable for ball detection
    geometry_msgs/Point ballCenterInImage				# coordinates of ball center in the image
    float32 ballDistance								# ball distance from robot as seen in the image
    float32 ballAngle									# ball angle from center as seen in the image, [-90,90]
    geometry_msgs/Point ballCenterOnField				# coordinates of ball center on field relative to robot localization	
    bool bObstacleWasSeen                           	# boolean variable for obstacle detection
    int32 iObstacleNumber                           	# index of obstacle, if 2 found this is 2
    geometry_msgs/Point[] obstacleLeftEndInImage		# coordinates of obstacle left end point in the image
    geometry_msgs/Point[] obstacleRightEndInImage		# coordinates of obstacle right end point in the image
    float32[] obstacleDistance							# obstacle distance from robot as seen in the image
    float32[] obstacleAngle								# obstacle angle from center as seen in the image, [-90,90]
    geometry_msgs/Point[] obstacleCenterOnField			# coordinates of obstacle center on field relative to robot localization	
    float32[] obstacleRadiusOnField						# estimated radius of obstacle on field
    bool bKeeperWasSeen                             	# boolean variable for goalkeeper detection
    geometry_msgs/Point keeperLeftStartOnField      	# coordinates of goalkeeper left leg start point on field
    geometry_msgs/Point keeperRightEndOnField       	# coordinates of goalkeeper right leg end point on field
    
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
    
    ================================================================================
    MSG: geometry_msgs/Point
    # This contains the position of a point in free space
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
    const resolved = new OutputData(null);
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

    if (msg.bBallWasSeen !== undefined) {
      resolved.bBallWasSeen = msg.bBallWasSeen;
    }
    else {
      resolved.bBallWasSeen = false
    }

    if (msg.ballCenterInImage !== undefined) {
      resolved.ballCenterInImage = geometry_msgs.msg.Point.Resolve(msg.ballCenterInImage)
    }
    else {
      resolved.ballCenterInImage = new geometry_msgs.msg.Point()
    }

    if (msg.ballDistance !== undefined) {
      resolved.ballDistance = msg.ballDistance;
    }
    else {
      resolved.ballDistance = 0.0
    }

    if (msg.ballAngle !== undefined) {
      resolved.ballAngle = msg.ballAngle;
    }
    else {
      resolved.ballAngle = 0.0
    }

    if (msg.ballCenterOnField !== undefined) {
      resolved.ballCenterOnField = geometry_msgs.msg.Point.Resolve(msg.ballCenterOnField)
    }
    else {
      resolved.ballCenterOnField = new geometry_msgs.msg.Point()
    }

    if (msg.bObstacleWasSeen !== undefined) {
      resolved.bObstacleWasSeen = msg.bObstacleWasSeen;
    }
    else {
      resolved.bObstacleWasSeen = false
    }

    if (msg.iObstacleNumber !== undefined) {
      resolved.iObstacleNumber = msg.iObstacleNumber;
    }
    else {
      resolved.iObstacleNumber = 0
    }

    if (msg.obstacleLeftEndInImage !== undefined) {
      resolved.obstacleLeftEndInImage = new Array(msg.obstacleLeftEndInImage.length);
      for (let i = 0; i < resolved.obstacleLeftEndInImage.length; ++i) {
        resolved.obstacleLeftEndInImage[i] = geometry_msgs.msg.Point.Resolve(msg.obstacleLeftEndInImage[i]);
      }
    }
    else {
      resolved.obstacleLeftEndInImage = []
    }

    if (msg.obstacleRightEndInImage !== undefined) {
      resolved.obstacleRightEndInImage = new Array(msg.obstacleRightEndInImage.length);
      for (let i = 0; i < resolved.obstacleRightEndInImage.length; ++i) {
        resolved.obstacleRightEndInImage[i] = geometry_msgs.msg.Point.Resolve(msg.obstacleRightEndInImage[i]);
      }
    }
    else {
      resolved.obstacleRightEndInImage = []
    }

    if (msg.obstacleDistance !== undefined) {
      resolved.obstacleDistance = msg.obstacleDistance;
    }
    else {
      resolved.obstacleDistance = []
    }

    if (msg.obstacleAngle !== undefined) {
      resolved.obstacleAngle = msg.obstacleAngle;
    }
    else {
      resolved.obstacleAngle = []
    }

    if (msg.obstacleCenterOnField !== undefined) {
      resolved.obstacleCenterOnField = new Array(msg.obstacleCenterOnField.length);
      for (let i = 0; i < resolved.obstacleCenterOnField.length; ++i) {
        resolved.obstacleCenterOnField[i] = geometry_msgs.msg.Point.Resolve(msg.obstacleCenterOnField[i]);
      }
    }
    else {
      resolved.obstacleCenterOnField = []
    }

    if (msg.obstacleRadiusOnField !== undefined) {
      resolved.obstacleRadiusOnField = msg.obstacleRadiusOnField;
    }
    else {
      resolved.obstacleRadiusOnField = []
    }

    if (msg.bKeeperWasSeen !== undefined) {
      resolved.bKeeperWasSeen = msg.bKeeperWasSeen;
    }
    else {
      resolved.bKeeperWasSeen = false
    }

    if (msg.keeperLeftStartOnField !== undefined) {
      resolved.keeperLeftStartOnField = geometry_msgs.msg.Point.Resolve(msg.keeperLeftStartOnField)
    }
    else {
      resolved.keeperLeftStartOnField = new geometry_msgs.msg.Point()
    }

    if (msg.keeperRightEndOnField !== undefined) {
      resolved.keeperRightEndOnField = geometry_msgs.msg.Point.Resolve(msg.keeperRightEndOnField)
    }
    else {
      resolved.keeperRightEndOnField = new geometry_msgs.msg.Point()
    }

    return resolved;
    }
};

module.exports = OutputData;
