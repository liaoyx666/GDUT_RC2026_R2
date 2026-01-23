// Auto-generated. Do not edit!

// (in-package wheel_leg.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class wl_control {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.linear_vel = null;
      this.angular_vel = null;
      this.leg_pos = null;
    }
    else {
      if (initObj.hasOwnProperty('linear_vel')) {
        this.linear_vel = initObj.linear_vel
      }
      else {
        this.linear_vel = 0.0;
      }
      if (initObj.hasOwnProperty('angular_vel')) {
        this.angular_vel = initObj.angular_vel
      }
      else {
        this.angular_vel = 0.0;
      }
      if (initObj.hasOwnProperty('leg_pos')) {
        this.leg_pos = initObj.leg_pos
      }
      else {
        this.leg_pos = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type wl_control
    // Serialize message field [linear_vel]
    bufferOffset = _serializer.float64(obj.linear_vel, buffer, bufferOffset);
    // Serialize message field [angular_vel]
    bufferOffset = _serializer.float64(obj.angular_vel, buffer, bufferOffset);
    // Serialize message field [leg_pos]
    bufferOffset = _serializer.float64(obj.leg_pos, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type wl_control
    let len;
    let data = new wl_control(null);
    // Deserialize message field [linear_vel]
    data.linear_vel = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [angular_vel]
    data.angular_vel = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [leg_pos]
    data.leg_pos = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 24;
  }

  static datatype() {
    // Returns string type for a message object
    return 'wheel_leg/wl_control';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '7fc1c80748cd92d4ec0e0172340cea1e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64 linear_vel  # 线速度
    float64 angular_vel # 角速度
    float64 leg_pos  # 腿关节位置
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new wl_control(null);
    if (msg.linear_vel !== undefined) {
      resolved.linear_vel = msg.linear_vel;
    }
    else {
      resolved.linear_vel = 0.0
    }

    if (msg.angular_vel !== undefined) {
      resolved.angular_vel = msg.angular_vel;
    }
    else {
      resolved.angular_vel = 0.0
    }

    if (msg.leg_pos !== undefined) {
      resolved.leg_pos = msg.leg_pos;
    }
    else {
      resolved.leg_pos = 0.0
    }

    return resolved;
    }
};

module.exports = wl_control;
