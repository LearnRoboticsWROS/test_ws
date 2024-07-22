// Auto-generated. Do not edit!

// (in-package position_tracker.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class GetPositionRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.get_position = null;
    }
    else {
      if (initObj.hasOwnProperty('get_position')) {
        this.get_position = initObj.get_position
      }
      else {
        this.get_position = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetPositionRequest
    // Serialize message field [get_position]
    bufferOffset = _serializer.bool(obj.get_position, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetPositionRequest
    let len;
    let data = new GetPositionRequest(null);
    // Deserialize message field [get_position]
    data.get_position = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'position_tracker/GetPositionRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'a61a6675ded7cb2c8471dcd8afdc3b74';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool get_position
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetPositionRequest(null);
    if (msg.get_position !== undefined) {
      resolved.get_position = msg.get_position;
    }
    else {
      resolved.get_position = false
    }

    return resolved;
    }
};

class GetPositionResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.x_position = null;
      this.y_position = null;
      this.z_position = null;
    }
    else {
      if (initObj.hasOwnProperty('x_position')) {
        this.x_position = initObj.x_position
      }
      else {
        this.x_position = 0.0;
      }
      if (initObj.hasOwnProperty('y_position')) {
        this.y_position = initObj.y_position
      }
      else {
        this.y_position = 0.0;
      }
      if (initObj.hasOwnProperty('z_position')) {
        this.z_position = initObj.z_position
      }
      else {
        this.z_position = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetPositionResponse
    // Serialize message field [x_position]
    bufferOffset = _serializer.float64(obj.x_position, buffer, bufferOffset);
    // Serialize message field [y_position]
    bufferOffset = _serializer.float64(obj.y_position, buffer, bufferOffset);
    // Serialize message field [z_position]
    bufferOffset = _serializer.float64(obj.z_position, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetPositionResponse
    let len;
    let data = new GetPositionResponse(null);
    // Deserialize message field [x_position]
    data.x_position = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [y_position]
    data.y_position = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [z_position]
    data.z_position = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 24;
  }

  static datatype() {
    // Returns string type for a service object
    return 'position_tracker/GetPositionResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '4a65f5a0cdef54b2e1dbb78f7873209a';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64 x_position
    float64 y_position
    float64 z_position
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetPositionResponse(null);
    if (msg.x_position !== undefined) {
      resolved.x_position = msg.x_position;
    }
    else {
      resolved.x_position = 0.0
    }

    if (msg.y_position !== undefined) {
      resolved.y_position = msg.y_position;
    }
    else {
      resolved.y_position = 0.0
    }

    if (msg.z_position !== undefined) {
      resolved.z_position = msg.z_position;
    }
    else {
      resolved.z_position = 0.0
    }

    return resolved;
    }
};

module.exports = {
  Request: GetPositionRequest,
  Response: GetPositionResponse,
  md5sum() { return '4931ed2ee82bda66c8bc53fd24c21c60'; },
  datatype() { return 'position_tracker/GetPosition'; }
};
