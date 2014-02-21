// 
// Hekateros control rosbridge handle
// 
// required js imports:
//    http://cdn.robotwebtools.org/EventEmitter2/current/eventemitter2.min.js
//    http://cdn.robotwebtools.org/roslibjs/current/roslib.min.js
//    rosbridge_global.js
//

function hekateros(throttle_rate) {
  // set default throttle rate = 0
  this.throttle_rate = typeof throttle_rate !== 'undefined' ? throttle_rate : 0;
  
  this.sub_state = function(cb) {
    if(typeof cb == 'undefined') {
      console.error("When subscribing to a topic, you must provide a callback");
      return;
   }
    this.robot_status.subscribe(function(msg){cb(msg);});
  }


  this.calibrate = function(cb) {
    cb= typeof cb !== 'undefined' ? cb : function(rsp){};
    var msg = {force_recalib:1};
    var goal = new ROSLIB.Goal({ 
      actionClient: this.calibrate_client,
      goalMessage: msg
    });
    return goal;
  }

  this.sub_feedback = function(cb) {
    if(typeof cb == 'undefined') {
      console.error("When subscribing to a topic, you must provide a callback");
      return;
    }
    this.calibrate_feedback.subscribe(function(msg){cb(msg);});
  }
  
  this.sub_robotStatus = function(cb) {
    if(typeof cb == 'undefined') {
      console.error("When subscribing to a topic, you must provide a callback");
      return;
    }
    this.robot_status_EX.subscribe(function(msg){cb(msg);});
  }
  
  this.sub_jointStates = function(cb) {
    if(typeof cb == 'undefined') {
      console.error("When subscribing to a topic, you must provide a callback");
      return;
    }
    this.joint_states_EX.subscribe(function(msg){cb(msg);});
  }

  this.EStop = function(cb) {
    cb = typeof cb !== 'undefined' ? cb : function(rsp){};
    var req = new ROSLIB.ServiceRequest({});
    this.EStop_srv.callService(req, function(rsp){cb(rsp);});
  }

  this.resetEStop = function(cb) {
    cb = typeof cb !== 'underfined' ? cb : function(rsp){};
    var req = new ROSLIB.serviceRequest({});
    this.resetEStop_srv.callService(req, function(rsp){cb(rsp);});
  }
  
  this.freeze = function(cb) {
    cb = typeof cb !== 'underfined' ? cb : function(rsp){};
    var req = new ROSLIB.serviceRequest({});
    this.freeze_srv.callService(req, function(rsp){cb(rsp);});
  }
  
  this.release = function(cb) {
    cb = typeof cb !== 'underfined' ? cb : function(rsp){};
    var req = new ROSLIB.serviceRequest({});
    this.release_srv.callService(req, function(rsp){cb(rsp);});
  }
  
  this.park = function(cb) {
    cb = typeof cb !== 'undefined' ? cb : function(rsp){};
    var req = new ROSLIB.ServiceRequest({});
    this.goToParkedPos_srv.callService(req, function(rsp){cb(rsp);});
  }
  
  this.balanced = function(cb) {
    cb = typeof cb !== 'undefined' ? cb : function(rsp){};
    var req = new ROSLIB.ServiceRequest({});
    this.goToBalancedPos_srv.callService(req, function(rsp){cb(rsp);});
  }

  this.zeroPoint = function(cb) {
    cb = typeof cb !== 'undefined' ? cb : function(rsp){};
    var req = new ROSLIB.ServiceRequest({});
    this.goToZeroPt_srv.callService(req, function(rsp){cb(rsp);});
  }

  this.openGripper = function(cb) {
    cb = typeof cb !== 'undefined' ? cb : function(rsp){};
    var req = new ROSLIB.ServiceRequest({});
    this.openGripper_srv.callService(req, function(rsp){cb(rsp);});
  }

  this.closeGripper = function(cb) {
    cb = typeof cb !== 'undefined' ? cb : function(rsp){};
    var req = new ROSLIB.ServiceRequest({});
    this.closeGripper_srv.callService(req, function(rsp){cb(rsp);});
  }
    
  this.clearAlarms = function(cb) {
    cb = typeof cb !== 'undefined' ? cb : function(rsp){};
    var req = new ROSLIB.ServiceRequest({});
    this.clearAlarms_srv.callService(req, function(rsp){cb(rsp);});
  }

  this.ProductInfo = function(cb) {
    cb = typeof cb !== 'undefined' ? cb : function(rsp){};
    var req = new ROSLIB.ServiceRequest({});
    this.getProductInfo_srv.callService(req, function(rsp){cb(rsp);});
  }

  this.isCalibrated = function(message, cb) {
    cb = typeof cb !== 'undefined' ? cb : function(rsp){};
    var msg = new ROSLIB.Message({
      val:message,
    });

    var req = new ROSLIB.ServiceRequest({
      calibrated_state:msg
    })
    this.isCalibrated_srv.callService(req, function(rsp){cb(rsp);});
  }

  this.calibrated_state = {
    uncalibrated : 0,
    calibrating  : 1,
    calibrated   : 2
  };
  //Draw function for svg 

  //------------------------------------------------------------------------//
  //                          PRIVATE IMPLEMENTATION                        //
  //------------------------------------------------------------------------//

  //Action Servers
  this.calibrate_client = new ROSLIB.ActionClient({
    ros: ros,
    serverName: "/hekateros_control/calibrate_as",
    actionName: "hekateros_control/CalibrateAction"
  });

  //Topics
  this.robot_status = new ROSLIB.Topic({
    ros: ros,
    name: "/hekateros_control/robot_status",
    messageType: "industrial_msgs/RobotStatus",
    throttle_rate : this.throttle_rate 
  });

  this.calibrate_feedback = new ROSLIB.Topic({
    ros: ros,
    name: "/hekateros_control/calibrate_as/feedback",
    messageType: "hekateros_control/CalibrateActionFeedback"
  });

  this.robot_status_EX = new ROSLIB.Topic({
    ros:ros,
    name: "/hekateros_control/robot_status_ex",
    messageType: "hekateros_control/HekRobotStatusExtended"
  });
  
  this.joint_states_EX = new ROSLIB.Topic({
    ros:ros,
    name: "/hekateros_control/joint_states_ex",
    messageType: "hekateros_control/HekJointStateExtended"
  });

  //Services
  this.EStop_srv = new ROSLIB.Service({
    ros: ros,
    name: "/hekateros_control/estop",
    messageType: "/hekateros/Estop"
  });

  this.resetEStop_srv = new ROSLIB.Service({
    ros: ros,
    name: "/hekateros_control/reset_estop",
    messageType: "/hekateros/ResetEStop"
  });
  
  this.release_srv = new ROSLIB.Service({
    ros: ros,
    name: "/hekateros_control/release",
    messageType:"Release"
  });

  this.clearAlarms_srv = new ROSLIB.Service({
    ros: ros,
    name: "/hekateros_control/clear_alarms",
    messageType: "/hekateros/ClearAlarms"
  });
  
  this.closeGripper_srv = new ROSLIB.Service({
    ros: ros,
    name: "/hekateros_control/close_gripper",
    messageType: "/hekateros/CloseGripper"
  });

  this.freeze_srv = new ROSLIB.Service({
    ros: ros,
    name: "/hekateros_control/freeze",
    messageType: "/hekateros/Freeze"
  });

  this.stop_srv = new ROSLIB.Service({
    ros: ros,
    name: "/hekateros_control/stop",
    messageType: "/hekateros/Stop"
  });

  this.setRobotMode_srv = new ROSLIB.Service({
    ros: ros,
    name: "/hekateros_control/set_robot_mode",
    messageType: "/hekateros/SetRobotMode"
  });
  
  this.openGripper_srv = new ROSLIB.Service({
    ros: ros,
    name: "/hekateros_control/open_gripper",
    messageType: "/hekateros/OpenGripper"
  });

  this.isCalibrated_srv = new ROSLIB.Service({
    ros: ros,
    name: "/hekateros_control/is_calibrated",
    messageType: "/hekateros/IsCalibrated"
  });

  this.isDescLoaded_srv = new ROSLIB.Service({
    ros: ros,
    name: "/hekateros_control/is_description_loaded",
    messageType: "/hekateros/IsDescLoaded"
  });
  
  this.isAlarmed_srv = new ROSLIB.Service({
    ros: ros,
    name: "/hekateros_control/is_alarmed",
    messageType: "/hekateros/IsAlarmed"
  });

  this.goToZeroPt_srv = new ROSLIB.Service({
    ros: ros,
    name: "/hekateros_control/goto_zero",
    messageType: "/hekateros/GotoZeroPt"
  });

  this.goToParkedPos_srv = new ROSLIB.Service({
    ros: ros,
    name: "/hekateros_control/goto_parked",
    messageType: "/hekateros/GotoParkedPos"
  });

  this.goToBalancedPos_srv = new ROSLIB.Service({
    ros: ros,
    name: "/hekateros_control/goto_balanced",
    messageType: "/hekateros/GotoBalancedPos"
  });

  this.getProductInfo_srv = new ROSLIB.Service({
    ros: ros,
    name: "/hekateros_control/get_product_info",
    messageType: "/hekateros/GetProductInfo"
  });
}
hek = new hekateros(50);
