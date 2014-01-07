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
}

hek = new hekateros(50);
