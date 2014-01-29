// 
// Global rosbridge connection.
// 
// required js imports:
//    http://cdn.robotwebtools.org/EventEmitter2/current/eventemitter2.min.js
//    http://cdn.robotwebtools.org/roslibjs/current/roslib.min.js
//

// rosbridge websocket URI
var rosbridge_uri = "ws://localhost:9090"

// create a rosbridge websocket connection
var ros = new ROSLIB.Ros({
  url:rosbridge_uri
});

