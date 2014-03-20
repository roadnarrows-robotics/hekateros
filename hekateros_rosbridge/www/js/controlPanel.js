window.setInterval(function ()
          {
            //updateData();
          },1000)

var calibrate_action_goal;
var robotState;
var jointState;
var i=0;
var is_calibrated = 0; //FALSE 

function buttons_panel(id, name, side, color)
{
  d3.select("#" + side + "Column").append("button")
      .attr("id", id)
      .attr("class", side)
      .style("background-color", color)
      .style("background-image", "url('img/" + id + ".png')" )
      //MIW Change icon path here
      .style("text-align", "right")
      //.style("text-shadow", "-1px 0 #ccc, 0 1px #ccc, 1px 0 #ccc, 0 -1px #ccc")
      .text(name)
      .attr("onclick", id + "()");

}


function create_panel() {
d3.select("#leftColumn").append("imageleft")
    .style("background-image", "url(img/RoadNarrows.png)");

d3.select("#rightColumn").append("imageright")
    .style("background-image", "url(img/hekateros.png)");

buttons_panel("Calibrate", "Calibrate", "left", "#ccc");
buttons_panel("Park", "Park", "left", "#ccc");
buttons_panel("Balance", "Balance", "left", "#ccc");
buttons_panel("ZeroPoint", "Zero Point", "left", "#ccc");
buttons_panel("OpenGripper", "Open Gripper", "left", "#ccc");
buttons_panel("CloseGripper", "Close Gripper", "left", "#ccc");
buttons_panel("SpecifyMove", "Specify Move", "left", "#ccc");
buttons_panel("EStop", "EStop", "right", "#900");
buttons_panel("Freeze", "Freeze", "right", "#ccc");
buttons_panel("Release", "Release", "right", "#ccc");
buttons_panel("ClearAlarms", "Clear Alarms", "right", "#ccc");
buttons_panel("Settings", "Settings", "right", "#ccc");
buttons_panel("About", "About", "right", "#ccc");

d3.select("#centerColumn").append("hekateros")
    .text("Hekateros Control Panel");
d3.select("#centerColumn").append("title")
d3.select("#centerColumn").append("main")
    .text("Joint State");
d3.select("#centerColumn").append("box")
    .text("NA");

//Initilizing bottom status bar
d3.select("box").text("Pan-Tilt interface initialized.");

first  = d3.select("title").append("sbar")
    .text("Mode:");

second = d3.select("title").append("sbar")
    .text("State:");

third  = d3.select("title").append("sbar")
    .style("background-image", "url('img/dark.png')")
    .text("Motors");

fourth = d3.select("title").append("sbar")
    .style("background-image", "url('img/dark.png')")
    .text("Moving");

fifth  = d3.select("title").append("sbar")
    .style("background-image", "url('img/dark.png')")
    .text("Alarms");

sixth  = d3.select("title").append("sbar")
    .style("background-image", "url('img/dark.png')")
    .text("EStop");
}


function EStop()
{
  console.info("EStop");
 var r=confirm("Warning: All motors will become undriven. The arm may fall and be damaged.");
    if (r==false)
    { 
      d3.select("#resetEStop")
          .text("EStop")
          .style("background-color","#900")
          .attr("id","EStop")
          .attr("onclick","EStop()");
    }
    else
    {
      hek.EStop();
      d3.select("box").text("Pan-Tilt emergency stopped.");
      d3.select("#EStop")
          .text("Reset EStop")
          .style("background-color", "#090")
          .attr("id","resetEStop")
          .attr("onclick","resetEStop()");
    }
} 

function resetEStop()
{
  console.info("EStop Reset");
  d3.select("box").text("Pan-Tilt emergency stop has been reset.");
  d3.select("#resetEStop")
      .text("EStop")
      .style("background-color","#900")
      .attr("id","EStop")
      .attr("onclick","EStop()");
  hek.resetEStop();
}

function Freeze()
{
  console.info("Freeze");
  d3.select("box").text("Pan-Tilt has been frozen at current position.");
  hek.freeze();
}

function Release()
{
  console.info("Release");
  var r=confirm("Warning: All motors will become undriven. The arm may fall and become damaged.");
  if (r==false)
  {
    console.info("Did not release arm")
  }
  else
  {
    d3.select("box").text("Pan-Tilt has been released, all motors are unpowered.");
    hek.release();
  }
}

function Calibrate()
{
  console.info("Calibrate");
  if(document.getElementById('calibrate').checked == true )
  {
    var r=confirm("Warning: the arm should be placed near the Zero point and the workspace cleared of obstructions");
  }
  if (r==false)
  {
    d3.select("#cancelCalibrate")
        .text("Calibrate")
        .style("background-color","#ccc")
        .style("background-image", "url('img/Calibrate.png')")
        .attr("id","Calibrate")
        .attr("onclick","Calibrate()");
  }
  else
  {
    calibrate_action_goal = hek.calibrate();
    calibrate_action_goal.send();
    d3.select("#Calibrate")
        .text("Cancel Calibrate")
        .style("background-color", "#900")
        .style("background-image", "url('img/cancelCalibrate.png')")
        .attr("id","cancelCalibrate")
        .attr("onclick","cancelCalibrate()");
    hek.sub_feedback(function(message) {
      console.info(message);
      d3.select("box").text("Calibrating complete.");
    is_calibrated = 0; //TRUE
    }) 
  }  
}

function cancelCalibrate()
{
  var r=confirm("Warning: The arm MUST be calibrated before operation. Calibration will have to be restarted to use the arm");
  if (r==false)
  {
    calibrate_action_goal = hek.calibrate();
    calibrate_action_goal.send();
    d3.select("#Calibrate")
        .text("Cancel Calibrate")
        .style("background-color", "#900")
        .style("background-image", "url('img/cancelCalibrate.png')")
        .attr("id","cancelCalibrate")
        .attr("onclick","cancelCalibrate()"); 
  }
  else
  {
    console.info("STOP calibration")
    calibrate_action_goal.cancel();
    d3.select("#cancelCalibrate")
        .text("Calibrate")
        .style("background-color","#ccc")
        .style("background-image", "url('img/Calibrate.png')")
        .attr("id","Calibrate")
        .attr("onclick","Calibrate()");
    d3.select("box").text("Calibrating canceled.");
    is_calibrated = 1; //FALSE
  }
}

function Park()
{
  console.info("Park");
  d3.select("box").text("Hekateros is in the parked position.");
  hek.park();
}

function Balance()
{
  console.info("Balanced");
  d3.select("box").text("Hekateros is in the balanced position.");
  hek.balanced();
}

function ZeroPoint()
{
  console.info("Zero Point");
  hek.zeroPoint();
  d3.select("box").text("Zero Point");
}

function OpenGripper()
{
  console.info("Open Gripper");
  d3.select("box").text("Hekateros gripper is open.");
  hek.openGripper();
}

function CloseGripper()
{
  console.info("Close Gripper");
  d3.select("box").text("Hekateros gripper is closed.");
  hek.closeGripper();
}

function About()
{
  console.info("about hekateros");
  var productInformation = hek.ProductInfo();
  alert(productInformation);
}

function ClearAlarms()
{
  console.info("Clear Alarms");
  d3.select("box").text("Hekateros alarms cleared.");
  var r=confirm("Note: Only over-loading conditions may be cleared. Temperature and volatge cannot be cleared by this action, the arm must be powered off.");
  if (r==false)
  {
  }
  else
  {
  }
}

function Settings()
{
  console.info("Settings");
  settingOverlay();
}

function settingOverlay()
{
  el = document.getElementById("settingOverlay");
  el.style.visibility = (el.style.visibility == "visible")? "hidden":"visible";
  var foo = d3.select("#settingOverlay").selectAll("p")
  foo.html(
        "<form><input type=checkbox id=calibrate oncahnge=save()/>show warning dialog before calibrating hekateros.</form>" 
      + "<form><input type=checkbox id=recalibrate checked/>Force (re)calibration for all joints on calibrate action.</form>" 
      + "<form><input type=checkbox id=release checked/>Show warning dialog before releasing hekateros.</form>")
  foo.append("savebutton")
      .text("Cancel")
      .attr("onclick","settingOverlay()");
  foo.append("savebutton")
      .text("OK")
      .attr("onclick","save()");
}

function save()
{
  console.info(document.getElementById.calibrate)
  console.info("Save");
  //document.getElementById('calibrate').unchecked
  d3.select("calibrate").property("checked", "true");
  settingOverlay();
}

function About()
{
  /*servoType   = robInfo.i.desc;
  productName = robInfo.i.product_name;
  productID   = robInfo.i.product_id;
  HWVersion   = robInfo.i.version_string;
  AppVersion  = robInfo.i.version_string;*/
  servoType   = "hello"
  productName = "hello"
  productID   = "hello"
  HWVersion   = "hello"
  AppVersion  = "hello"
  URL         = "www.roadnarrows.com/";
  Email       = "support@roadnarrows.com";
  Tel         = "+1.800.275.9568";

  aboutOverlay();
}

function aboutOverlay()
{
  el = document.getElementById("overlay");
  el.style.visibility = (el.style.visibility == "visible")? "hidden":"visible";
  d3.select("#overlay").selectAll("p").style("background-image", "url('img/HekaterosLogo.png')").style("background-repeat", "no-repeat").style("background-position-x", "-15px").html( 
            "<h2>" + servoType + "</h2><br/>" + 
            "Product:     " + "&nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp;                                &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp;                                &nbsp; &nbsp; &nbsp; &nbsp;" + productName                   + "<br/>" + 
            "Product Id:  " + "&nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp;                                &nbsp; &nbsp; &nbsp; &nbsp; &nbsp;                                              000" + productID               
            + "<br/>" +
            "HW Version:  " + "&nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp;                                &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp;"                               + HWVersion   
            + "<br/>" +
            "App Version: " + "&nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp;                                &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp;"                               + AppVersion  
            
            + "<br/>" +
            "URL:         " + "&nbsp; &nbsp; &nbsp; &nbsp; &nbsp;"+ URL                     
            + "<br/>" +
            "Email:       " + "&nbsp; &nbsp;" + Email       
            + "<br/>" +
            
            "Tel:         " + "&nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp;                                &nbsp; &nbsp; &nbsp; &nbsp; &nbsp;"+ Tel                     + "<br/> </br/> <br/> <br/> <br/>" +

            "TODO JNT -- Write Descrption");
}

//GLOBAL table variables 
var columns;
var tbl;
var table;
var thead;
var tbody;
var th;
var rows;
var cells;
var data;

function mainTable() {      
  columns = ['joint', 'ServoID', 'State', 'Position', 'Odometer', 'Encoder', 'Velocity', 'Speed', 'Effort', 'Temperature', 'Voltage', 'Alarms'];
table = d3.select("main").append('table');
 data =[
        {"joint"      : "base_rot:", 
         "ServoID"    : "NA", 
         "State"      : "NA", 
         "Position"   : "NA", 
         "Odometer"   : "NA", 
         "Encoder"    : "NA", 
         "Velocity"   : "NA", 
         "Speed"      : "NA", 
         "Effort"     : "NA", 
         "Temperature": "NA", 
         "Voltage"    : "NA", 
         "Alarms"     : "NA"  },

        {"joint"      : "shoulder:", 
         "ServoID"    : "NA", 
         "State"      : "NA", 
         "Position"   : "NA", 
         "Odometer"   : "NA", 
         "Encoder"    : "NA", 
         "Velocity"   : "NA", 
         "Speed"      : "NA", 
         "Effort"     : "NA", 
         "Temperature": "NA", 
         "Voltage"    : "NA", 
         "Alarms"     : "NA"  },
       
        {"joint"      : "elbow:", 
         "ServoID"    : "NA", 
         "State"      : "NA", 
         "Position"   : "NA", 
         "Odometer"   : "NA", 
         "Encoder"    : "NA", 
         "Velocity"   : "NA", 
         "Speed"      : "NA", 
         "Effort"     : "NA", 
         "Temperature": "NA", 
         "Voltage"    : "NA", 
         "Alarms"     : "NA"  },

        {"joint"      : "wrist_pitch:", 
         "ServoID"    : "NA", 
         "State"      : "NA", 
         "Position"   : "NA", 
         "Odometer"   : "NA", 
         "Encoder"    : "NA", 
         "Velocity"   : "NA", 
         "Speed"      : "NA", 
         "Effort"     : "NA", 
         "Temperature": "NA", 
         "Voltage"    : "NA", 
         "Alarms"     : "NA"  },
       
        {"joint"      : "wrist_rot:", 
         "ServoID"    : "NA", 
         "State"      : "NA", 
         "Position"   : "NA", 
         "Odometer"   : "NA", 
         "Encoder"    : "NA", 
         "Velocity"   : "NA", 
         "Speed"      : "NA", 
         "Effort"     : "NA", 
         "Temperature": "NA", 
         "Voltage"    : "NA", 
         "Alarms"     : "NA"  },

        {"joint"      : "grip:", 
         "ServoID"    : "NA", 
         "State"      : "NA", 
         "Position"   : "NA", 
         "Odometer"   : "NA", 
         "Encoder"    : "NA", 
         "Velocity"   : "NA", 
         "Speed"      : "NA", 
         "Effort"     : "NA", 
         "Temperature": "NA", 
         "Voltage"    : "NA", 
         "Alarms"     : "NA"  }
        ];
d3.select("table").call(t);

}

d3.table = function(config) {

  tbl = function(selection) {
    if(columns.length == 0)
    { 
      columns = d3.keys(selection.data()[0][0]);
      console.info(columns);
    }

  table.selectAll('thead').data([0]).enter().append('thead');
  thead = table.select('thead');

  table.selectAll('tbody').data([0]).enter().append('tbody');
  tbody = table.select('tbody');

  th = thead.append("tr").selectAll("th")
     .data(columns)

  th.enter().append("th");
  th.text(function(d) { return d;})
  th.exit().remove();

 

  rows = tbody.selectAll("tr")
      .data(data) 

  rows.enter().append("tr");
  rows.exit().remove();

  cells = rows.selectAll("td")
      .data(function(row) 
           {
             return columns.map(function(key) 
                               {
                                 return {key:key, value:row[key]};
                               });
           })
  cells.enter().append("td");
  cells.text(function(d) { return d.value; })
      .attr('data-col', function(d,i){ return i})
      .attr('data-key', function(d,i){(d.key); return d.key});
  cells.exit().remove();
  return tbl;
  };

  tbl.columns = function(_) {
    if(!arguments.length) { return columns;}
    columns = _;
    return this;
  };

  return tbl;
};

var t= d3.table();

 

function updateData() {
  data[0]["ServoID"]    = robotState.servo_health[0].servo_id;
  data[0]["Temperature"]= robotState.servo_health[0].temp;
  data[0]["Voltage"]    = robotState.servo_health[0].voltage.toFixed(2);
  data[0]["Effort"]     = jointState.effort[0];
  data[0]["Speed"]      = jointState.raw_speed[0];
  data[0]["Odometer"]   = jointState.odometer_pos[0];
  data[0]["Encoder"]    = jointState.encoder_pos[0];
  data[0]["Position"]   = jointState.position[0].toFixed(2);
  data[0]["Velocity"]   = jointState.velocity[0].toFixed(2);


  data[1]["ServoID"]    = robotState.servo_health[1].servo_id;
  data[1]["Temperature"]= robotState.servo_health[1].temp;
  data[1]["Voltage"]    = robotState.servo_health[1].voltage.toFixed(2);
  data[1]["Effort"]     = jointState.effort[1];
  data[1]["Speed"]      = jointState.raw_speed[1];
  data[1]["Odometer"]   = jointState.odometer_pos[1];
  data[1]["Encoder"]    = jointState.encoder_pos[1];
  data[1]["Position"]   = jointState.position[1].toFixed(2);
  data[1]["Velocity"]   = jointState.velocity[1].toFixed(2);
  
  
  data[2]["ServoID"]    = robotState.servo_health[3].servo_id;
  data[2]["Temperature"]= robotState.servo_health[3].temp;
  data[2]["Voltage"]    = robotState.servo_health[3].voltage.toFixed(2);
  data[2]["Effort"]     = jointState.effort[2];
  data[2]["Speed"]      = jointState.raw_speed[2];
  data[2]["Odometer"]   = jointState.odometer_pos[2];
  data[2]["Encoder"]    = jointState.encoder_pos[2];
  data[2]["Position"]   = jointState.position[2].toFixed(2);
  data[2]["Velocity"]   = jointState.velocity[2].toFixed(2);
  
  
  data[3]["ServoID"]    = robotState.servo_health[4].servo_id;
  data[3]["Temperature"]= robotState.servo_health[4].temp;
  data[3]["Voltage"]    = robotState.servo_health[4].voltage.toFixed(2);
  data[3]["Effort"]     = jointState.effort[3];
  data[3]["Speed"]      = jointState.raw_speed[3];
  data[3]["Odometer"]   = jointState.odometer_pos[3];
  data[3]["Encoder"]    = jointState.encoder_pos[3];
  data[3]["Position"]   = jointState.position[3].toFixed(2);
  data[3]["Velocity"]   = jointState.velocity[3].toFixed(2);
  
  
  data[4]["ServoID"]    = robotState.servo_health[5].servo_id;
  data[4]["Temperature"]= robotState.servo_health[5].temp;
  data[4]["Voltage"]    = robotState.servo_health[5].voltage.toFixed(2);
  data[4]["Effort"]     = jointState.effort[4];
  data[4]["Speed"]      = jointState.raw_speed[4];
  data[4]["Odometer"]   = jointState.odometer_pos[4];
  data[4]["Encoder"]    = jointState.encoder_pos[4];
  data[4]["Position"]   = jointState.position[4].toFixed(2);
  data[4]["Velocity"]   = jointState.velocity[4].toFixed(2);
  
  
  data[5]["ServoID"]    = robotState.servo_health[6].servo_id;
  data[5]["Temperature"]= robotState.servo_health[6].temp;
  data[5]["Voltage"]    = robotState.servo_health[6].voltage.toFixed(2);
  data[5]["Effort"]     = jointState.effort[5];
  data[5]["Speed"]      = jointState.raw_speed[5];
  data[5]["Odometer"]   = jointState.odometer_pos[5];
  data[5]["Encoder"]    = jointState.encoder_pos[5];
  data[5]["Position"]   = jointState.position[5].toFixed(2);
  data[5]["Velocity"]   = jointState.velocity[5].toFixed(2);

            
  var i;  
  for(i = 0; i<6; i++)  
  {        
    if(jointState.op_state[i] == 0)
    {
      data[i]["State"]="uncalibrated";
    }
    else if(jointState.op_state[i] == 1)
    {
      data[i]["State"]="calibrating";
    }          
    else
    {
      data[i]["State"]="calibrated";
    }
  }

  if(jointState.op_state[0]==1 && jointState.op_state[1]==1 && jointState.op_state[2]==1 && jointState.op_state[3]==1 && jointState.op_state[4]==1 && jointState.op_state[5]==1 )
  {
    is_calibrated = 0; //TRUE
    console.info("in loop")
    //set RobotStatus to calibrated
    d3.select("#cancelCalibrate")
      .text("Calibrate")
      .style("background-color","#ccc")
      .style("background-image", "url('img/Calibrate.png')")
      .attr("id","Calibrate")
      .attr("onclick","Calibrate()");
  }

  
  //NEEDS TO BE TESTED -- BUTTONS SHOULD BE FADED OVER IF NOT CALIBRATED
  /*if (is_calibrated = 1)
  {
    d3.select("#Park").attr("background-color", "rgba(0,0,0,0.5").attr("onClick",null);
    d3.select("#Balance").attr("background-color", "rgba(0,0,0,0.5").attr("onClick",null);
    d3.select("#ZeroPoint").attr("background-color", "rgba(0,0,0,0.5").attr("onClick",null);
    d3.select("#OpenGripper").attr("background-color", "rgba(0,0,0,0.5").attr("onClick",null);
    d3.select("#CloseGripper").attr("background-color", "rgba(0,0,0,0.5").attr("onClick",null);
    d3.select("#SpecifyMove").attr("background-color", "rgba(0,0,0,0.5").attr("onClick",null);
  }
  else
  {
    d3.select("#Park").attr("background-color", "#ccc").attr("onClick",'Park()");
    d3.select("#Balance").attr("background-color", "#ccc").attr("onClick","Balance()");
    d3.select("#ZeroPoint").attr("background-color", "#ccc").attr("onClick","ZeroPoint());
    d3.select("#OpenGripper").attr("background-color", "#ccc").attr("onClick","OpenGripper()");
    d3.select("#CloseGripper").attr("background-color", "#ccc").attr("onClick","CloseGripper()");
    d3.select("#SpecifyMove").attr("background-color", "#ccc").attr("onClick","SpecifyMove()");
  }*/

  rows.data(data)

  cells.data(function(row) 
            {
              return columns.map(function(key) 
                                {
                                  return {key:key, value:row[key]};
                                });
            })
      .text(function(d) { return d.value; });

  
};
       



