////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics Hekateros Robotic Manipulator ROS Package
//
// Link:      https://github.com/roadnarrows-robotics/hekateros
//
// ROS Node:  hek_teleop
//
// File:      hek_teleop.cpp
//
/*! \file
 *
 * $LastChangedDate$
 * $Rev$
 *
 * \brief The ROS hek_teleop node class implementation.
 *
 * \author Daniel Packard (daniel@roadnarrows.com)
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \par Copyright:
 * (C) 2013-2014  RoadNarrows
 * (http://www.roadnarrows.com)
 * \n All Rights Reserved
 */
/*
 * @EulaBegin@
 * 
 * Permission is hereby granted, without written agreement and without
 * license or royalty fees, to use, copy, modify, and distribute this
 * software and its documentation for any purpose, provided that
 * (1) The above copyright notice and the following two paragraphs
 * appear in all copies of the source code and (2) redistributions
 * including binaries reproduces these notices in the supporting
 * documentation.   Substantial modifications to this software may be
 * copyrighted by their authors and need not follow the licensing terms
 * described here, provided that the new terms are clearly indicated in
 * all files where they apply.
 * 
 * IN NO EVENT SHALL THE AUTHOR, ROADNARROWS LLC, OR ANY MEMBERS/EMPLOYEES
 * OF ROADNARROW LLC OR DISTRIBUTORS OF THIS SOFTWARE BE LIABLE TO ANY
 * PARTY FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
 * DAMAGES ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION,
 * EVEN IF THE AUTHORS OR ANY OF THE ABOVE PARTIES HAVE BEEN ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 * 
 * THE AUTHOR AND ROADNARROWS LLC SPECIFICALLY DISCLAIM ANY WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
 * FITNESS FOR A PARTICULAR PURPOSE. THE SOFTWARE PROVIDED HEREUNDER IS ON AN
 * "AS IS" BASIS, AND THE AUTHORS AND DISTRIBUTORS HAVE NO OBLIGATION TO
 * PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.
 * 
 * @EulaEnd@
 */
////////////////////////////////////////////////////////////////////////////////

//
// System
//
#include <sys/types.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <string>
#include <map>

//
// Boost
//
#include <boost/bind.hpp>
#include "boost/assign.hpp"

//
// ROS
//
#include "ros/ros.h"
#include "actionlib/server/simple_action_server.h"

//
// ROS generated core, industrial, and hekateros messages.
//
#include "trajectory_msgs/JointTrajectory.h"
#include "sensor_msgs/JointState.h"
#include "industrial_msgs/RobotStatus.h"
#include "hekateros_control/HekJointStateExtended.h"
#include "hekateros_control/HekRobotStatusExtended.h"

//
// ROS generatated hekateros services.
//
#include "hekateros_control/Calibrate.h"
#include "hekateros_control/ClearAlarms.h"
#include "hekateros_control/CloseGripper.h"
#include "hekateros_control/EStop.h"
#include "hekateros_control/Freeze.h"
#include "hekateros_control/GetProductInfo.h"
#include "hekateros_control/GotoBalancedPos.h"
#include "hekateros_control/GotoParkedPos.h"
#include "hekateros_control/GotoZeroPt.h"
#include "hekateros_control/IsAlarmed.h"
#include "hekateros_control/IsCalibrated.h"
#include "hekateros_control/IsDescLoaded.h"
#include "hekateros_control/OpenGripper.h"
#include "hekateros_control/Release.h"
#include "hekateros_control/ResetEStop.h"
#include "hekateros_control/SetRobotMode.h"
#include "hekateros_control/Stop.h"

//
// ROS generated action servers.
//

//
// ROS generated HID messages.
//
#include "hid/ConnStatus.h"           // subscribe
#include "hid/Controller360State.h"   // subscribe
#include "hid/LEDPattern.h"           // service
#include "hid/RumbleCmd.h"            // publish

//
// ROS generatated HID services.
//
#include "hid/SetLED.h"
#include "hid/SetRumble.h"

//
// RoadNarrows
//
#include "rnr/rnrconfig.h"
#include "rnr/log.h"
#include "rnr/hid/HIDXbox360.h"

//
// RoadNarrows embedded hekateros library.
//
#include "Hekateros/hekateros.h"
#include "Hekateros/hekXmlCfg.h"
#include "Hekateros/hekRobot.h"
#include "Hekateros/hekUtils.h"

//
// Node headers.
//
#include "hek_teleop.h"

using namespace std;
using namespace boost::assign;
using namespace hid;
using namespace hekateros;
using namespace trajectory_msgs;
using namespace industrial_msgs;
using namespace hekateros_control;


//------------------------------------------------------------------------------
// HekTeleop Class
//------------------------------------------------------------------------------

HekTeleop::HekTeleop(ros::NodeHandle &nh, double hz) : m_nh(nh), m_hz(hz)
{
  m_eState            = TeleopStateUninit;
  m_eMode             = TeleopModeFirstPerson;
  m_bHasXboxComm      = false;
  m_nWdXboxCounter    = 0;
  m_nWdXboxTimeout    = countsPerSecond(3.0);
  m_bRcvdRobotStatus  = false;
  m_bRcvdJointState   = false;
  m_bHasRobotComm     = false;
  m_nWdRobotCounter   = 0;
  m_nWdRobotTimeout   = countsPerSecond(5.0);
  m_bHasFullComm      = false;

  m_buttonState = map_list_of
      (ButtonIdGotoBalPos,    0)
      (ButtonIdEStop,         0)
      (ButtonIdGotoParkedPos, 0)
      (ButtonIdGotoZeroPt,    0)

      (ButtonIdPause,         0)
      (ButtonIdToggleMode,    0)
      (ButtonIdStart,         0)

      (ButtonIdPrevJoint,     0)
      (ButtonIdNextJoint,     0)

      (ButtonIdMoveJoints,    0)
      (ButtonIdRotBase,       0)
      (ButtonIdPitchWrist,    0)

      (ButtonIdRotWristCw,    0)
      (ButtonIdRotWristCcw,   0)

      (ButtonIdOpenGripper,   0)
      (ButtonIdCloseGripper,  0);

  m_rumbleLeft    = 0;
  m_rumbleRight   = 0;

  m_fpState.m_bNewGoal        = true;
  m_fpState.m_goalSign        = 1.0;
  m_fpState.m_goalJoint.alpha = 0.0;
  m_fpState.m_goalJoint.beta  = 0.0;
  m_fpState.m_goalJoint.gamma = 0.0;
  m_fpState.m_goalCart.x      = 0.0;
  m_fpState.m_goalCart.y      = 0.0;
}

HekTeleop::~HekTeleop()
{
}


//. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 
// Server Services
//. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 


//. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 
// Client Services
//. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 

void HekTeleop::clientServices()
{
  string  strSvc;

  strSvc = "/xbox_360/set_led";
  m_clientServices[strSvc] = m_nh.serviceClient<hid::SetLED>(strSvc);

  strSvc = "/xbox_360/set_rumble";
  m_clientServices[strSvc] = m_nh.serviceClient<hid::SetRumble>(strSvc);

  strSvc = "/hekateros_control/estop";
  m_clientServices[strSvc] =
      m_nh.serviceClient<hekateros_control::EStop>(strSvc);

  strSvc = "/hekateros_control/freeze";
  m_clientServices[strSvc] =
      m_nh.serviceClient<hekateros_control::Freeze>(strSvc);

  strSvc = "/hekateros_control/goto_balanced";
  m_clientServices[strSvc] =
      m_nh.serviceClient<hekateros_control::GotoBalancedPos>(strSvc);

  strSvc = "/hekateros_control/goto_parked";
  m_clientServices[strSvc] =
      m_nh.serviceClient<hekateros_control::GotoParkedPos>(strSvc);

  strSvc = "/hekateros_control/goto_zero";
  m_clientServices[strSvc] =
      m_nh.serviceClient<hekateros_control::GotoZeroPt>(strSvc);

  strSvc = "/hekateros_control/reset_estop";
  m_clientServices[strSvc] =
      m_nh.serviceClient<hekateros_control::ResetEStop>(strSvc);

  strSvc = "/hekateros_control/set_robot_mode";
  m_clientServices[strSvc] =
      m_nh.serviceClient<hekateros_control::SetRobotMode>(strSvc);

}

void HekTeleop::setLED(int pattern)
{
  hid::SetLED svc;

  svc.request.led_pattern.val = pattern;

  if( m_clientServices["/xbox_360/set_led"].call(svc) )
  {
    ROS_DEBUG("Xbox360 LED set to pattern to %d.", pattern);
  }
  else
  {
    ROS_ERROR("Failed to set Xbox360 LED.");
  }
}

void HekTeleop::setRumble(int motorLeft, int motorRight)
{
  hid::SetRumble svc;

  // no change
  if( (motorLeft == m_rumbleLeft) && (motorRight == m_rumbleRight) )
  {
    return;
  }

  svc.request.left_rumble  = motorLeft;
  svc.request.right_rumble = motorRight;

  if( m_clientServices["/xbox_360/set_rumble"].call(svc) )
  {
    ROS_DEBUG("Xbox360 rumble motors set to %d, %d.", motorLeft, motorRight);
    m_rumbleLeft  = motorLeft;
    m_rumbleRight = motorRight;
  }
  else
  {
    ROS_ERROR("Failed to set Xbox360 rumble motors.");
  }
}

void HekTeleop::estop()
{
  hekateros_control::EStop svc;

  if( m_clientServices["/hekateros_control/estop"].call(svc) )
  {
    ROS_INFO("Robot emergency stopped.");
  }
  else
  {
    ROS_ERROR("Failed to estop robot.");
  }
}

void HekTeleop::freeze()
{
  hekateros_control::Freeze svc;

  if( m_clientServices["/hekateros_control/freeze"].call(svc) )
  {
    ROS_INFO("Robot stopped.");
  }
  else
  {
    ROS_ERROR("Failed to freeze robot.");
  }
}

void HekTeleop::gotoBalancedPos()
{
  hekateros_control::GotoBalancedPos svc;

  if( m_clientServices["/hekateros_control/goto_balanced"].call(svc) )
  {
    ROS_INFO("Moving to balanced position.");
  }
  else
  {
    ROS_ERROR("Failed to move.");
  }
}

void HekTeleop::gotoParkedPos()
{
  hekateros_control::GotoParkedPos svc;

  if( m_clientServices["/hekateros_control/goto_parked"].call(svc) )
  {
    ROS_INFO("Moving to parked position.");
  }
  else
  {
    ROS_ERROR("Failed to move.");
  }
}

void HekTeleop::gotoZeroPt()
{
  hekateros_control::GotoZeroPt svc;

  if( m_clientServices["/hekateros_control/goto_zero"].call(svc) )
  {
    ROS_INFO("Moving to zero point.");
  }
  else
  {
    ROS_ERROR("Failed to move.");
  }
}

void HekTeleop::resetEStop()
{
  hekateros_control::ResetEStop svc;

  if( m_clientServices["/hekateros_control/reset_estop"].call(svc) )
  {
    ROS_INFO("Robot emergency stopped has been reset.");
  }
  else
  {
    ROS_ERROR("Failed to reset estop.");
  }
}

void HekTeleop::setRobotMode(HekRobotMode mode)
{
  hekateros_control::SetRobotMode svc;

  svc.request.mode.val = mode;

  if( m_clientServices["/hekateros_control/set_robot_mode"].call(svc) )
  {
    ROS_INFO("Robot mode set to %d.", mode);
  }
  else
  {
    ROS_ERROR("Failed to set robot mode.");
  }
}


//. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 
// Topic Publishers
//. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 

void HekTeleop::advertisePublishers(int nQueueDepth)
{
  string  strPub;

  strPub = "/hekateros_control/joint_command";
  m_publishers[strPub] =
    m_nh.advertise<trajectory_msgs::JointTrajectory>(strPub, nQueueDepth);

  strPub = "/xbox_360/rumble_command";
  m_publishers[strPub] =
    m_nh.advertise<hid::RumbleCmd>(strPub, nQueueDepth);
}

void HekTeleop::publishJointCmd()
{
  static bool bPubJointCmd = false;
  
  if( m_msgJointTraj.joint_names.size() > 0 )
  {
    m_msgJointTraj.header.stamp    = ros::Time::now();
    m_msgJointTraj.header.frame_id = "0";
    m_msgJointTraj.header.seq++;
    m_msgJointTraj.points.push_back(m_msgJointTrajPoint);

    // publish
    m_publishers["/hekateros_control/joint_command"].publish(m_msgJointTraj);
    bPubJointCmd = true;
  }
  else if( bPubJointCmd )
  {
    freeze();
    bPubJointCmd = false;
  }
}

void HekTeleop::publishRumbleCmd(int motorLeft, int motorRight)
{
  RumbleCmd msg;
  
  msg.left_rumble  = motorLeft;
  msg.right_rumble = motorRight;

  // publish
  m_publishers["/xbox_360/rumble_command"].publish(msg);
}


//. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 
// Subscribed Topics
//. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 

void HekTeleop::subscribeToTopics(int nQueueDepth)
{
  string  strSub;

  strSub = "/hekateros_control/robot_status_ex";
  m_subscriptions[strSub] = m_nh.subscribe(strSub, nQueueDepth,
                                          &HekTeleop::cbRobotStatus,
                                          &(*this));

  strSub = "/hekateros_control/joint_states_ex";
  m_subscriptions[strSub] = m_nh.subscribe(strSub, nQueueDepth,
                                          &HekTeleop::cbJointState,
                                          &(*this));

  strSub = "/xbox_360/conn_status";
  m_subscriptions[strSub] = m_nh.subscribe(strSub, nQueueDepth,
                                          &HekTeleop::cbXboxConnStatus,
                                          &(*this));

  strSub = "/xbox_360/controller_360_state";
  m_subscriptions[strSub] = m_nh.subscribe(strSub, nQueueDepth,
                                          &HekTeleop::cbXboxBttnState,
                                          &(*this));
}

void HekTeleop::cbRobotStatus(const HekRobotStatusExtended &msg)
{
  ROS_DEBUG("Received robot status.");

  // RDK might autoset pause/start here

  m_msgRobotStatus    = msg;
  m_bRcvdRobotStatus  = true;
  m_bHasRobotComm     = m_bRcvdJointState;
  m_nWdRobotCounter   = 0;
}

void HekTeleop::cbJointState(const HekJointStateExtended &msg)
{
  static float  MaxEffort = 1023; // effort is between [0, 1023] 
  static float  DeadZone  = 256;  // no haptic feedback in this zone

  ssize_t   i;
  int       rumbleRight;

  ROS_DEBUG("Received joint state.");

  m_nWdRobotCounter = 0;
  m_msgJointState   = msg;

  if( !m_bRcvdJointState )
  {
    for(i=0; i<msg.name.size(); ++i)
    {
      m_mapJoints[msg.name[i]] = -1;
    }

    if( i > 0 )
    {
      m_bRcvdJointState = true;
    }
  }

  //
  // Gripper tactile feedback.
  //
  if( (i = indexOfRobotJoint("grip")) >= 0 )
  {
    rumbleRight == ((float)msg.effort[i] - DeadZone)/(MaxEffort - DeadZone) *
                                                  XBOX360_RUMBLE_RIGHT_MAX;
    if( rumbleRight >= 0 )
    {
      setRumble(m_rumbleLeft, rumbleRight);
    }
  }
}

void HekTeleop::cbXboxConnStatus(const hid::ConnStatus &msg)
{
  ROS_DEBUG("Received Xbox360 connectivity status.");

  m_bHasXboxComm    = msg.is_connected && msg.is_linked;
  m_nWdXboxCounter  = 0;

  m_msgConnStatus = msg;
}

void HekTeleop::cbXboxBttnState(const hid::Controller360State &msg)
{
  ButtonState buttonState;

  ROS_DEBUG("Received Xbox360 button state.");

  if( m_bHasFullComm )
  {
    msgToState(msg, buttonState);

    switch( m_eState )
    {
      case TeleopStateReady:
        execAllButtonActions(buttonState);
        break;
      case TeleopStatePaused:
        buttonStart(buttonState);    // only active button in pause state
        break;
      case TeleopStateUninit:
      default:
        pause();
        break;
    }
  }

  m_buttonState = buttonState;
}


//. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 
// Sanity
//. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 

void HekTeleop::commCheck()
{
  if( m_bHasXboxComm )
  {
    if( ++m_nWdXboxCounter >= m_nWdXboxTimeout )
    {
      m_bHasXboxComm = false;
    }
  }

  if( m_bHasRobotComm )
  {
    if( ++m_nWdRobotCounter >= m_nWdRobotTimeout )
    {
      m_bHasRobotComm = false;
    }
  }

  bool hasComm  = m_bHasXboxComm && m_bHasRobotComm;

  // had communitcation, but no more
  if( m_bHasFullComm && !hasComm )
  {
    ROS_INFO("Lost communication with Xbox360 and/or Hek.");
    putRobotInSafeMode(m_msgConnStatus.is_connected);
  }

  m_bHasFullComm = hasComm;
  
  // not really a communication check function, but convenient.
  if( m_eState == TeleopStatePaused )
  {
    driveLEDsFigure8Pattern();
  }
}

void HekTeleop::putRobotInSafeMode(bool bHard)
{
  static float  fGovDft = 0.20;

  // stop robot
  freeze();

  // set robot mode
  setRobotMode(HekRobotModeAuto);
 
  if( bHard )
  {
    // nothing to do
  }
  
  m_eState = TeleopStateUninit;

  setLED(LEDPatOn);
}


//. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 
// Xbox Actions
//. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 

void HekTeleop::msgToState(const hid::Controller360State &msg,
                            ButtonState                   &buttonState)
{
  buttonState[ButtonIdGotoBalPos]     = msg.a_button;
  buttonState[ButtonIdEStop]          = msg.b_button;
  buttonState[ButtonIdGotoParkedPos]  = msg.x_button;
  buttonState[ButtonIdGotoZeroPt]     = msg.y_button;

  buttonState[ButtonIdPause]          = msg.back_button;
  buttonState[ButtonIdToggleMode]     = msg.center_button;
  buttonState[ButtonIdStart]          = msg.start_button;

  buttonState[ButtonIdPrevJoint]      = msg.dpad_left;
  buttonState[ButtonIdNextJoint]      = msg.dpad_right;

  buttonState[ButtonIdMoveJoints]     = msg.left_joy_y;
  buttonState[ButtonIdRotBase]        = msg.right_joy_x;
  buttonState[ButtonIdPitchWrist]     = msg.right_joy_y;

  buttonState[ButtonIdRotWristCw]     = msg.left_bump;
  buttonState[ButtonIdRotWristCcw]    = msg.right_bump;

  buttonState[ButtonIdOpenGripper]    = msg.left_trig;
  buttonState[ButtonIdCloseGripper]   = msg.right_trig;
}

void HekTeleop::execAllButtonActions(ButtonState &buttonState)
{
  clearJointTrajectory();

  // emergency stop
  buttonEStop(buttonState);

  //
  // Teleoperation state.
  //
  if( m_eState == TeleopStateReady )
  {
    buttonPause(buttonState);
  }
  else if( m_eState == TeleopStatePaused )
  {
    buttonStart(buttonState);
  }

  //
  // Teleoperation Modes.
  //
  buttonToggleMode(buttonState);
  
  if( m_eMode != TeleopModeFirstPerson )
  {
    buttonPrevJoint(buttonState);
    buttonNextJoint(buttonState);
  }
 
  //
  // Moves.
  //
  if( canMove() )
  {
    buttonGotoBalancedPos(buttonState);
    buttonGotoParkedPos(buttonState);
    buttonGotoZeroPt(buttonState);

    buttonCloseGripper(buttonState);
    buttonOpenGripper(buttonState);
    buttonMoveJoints(buttonState);

    publishJointCmd();
  }
}

void HekTeleop::buttonStart(ButtonState &buttonState)
{
  if( buttonOffToOn(ButtonIdStart, buttonState) )
  {
    ROS_INFO("Manual operation active, auto mode disabled.");

    setRobotMode(HekRobotModeManual);

    if( m_msgRobotStatus.e_stopped.val == TriState::TRUE )
    {
      resetEStop();
    }

    ready();
  }
}

void HekTeleop::buttonPause(ButtonState &buttonState)
{
  if( buttonOffToOn(ButtonIdPause, buttonState) )
  {
    ROS_INFO("Manual operation paused, auto mode enabled.");

    setRobotMode(HekRobotModeAuto);

    pause();
  }
}

void HekTeleop::buttonToggleMode(ButtonState &buttonState)
{
  if( buttonOffToOn(ButtonIdToggleMode, buttonState) )
  {
    if( m_eMode == TeleopModeFirstPerson )
    {
      m_eMode = TeleopModeShoulder;
      setLED(LEDPatShoulder);
      ROS_INFO("Move shoulder in isolation.");
    }
    else
    {
      m_eMode = TeleopModeFirstPerson;
      m_fpState.m_bNewGoal = true;
      setLED(LEDPatReady);
      ROS_INFO("First person mode.");
    }
  }
}

void HekTeleop::buttonPrevJoint(ButtonState &buttonState)
{
  if( buttonOffToOn(ButtonIdPrevJoint, buttonState) )
  {
    if( m_eMode == TeleopModeShoulder )
    {
      m_eMode = TeleopModeElbow;
      setLED(LEDPatElbow);
      ROS_INFO("Move elbow in isolation.");
    }
    else if( m_eMode == TeleopModeElbow )
    {
      m_eMode = TeleopModeShoulder;
      setLED(LEDPatShoulder);
      ROS_INFO("Move shoulder in isolation.");
    }
  }
}

void HekTeleop::buttonNextJoint(ButtonState &buttonState)
{
  if( buttonOffToOn(ButtonIdNextJoint, buttonState) )
  {
    if( m_eMode == TeleopModeShoulder )
    {
      m_eMode = TeleopModeElbow;
      setLED(LEDPatElbow);
      ROS_INFO("Move elbow in isolation.");
    }
    else if( m_eMode == TeleopModeElbow )
    {
      m_eMode = TeleopModeShoulder;
      setLED(LEDPatShoulder);
      ROS_INFO("Move shoulder in isolation.");
    }
  }
}

void HekTeleop::buttonEStop(ButtonState &buttonState)
{
  static int  clicks        = 0;            // number of button clicks
  static int  intvlCounter  = 0;            // intra-click interval counter
  static int  intvlTimeout  = countsPerSecond(0.3); // intra-click timeout

  //
  // Robot is estopped. This can come from a different node source. Make sure
  // counters are cleared.
  //
  if( m_msgRobotStatus.e_stopped.val == TriState::TRUE )
  {
    clicks = 0;
    intvlCounter = 0;
    return;
  }

  // button off to on
  if( buttonOffToOn(ButtonIdEStop, buttonState) )
  {
    ++clicks;
  }

  switch( clicks )
  {
    case 0:     // no click
      break;
    case 1:     // single click
      if( intvlCounter > intvlTimeout )
      {
        clicks = 0;
        intvlCounter = 0;
      }
      break;
    case 2:     // double click
      if( intvlCounter <= intvlTimeout )
      {
        estop();
        m_eState = TeleopStatePaused;
        setLED(LEDPatOn);       // see comment block in buttonPause()
        setLED(LEDPatPaused);
      }
      clicks = 0;
      intvlCounter = 0;
      break;
    default:    // multiple clicks
      clicks = 0;
      intvlCounter = 0;
      break;
  }
}

void HekTeleop::buttonCloseGripper(ButtonState &buttonState)
{
  static int deadzone = 10;

  int     trigger = buttonState[ButtonIdOpenGripper];
  int     i;
  double  vel;

  if( trigger < deadzone )
  {
    return;
  }

  // no joint found on arm
  if( (i = addJointToTrajectoryPoint("grip")) < 0 )
  {
    return;
  }

  vel = (double)(trigger-deadzone) / XBOX360_TRIGGER_MAX * 2.0;

  m_msgJointTrajPoint.positions[i] += degToRad(10.0);
  m_msgJointTrajPoint.velocities[i] = vel;
}

void HekTeleop::buttonOpenGripper(ButtonState &buttonState)
{
  static int deadzone = 10;

  int     trigger = buttonState[ButtonIdCloseGripper];
  int     i;
  double  vel;

  if( trigger < deadzone )
  {
    return;
  }

  // no joint found on arm
  if( (i = addJointToTrajectoryPoint("grip")) < 0 )
  {
    return;
  }

  vel = (double)(trigger-deadzone) / XBOX360_TRIGGER_MAX * 2.0;

  m_msgJointTrajPoint.positions[i] -= degToRad(10.0);
  m_msgJointTrajPoint.velocities[i] = vel;
}

void HekTeleop::buttonMoveJoints(ButtonState &buttonState)
{
  int joy = buttonState[ButtonIdMoveJoints];

  if( joy == 0 )
  {
    return;
  }

  switch( m_eMode )
  {
    case TeleopModeFirstPerson:
      buttonMoveFirstPerson(joy);
      break;
    case TeleopModeShoulder:
      buttonMoveShoulder(joy);
      break;
    case TeleopModeElbow:
      buttonMoveElbow(joy);
      break;
    default:
      break;
  }
}

void HekTeleop::buttonMoveFirstPerson(int joy)
{
  //
  // Fixed tuned parameters.
  //
  // RDK TODO:
  // The Upper and lower arm lengths (mm) are for the large Hakateros.
  // Need to dynamically find these values from the robot.
  //
  static double UPPER_ARM = 406.27;   // upper arm (shoulder - elbow) 
  static double LOWER_ARM = 401.60;   // lower arm (elbow - wrist)
  static double V_MAX     = 4.0;      // maximum velocity scale
  static double EPSILON   = 0.7;      // tolerance in L1 angle space

  int       i, j, k;
  double    goal_sign;
  double    A, B, C, D;
  double    alpha, beta, gamma, theta;
  double    dir;
  double    x0, y0;
  double    xp, yp;
  double    b, c;

  if( (i = addJointToTrajectoryPoint("shoulder")) < 0 )
  {
    return;
  }
  else if( (j = addJointToTrajectoryPoint("elbow")) < 0 )
  {
    return;
  }
  else if( (k = addJointToTrajectoryPoint("wrist_pitch")) < 0 )
  {
    return;
  }

  goal_sign = joy < 0.0? -1.0: 1.0;

  if( goal_sign != m_fpState.m_goalSign )
  {
    m_fpState.m_bNewGoal = true;
  }

  //
  // New goal, calculate new targets.
  //
  if( m_fpState.m_bNewGoal )
  {
    m_fpState.m_goalSign = goal_sign;

    A = UPPER_ARM;
    B = LOWER_ARM;
    D = goal_sign * 200.0;  // distance to move in mm

    alpha = m_msgJointTrajPoint.positions[i];
    beta  = m_msgJointTrajPoint.positions[j];
    gamma = m_msgJointTrajPoint.positions[k];

    dir = alpha + beta + gamma;

    x0 = A * sin(alpha) + B * sin(alpha + beta);
    y0 = A * cos(alpha) + B * cos(alpha + beta);
    xp = x0 + D * sin(dir);
    yp = y0 + D * cos(dir);

    // save cartesian target
    m_fpState.m_goalCart.x = xp;
    m_fpState.m_goalCart.y = yp;

    C = sqrt(xp*xp + yp*yp);
    if( C > (A + B) )
    {
      C = A + B;
    }

    theta = atan2(xp, yp);
    b = acos((A*A+C*C-B*B)/(2*A*C));
    c = acos((A*A+B*B-C*C)/(2*A*B));

    // save joint targets
    m_fpState.m_goalJoint.alpha = theta - b;
    m_fpState.m_goalJoint.beta  = M_PI - c;
    m_fpState.m_goalJoint.gamma = dir - 
                                  m_fpState.m_goalJoint.alpha -
                                  m_fpState.m_goalJoint.beta;

    m_msgJointTrajPoint.positions[i]  = m_fpState.m_goalJoint.alpha;
    m_msgJointTrajPoint.velocities[i] = 1.0;  // minimal movement on new goal
    m_msgJointTrajPoint.positions[j]  = m_fpState.m_goalJoint.beta;
    m_msgJointTrajPoint.velocities[j] = 1.0;  // minimal movement on new goal
    m_msgJointTrajPoint.positions[k]  = m_fpState.m_goalJoint.gamma;
    m_msgJointTrajPoint.velocities[k] = 1.0;  // minimal movement on new goal

    m_fpState.m_bNewGoal = false;
  }

  //
  // Same goal, calculate delta targets.
  //
  else
  {
    A = UPPER_ARM;
    B = LOWER_ARM;
    D = 10.0;       // distance used to calculate joint velocities (mm)

    alpha = m_msgJointTrajPoint.positions[i];
    beta  = m_msgJointTrajPoint.positions[j];
    gamma = m_msgJointTrajPoint.positions[k];

    double delta  = abs(m_fpState.m_goalJoint.alpha - alpha) +
                    abs(m_fpState.m_goalJoint.beta - beta) +
                    abs(m_fpState.m_goalJoint.gamma - gamma);

    if( delta < EPSILON )
    {
      m_fpState.m_bNewGoal = true;
    }

    // current position
    x0 = A * sin(alpha) + B * sin(alpha + beta);
    y0 = A * cos(alpha) + B * cos(alpha + beta);

    // calculate direction towards target
    double xdelta = m_fpState.m_goalCart.x - x0;
    double ydelta = m_fpState.m_goalCart.y - y0;
    if( ydelta == 0.0 )
    {
      dir = M_PI/2.0;
    }
    else
    {
      dir = atan2(xdelta, ydelta);
    }

    // target for calculating joint velocities
    xp = x0 + D * sin(dir);
    yp = y0 + D * cos(dir);

    C = sqrt(xp*xp + yp*yp);
    if( C > (A + B) )
    {
      C = A + B;
    }

    theta = atan2(xp, yp);
    b = acos((A*A+C*C-B*B)/(2*A*C));
    c = acos((A*A+B*B-C*C)/(2*A*B));

    double alpha_target = theta - b;
    double beta_target  = M_PI - c;

    // ok, calculate velocities
    double alpha_delta = alpha_target - alpha;
    double beta_delta  = beta_target - beta;
    if( abs(alpha_delta) > abs(beta_delta) )
    {
      beta_delta  = beta_delta/alpha_delta;
      alpha_delta = 1.0;
    }
    else
    {
      alpha_delta = alpha_delta/beta_delta;
      beta_delta = 1.0;
    }

    double scale        = (double)joy/(double)(XBOX360_JOY_MAX * V_MAX);
    double shoulder_vel = alpha_delta * scale;
    double elbow_vel    = beta_delta * scale;

    m_msgJointTrajPoint.positions[i]  = m_fpState.m_goalJoint.alpha;
    m_msgJointTrajPoint.velocities[i] = abs(shoulder_vel);
    m_msgJointTrajPoint.positions[j]  = m_fpState.m_goalJoint.beta;
    m_msgJointTrajPoint.velocities[j] = abs(elbow_vel);
    m_msgJointTrajPoint.positions[k]  = m_fpState.m_goalJoint.gamma;
    m_msgJointTrajPoint.velocities[k] = abs(elbow_vel+shoulder_vel);
  }
}

void HekTeleop::buttonMoveShoulder(int joy)
{
  int     i;        // joint index
  double  pos;      // delta joint position
  double  vel;      // joint velocity

  // no joint found on arm
  if( (i = addJointToTrajectoryPoint("shoulder")) < 0 )
  {
    return;
  }

  pos = degToRad(40.0);
  vel = (double)(joy) / XBOX360_JOY_MAX * 10.0;

  if( vel < 0.0 )
  {
    pos = -pos;
    vel = -vel;
  }

  m_msgJointTrajPoint.positions[i] += pos;
  m_msgJointTrajPoint.velocities[i] = vel;

  m_fpState.m_bNewGoal = true;
}

void HekTeleop::buttonMoveElbow(int joy)
{
  int     i;        // joint index
  double  pos;      // delta joint position
  double  vel;      // joint velocity

  // no joint found on arm
  if( (i = addJointToTrajectoryPoint("elbow")) < 0 )
  {
    return;
  }

  pos = degToRad(40.0);
  vel = (double)(joy) / XBOX360_JOY_MAX * 10.0;

  if( vel < 0.0 )
  {
    pos = -pos;
    vel = -vel;
  }

  m_msgJointTrajPoint.positions[i] += pos;
  m_msgJointTrajPoint.velocities[i] = vel;

  m_fpState.m_bNewGoal = true;
}

void HekTeleop::buttonRotateBase(ButtonState &buttonState)
{
  int     joy = buttonState[ButtonIdRotBase];
  int     i;        // joint index
  double  pos;      // delta joint position
  double  vel;      // joint velocity

  if( joy == 0 )
  {
    return;
  }

  // no joint found on arm
  if( (i = addJointToTrajectoryPoint("base_rot")) < 0 )
  {
    return;
  }

  pos = degToRad(45.0);
  vel = (double)(joy) / XBOX360_JOY_MAX * 20.0;

  if( vel < 0.0 )
  {
    pos = -pos;
    vel = -vel;
  }

  m_msgJointTrajPoint.positions[i] += pos;
  m_msgJointTrajPoint.velocities[i] = vel;
}

void HekTeleop::buttonPitchWrist(ButtonState &buttonState)
{
  int     joy = buttonState[ButtonIdPitchWrist];
  int     i;        // joint index
  double  pos;      // delta joint position
  double  vel;      // joint velocity

  if( joy == 0 )
  {
    return;
  }

  // no joint found on arm
  if( (i = addJointToTrajectoryPoint("wrist_pitch")) < 0 )
  {
    return;
  }

  pos = degToRad(45.0);
  vel = (double)(joy) / XBOX360_JOY_MAX * 20.0;

  if( vel < 0.0 )
  {
    pos = -pos;
    vel = -vel;
  }

  m_msgJointTrajPoint.positions[i] += pos;
  m_msgJointTrajPoint.velocities[i] = vel;

  m_fpState.m_bNewGoal = true;
}

void HekTeleop::buttonRotateWristCw(ButtonState &buttonState)
{
  int   i;

  // off
  if( buttonState[ButtonIdRotWristCw] == 0 )
  {
    return;
  }

  // no joint found on arm
  if( (i = addJointToTrajectoryPoint("wrist_rot")) < 0 )
  {
    return;
  }

  m_msgJointTrajPoint.positions[i] -= degToRad(360.0);
  m_msgJointTrajPoint.velocities[i] = 20.0;
}

void HekTeleop::buttonRotateWristCcw(ButtonState &buttonState)
{
  int   i;

  // off
  if( buttonState[ButtonIdRotWristCcw] == 0 )
  {
    return;
  }

  // no joint found on arm
  if( (i = addJointToTrajectoryPoint("wrist_rot")) < 0 )
  {
    return;
  }

  m_msgJointTrajPoint.positions[i] += degToRad(360.0);
  m_msgJointTrajPoint.velocities[i] = 20.0;
}


//. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 
// Support
//. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 

void HekTeleop::pause()
{
  m_eState = TeleopStatePaused;

  setLED(LEDPatPaused);
}

void HekTeleop::ready()
{
  m_eState = TeleopStateReady;
  m_fpState.m_bNewGoal = true;

  setLED(LEDPatReady);
}

void HekTeleop::driveLEDsFigure8Pattern()
{
  static int nLEDTimeout = -1;
  static int nLEDCounter = 0;
  static int iLED = 0;
  static int LEDPat[] =
  {
    XBOX360_LED_PAT_1_ON, XBOX360_LED_PAT_2_ON,
    XBOX360_LED_PAT_3_ON, XBOX360_LED_PAT_4_ON
  };

  // lazy init
  if( nLEDTimeout < 0 )
  {
    nLEDTimeout = countsPerSecond(0.50);
  }

  // switch pattern
  if( nLEDCounter++ >= nLEDTimeout )
  {
    iLED = (iLED + 1) % arraysize(LEDPat);
    setLED(LEDPat[iLED]);
    nLEDCounter = 0;
  }
}

ssize_t HekTeleop::indexOfRobotJoint(const string &strJointName)
{
  for(size_t i=0; i<m_msgJointState.name.size(); ++i)
  {
    if( m_msgJointState.name[i] == strJointName )
    {
      return (ssize_t)i;
    }
  }

  return -1;
}

ssize_t HekTeleop::indexOfTrajectoryJoint(const string &strJointName)
{
  for(size_t i=0; i<m_msgJointTraj.joint_names.size(); ++i)
  {
    if( m_msgJointTraj.joint_names[i] == strJointName )
    {
      return (ssize_t)i;
    }
  }

  return -1;
}

ssize_t HekTeleop::addJointToTrajectoryPoint(const string &strJointName)
{
  MapJoints::iterator pos;
  ssize_t             i, j;

  //
  // First joint added. So add all joints to trajectory point, with each joint
  // set to its current position and with zero velocity. This would stop any
  // previous joint movements not controlled in current trajectory.
  //
  if( m_msgJointTraj.joint_names.size() == 0 )
  {
    for(pos=m_mapJoints.begin(), i=0; pos!=m_mapJoints.end(); ++pos, ++i)
    {
      m_msgJointTraj.joint_names.push_back(pos->first);
      if( (j = indexOfRobotJoint(strJointName)) >= 0 )
      {
        m_msgJointTrajPoint.positions.push_back(m_msgJointState.position[j]);
      }
      else
      {
        m_msgJointTrajPoint.positions.push_back(0.0);
      }
      m_msgJointTrajPoint.velocities.push_back(0.0);
      m_msgJointTrajPoint.accelerations.push_back(0.0);

      pos->second = i;
    }
  }

  // no joint
  if( (pos = m_mapJoints.find(strJointName)) == m_mapJoints.end() )
  {
    return -1;
  }

  // joint index
  else
  {
    return pos->second;
  }
}

void HekTeleop::clearJointTrajectory()
{
  MapJoints::iterator iter;
  
  m_msgJointTraj.joint_names.clear();
  m_msgJointTraj.points.clear();
  m_msgJointTrajPoint.positions.clear();
  m_msgJointTrajPoint.velocities.clear();
  m_msgJointTrajPoint.accelerations.clear();

  for(iter=m_mapJoints.begin(); iter!=m_mapJoints.end(); ++iter)
  {
    iter->second = -1;
  }
}
