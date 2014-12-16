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
// ROS generated action clients.
//
#include "actionlib/client/simple_action_client.h"
#include "actionlib/client/simple_client_goal_state.h"
#include "hekateros_control/CalibrateAction.h"

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

//
// All joints supported in current Hekaterors product line.
//
static string JointNameBaseRot("base_rot");
static string JointNameShoulder("shoulder");
static string JointNameElbow("elbow");
static string JointNameWristPitch("wrist_pitch");
static string JointNameWristRot("wrist_rot");
static string JointNameGrip("grip");

//
// Link lengths.
//
// RDK: TODO need to self discover these lengths. Current lengths are for the
// 1.3 arm.
//
static double LEN_UPPER_ARM = 461.0;    // upper arm (shoulder to elbow) 
static double LEN_LOWER_ARM = 405.0;    // lower arm (elbow to wrist)


//------------------------------------------------------------------------------
// HekTeleop Class
//------------------------------------------------------------------------------

HekTeleop::HekTeleop(ros::NodeHandle &nh, double hz) :
    m_nh(nh), m_hz(hz), m_acCalib("/hekateros_control/calibrate_as", true)
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
  m_bIsCalibrating    = false;

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

      (ButtonIdFineTune1,     0)
      (ButtonIdFineTune1,     0)

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

  m_bPreemptMove  = false;
  m_fMoveTuning   = 1.0;
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

  strSvc = "/hekateros_control/release";
  m_clientServices[strSvc] =
      m_nh.serviceClient<hekateros_control::Release>(strSvc);

  strSvc = "/hekateros_control/stop";
  m_clientServices[strSvc] =
      m_nh.serviceClient<hekateros_control::Stop>(strSvc);

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
    ROS_INFO("Xbox360 rumble motors set to %d, %d.", motorLeft, motorRight);
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
    cancelCalibration();
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
    cancelCalibration();
  }
  else
  {
    ROS_ERROR("Failed to freeze robot.");
  }
}

void HekTeleop::release()
{
  hekateros_control::Freeze svc;

  if( m_clientServices["/hekateros_control/release"].call(svc) )
  {
    ROS_INFO("Robot released.");
    cancelCalibration();
  }
  else
  {
    ROS_ERROR("Failed to release robot.");
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

void HekTeleop::calibrate()
{
  CalibrateGoal goal;

  ROS_INFO("Initiating calibration...");

  ROS_INFO("  Waiting for calibration action server to start.");

  if( m_acCalib.waitForServer() )
  {
    ROS_INFO("Calibration action server started.");
  }
  else
  {
    ROS_ERROR("Failed to connect to calibration action server.");
    return;
  }

  ROS_INFO("Sending calibration goal.");
  goal.force_recalib = true;

  m_acCalib.sendGoal(goal,
           boost::bind(&HekTeleop::cbCalibDone, this, _1, _2),
           boost::bind(&HekTeleop::cbCalibActive, this),
           boost::bind(&HekTeleop::cbCalibFeedback, this, _1));
}

// Called once when the goal completes
void HekTeleop::cbCalibDone(const actionlib::SimpleClientGoalState &state,
                            const CalibrateResultConstPtr          &result)
{
  m_bIsCalibrating = false;

  ROS_INFO("Finished in state [%s]", state.toString().c_str());
  //ROS_INFO("Answer: %i", result->sequence.back());
}

// Called once when the goal becomes active
void HekTeleop::cbCalibActive()
{
  ROS_INFO("Calibration goal just went active.");

  m_bIsCalibrating = true;
}

// Called every time feedback is received for the goal
void HekTeleop::cbCalibFeedback(const CalibrateFeedbackConstPtr &feedback)
{
  //ROS_DEBUG("Calibration feedback.");
}

void HekTeleop::cancelCalibration()
{
  if( isCalibrating() )
  {
    ROS_WARN("Canceled calibration.");
    m_acCalib.cancelGoal();
  }
}

bool HekTeleop::isCalibrating()
{
  if( !m_bIsCalibrating )
  {
    return false;
  }

  actionlib::SimpleClientGoalState state = m_acCalib.getState();

  return  (state == actionlib::SimpleClientGoalState::PENDING) ||
          (state == actionlib::SimpleClientGoalState::ACTIVE);
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

void HekTeleop::stop(const vector<string> &vecJointNames)
{
  hekateros_control::Stop svc;

  svc.request.joint_names = vecJointNames;

  if( m_clientServices["/hekateros_control/stop"].call(svc) )
  {
    ROS_INFO("Robot joint(s) stopped.");
  }
  else
  {
    ROS_ERROR("Failed to call 'stop' robot service.");
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

  // new working trajectory
  if( hasWorkingTrajectory() )
  {
    m_msgJointTraj.header.stamp    = ros::Time::now();
    m_msgJointTraj.header.frame_id = "0";
    m_msgJointTraj.header.seq++;
    m_msgJointTraj.points.push_back(m_msgJointTrajPoint);

    // publish
    m_publishers["/hekateros_control/joint_command"].publish(m_msgJointTraj);

    bPubJointCmd = true;
  }

  // transition to no active trajectory
  else if( bPubJointCmd && !hasActiveTrajectory() )
  {
    if( m_msgRobotStatus.in_motion.val == TriState::FALSE )
    {
      //freeze();
      clearActiveTrajectory();
      bPubJointCmd = false;
    }
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

  // state auto-transitions
  if( (m_msgRobotStatus.is_calibrated.val == TriState::FALSE) &&
      (m_eState == TeleopStateReady) )
  {
    gotoUncalib();
  }
  else if( (m_msgRobotStatus.is_calibrated.val == TriState::TRUE) &&
           (m_eState == TeleopStateUncalib) )
  {
    gotoReady();
  }
}

void HekTeleop::cbJointState(const HekJointStateExtended &msg)
{
  static float  DeadZone  = 256.0;    // no haptic feedback in this zone
  static float  Scale     = (float)XBOX360_RUMBLE_RIGHT_MAX/(1023.0 - DeadZone);
                                      // effort to rumble scale factor

  ssize_t   i;
  float     effort;
  int       rumbleRight;

  ROS_DEBUG("Received joint state.");

  m_nWdRobotCounter = 0;
  m_msgJointState   = msg;
  m_bRcvdJointState = true;

  m_mapCurPos.clear();
  m_mapCurVel.clear();

  for(i=0; i<msg.name.size(); ++i)
  {
    // current joint position and velocity
    m_mapCurPos[msg.name[i]] = msg.position[i]; 
    m_mapCurVel[msg.name[i]] = msg.velocity[i]; 

    //
    // Gripper tactile feedback.
    //
    if( msg.name[i] == "grip" &&
        m_bHasFullComm &&
        (m_eState == TeleopStateReady) )
    {
      effort = fabs((float)msg.effort[i]);

      if( effort > DeadZone )
      {
        // Setting this often seems to slow down button response. So set at
        // one fixed value.
        //rumbleRight = (int)((effort - DeadZone) * Scale);
        rumbleRight = XBOX360_RUMBLE_RIGHT_MAX/2;
      }
      else
      {
        rumbleRight = 0;
      }

      setRumble(m_rumbleLeft, rumbleRight);
    }
  }

  //
  // Auto-discovery of all joints to keep joint teleoperation state.
  //
  if( msg.name.size() > m_mapTeleop.size() )
  {
    m_mapTeleop.clear();

    for(i=0; i<msg.name.size(); ++i)
    {
      m_mapTeleop[msg.name[i]] = false;
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
      case TeleopStateUncalib:
        execAllButtonActions(buttonState);
        break;
      case TeleopStatePaused:
        buttonStart(buttonState);    // only active button in pause state
        break;
      case TeleopStateUninit:
      default:
        gotoPause();
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
  switch( m_eState )
  {
    case TeleopStatePaused:
      driveLEDsFigure8Pattern();
      break;
    case TeleopStateUncalib:
      driveLEDsRightFlashPattern();
      break;
    default:
      break;
  }
}

void HekTeleop::putRobotInSafeMode(bool bHard)
{
  // stop robot
  freeze();

  // set robot mode
  setRobotMode(HekRobotModeAuto);
 
  if( bHard )
  {
    // nothing to do
  }
  
  m_eState = TeleopStateUninit;

  setRumble(0, 0);
  setLED(LEDPatOn);
}

bool HekTeleop::canMove()
{
  if( (m_eState == TeleopStateReady) &&
      (m_msgRobotStatus.is_calibrated.val == industrial_msgs::TriState::TRUE) &&
      (m_msgRobotStatus.e_stopped.val == industrial_msgs::TriState::FALSE) &&
      (m_msgRobotStatus.in_error.val == industrial_msgs::TriState::FALSE) )
  {
    return true;
  }
  else
  {
    return false;
  }
}

bool HekTeleop::canCalibrate()
{
  if( (m_eState == TeleopStateUncalib) &&
      (m_msgRobotStatus.e_stopped.val == industrial_msgs::TriState::FALSE) &&
      (m_msgRobotStatus.in_error.val == industrial_msgs::TriState::FALSE) )
  {
    return true;
  }
  else
  {
    return false;
  }
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

  buttonState[ButtonIdPrevJoint]      = msg.dpad_down;
  buttonState[ButtonIdNextJoint]      = msg.dpad_up;

  buttonState[ButtonIdFineTune1]      = msg.left_joy_click;
  buttonState[ButtonIdFineTune2]      = msg.right_joy_click;

  buttonState[ButtonIdMoveJoints]     = msg.left_joy_y;
  buttonState[ButtonIdRotBase]        = msg.left_joy_x;
  buttonState[ButtonIdPitchWrist]     = msg.right_joy_y;

  buttonState[ButtonIdRotWristCw]     = msg.left_bump;
  buttonState[ButtonIdRotWristCcw]    = msg.right_bump;

  buttonState[ButtonIdOpenGripper]    = msg.left_trig;
  buttonState[ButtonIdCloseGripper]   = msg.right_trig;
}

void HekTeleop::execAllButtonActions(ButtonState &buttonState)
{
  //
  // Teleoperation state change by button.
  //
  switch( m_eState )
  {
    case TeleopStateUncalib:
    case TeleopStateReady:
      buttonPause(buttonState);
      break;
    case TeleopStatePaused:
      buttonStart(buttonState);
      break;
    case TeleopStateUninit:
    default:
      return;
  }

  //
  // Teleoperation state.
  //
  switch( m_eState )
  {
    case TeleopStateReady:
      execMoveButtonActions(buttonState);
      break;
    case TeleopStateUncalib:
      execCalibButtonActions(buttonState);
      break;
    case TeleopStatePaused:
      return;
  }
}

void HekTeleop::execMoveButtonActions(ButtonState &buttonState)
{
  // Emergency stop.
  buttonEStop(buttonState);

  //
  // Teleoperation Modes.
  //
  buttonToggleMode(buttonState);
  
  if( m_eMode != TeleopModeFirstPerson )
  {
    buttonPrevJoint(buttonState);
    buttonNextJoint(buttonState);
  }
 
  buttonFineTune(buttonState);

  //
  // Moves.
  //
  if( canMove() )
  {
    clearWorkingTrajectory();
    resetActiveTeleop();

    // preemptive canned moves
    buttonGotoBalancedPos(buttonState);
    buttonGotoParkedPos(buttonState);
    buttonGotoZeroPt(buttonState);

    if( m_bPreemptMove )
    {
      clearActiveTrajectory();
    }

    // manually controlled moves
    else
    {
      buttonCloseGripper(buttonState);
      buttonOpenGripper(buttonState);
      buttonPitchWrist(buttonState);
      buttonRotateWristCw(buttonState);
      buttonRotateWristCcw(buttonState);
      buttonRotateBase(buttonState);
      buttonMoveJoints(buttonState);

      publishJointCmd();
      stopUnteleopJoints();
    }
  }
}

void HekTeleop::execCalibButtonActions(ButtonState &buttonState)
{
  // Emergency stop.
  buttonEStop(buttonState);

  buttonReleaseArm(buttonState);
  buttonFreezeArm(buttonState);

  if( canCalibrate() )
  {
    buttonCalibrate(buttonState);
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

    if( m_msgRobotStatus.is_calibrated.val == TriState::TRUE )
    {
      gotoReady();
    }
    else
    {
      gotoUncalib();
    }
  }
}

void HekTeleop::buttonPause(ButtonState &buttonState)
{
  if( buttonOffToOn(ButtonIdPause, buttonState) )
  {
    ROS_INFO("Manual operation paused, auto mode enabled.");

    setRobotMode(HekRobotModeAuto);

    gotoPause();
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

void HekTeleop::buttonFineTune(ButtonState &buttonState)
{
  if( buttonOffToOn(ButtonIdFineTune1, buttonState) ||
      buttonOffToOn(ButtonIdFineTune2, buttonState) )
  {
    if( m_fMoveTuning >= 1.0 )
    {
      m_fMoveTuning = 0.25;
    }
    else
    {
      m_fMoveTuning = 1.0;
    }

    ROS_INFO("Manual movement scale set to %.1lf%%.", m_fMoveTuning * 100.0);
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

void HekTeleop::buttonGotoBalancedPos(ButtonState &buttonState)
{
  if( buttonOffToOn(ButtonIdGotoBalPos, buttonState) )
  {
    gotoBalancedPos();
    m_fpState.m_bNewGoal = true;
    m_bPreemptMove = true;
  }
}

void HekTeleop::buttonGotoParkedPos(ButtonState &buttonState)
{
  if( buttonOffToOn(ButtonIdGotoParkedPos, buttonState) )
  {
    gotoParkedPos();
    m_fpState.m_bNewGoal = true;
    m_bPreemptMove = true;
  }
}

void HekTeleop::buttonGotoZeroPt(ButtonState &buttonState)
{
  if( buttonOffToOn(ButtonIdGotoZeroPt, buttonState) )
  {
    gotoZeroPt();
    m_fpState.m_bNewGoal = true;
    m_bPreemptMove = true;
  }
}

void HekTeleop::buttonCalibrate(ButtonState &buttonState)
{
  if( buttonOffToOn(ButtonIdCalibrate, buttonState) )
  {
    if( isCalibrating() )
    {
      cancelCalibration();
    }
    else
    {
      calibrate();
    }
  }
}

void HekTeleop::buttonReleaseArm(ButtonState &buttonState)
{
  if( HekTeleop::buttonOffToOn(ButtonIdRelease, buttonState) )
  {
    release();
  }
}

void HekTeleop::buttonFreezeArm(ButtonState &buttonState)
{
  if( buttonOffToOn(ButtonIdFreeze, buttonState) )
  {
    freeze();
  }
}

void HekTeleop::buttonCloseGripper(ButtonState &buttonState)
{
  static int    deadzone  = 10;
  static double dpos      = degToRad(40.0);
  static double maxvel    = degToRad(90.0);

  int     trigger = buttonState[ButtonIdCloseGripper];
  double  r;
  double  pos;
  double  vel;

  //
  // Two buttons control gripper movement.
  //
  if( trigger <= deadzone )
  {
    return;
  }

  // no joint
  if( m_mapCurPos.find(JointNameGrip) == m_mapCurPos.end() )
  {
    return;
  }

  // goal position and velocity
  r   = (double)(trigger-deadzone) / (double)(XBOX360_TRIGGER_MAX-deadzone);
  pos = m_mapCurPos[JointNameGrip] - dpos;
  vel = m_fMoveTuning * r * maxvel;

  // new joint trajectory point component
  if( setJoint(JointNameGrip, pos, vel) >= 0 )
  {
    ROS_INFO("Closing %s at %.1lfdeg/s.", JointNameGrip.c_str(), radToDeg(vel));
  }
}

void HekTeleop::buttonOpenGripper(ButtonState &buttonState)
{
  static int    deadzone  = 10;
  static double dpos      = degToRad(40.0);
  static double maxvel    = degToRad(90.0);

  int     trigger = buttonState[ButtonIdOpenGripper];
  double  r;
  double  pos;
  double  vel;

  //
  // Two buttons control gripper movement.
  //
  if( trigger <= deadzone )
  {
    return;
  }

  // no joint
  if( m_mapCurPos.find(JointNameGrip) == m_mapCurPos.end() )
  {
    return;
  }

  // goal position and velocity
  r   = (double)(trigger-deadzone) / (double)(XBOX360_TRIGGER_MAX-deadzone);
  pos = m_mapCurPos[JointNameGrip] + dpos;
  vel = m_fMoveTuning * r * maxvel;

  // new joint trajectory point component
  if( setJoint(JointNameGrip, pos, vel) >= 0 )
  {
    ROS_INFO("Opening %s at %.1lfdeg/s.", JointNameGrip.c_str(), radToDeg(vel));
  }
}

void HekTeleop::buttonMoveJoints(ButtonState &buttonState)
{
  int joy = buttonState[ButtonIdMoveJoints];

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
  static double UPPER_ARM = 461.0;    // upper arm (shoulder - elbow) 
  static double LOWER_ARM = 405.0;    // lower arm (elbow - wrist)
  static double V_MAX     = degToRad(40.0); // maximum velocity scale
  static double EPSILON   = 0.7;      // tolerance in L1 angle space

  int       i, j, k;
  double    goal_sign;
  double    A, B, C, D;
  double    alpha, beta, gamma, theta;
  double    dir;
  double    x0, y0;
  double    xp, yp;
  double    b, c;

  if( joy == 0 )
  {
    return;
  }

  // no joint state available
  if( (m_mapCurPos.find(JointNameShoulder)   == m_mapCurPos.end()) ||
      (m_mapCurPos.find(JointNameElbow)      == m_mapCurPos.end()) ||
      (m_mapCurPos.find(JointNameWristPitch) == m_mapCurPos.end()) )
  {
    return;
  }

  goal_sign = joy < 0.0? -1.0: 1.0;

  if( goal_sign != m_fpState.m_goalSign )
  {
    m_fpState.m_bNewGoal = true;
  }

  alpha = m_mapCurPos[JointNameShoulder];
  beta  = m_mapCurPos[JointNameElbow];
  gamma = m_mapTeleop[JointNameWristPitch]? m_mapGoalPos[JointNameWristPitch]:
                                            m_mapCurPos[JointNameWristPitch];

  //
  // New goal, calculate new targets.
  //
  if( m_fpState.m_bNewGoal )
  {
    m_fpState.m_goalSign = goal_sign;

    A = LEN_UPPER_ARM;
    B = LEN_LOWER_ARM;
    D = goal_sign * 400.0;  // distance to move in mm

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

    // minimum movement on new goal
    setJoint(JointNameShoulder,   m_fpState.m_goalJoint.alpha, degToRad(2.5));
    setJoint(JointNameElbow,      m_fpState.m_goalJoint.beta,  degToRad(2.5));
    setJoint(JointNameWristPitch, m_fpState.m_goalJoint.gamma, degToRad(5.0));

    m_fpState.m_bNewGoal = false;

    ROS_INFO("Moving in first-person mode, reach at %.1lfmm.", reach());
  }

  //
  // Same goal, calculate delta targets.
  //
  else
  {
    A = LEN_UPPER_ARM;
    B = LEN_LOWER_ARM;
    D = 200.0;      // distance used to calculate joint velocities (mm)

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

    double scale        = (double)joy/(double)(XBOX360_JOY_MAX) * V_MAX;
    double shoulder_vel = alpha_delta * scale * m_fMoveTuning;
    double elbow_vel    = beta_delta * scale * m_fMoveTuning;

    setJoint(JointNameShoulder,   m_fpState.m_goalJoint.alpha,
                                  abs(shoulder_vel));
    setJoint(JointNameElbow,      m_fpState.m_goalJoint.beta,
                                  abs(elbow_vel));
    setJoint(JointNameWristPitch, m_fpState.m_goalJoint.gamma,
                                  abs(shoulder_vel+elbow_vel));

    ROS_INFO("Moving in first-person mode, reach at %.1lfmm.", reach());
  }
}

void HekTeleop::buttonMoveShoulder(int joy)
{
  static double dpos      = degToRad(40.0);
  static double maxvel    = degToRad(60.0);

  double  r;        // joy ratio
  double  pos;      // delta joint position
  double  vel;      // joint velocity

  if( joy == 0 )
  {
    return;
  }

  // no joint
  if( m_mapCurPos.find(JointNameShoulder) == m_mapCurPos.end() )
  {
    return;
  }

  // goal position and velocity
  r   = (double)(joy) / (double)XBOX360_JOY_MAX;
  pos = dpos;
  vel = m_fMoveTuning * r * maxvel;

  if( vel < 0.0 )
  {
    pos = -pos;
  }

  pos = m_mapCurPos[JointNameShoulder] + pos;

  // new joint trajectory point component
  if( setJoint(JointNameShoulder, pos, vel) >= 0 )
  {
    m_fpState.m_bNewGoal = true;
    ROS_INFO("Moving %s at %.1lfdeg/s.",
        JointNameShoulder.c_str(), radToDeg(vel));
  }
}

void HekTeleop::buttonMoveElbow(int joy)
{
  static double dpos      = degToRad(40.0);
  static double maxvel    = degToRad(60.0);

  double  r;        // joy ratio
  double  pos;      // delta joint position
  double  vel;      // joint velocity

  if( joy == 0 )
  {
    return;
  }

  // no joint
  if( m_mapCurPos.find(JointNameElbow) == m_mapCurPos.end() )
  {
    return;
  }

  // goal position and velocity
  r   = (double)(joy) / (double)XBOX360_JOY_MAX;
  pos = dpos;
  vel = m_fMoveTuning * r * maxvel;

  if( vel < 0.0 )
  {
    pos = -pos;
  }

  pos = m_mapCurPos[JointNameElbow] + pos;

  // new joint trajectory point component
  if( setJoint(JointNameElbow, pos, vel) >= 0 )
  {
    m_fpState.m_bNewGoal = true;
    ROS_INFO("Moving %s at %.1lfdeg/s.",
        JointNameElbow.c_str(), radToDeg(vel));
  }
}

void HekTeleop::buttonRotateBase(ButtonState &buttonState)
{
  static double dpos      = degToRad(120.0);
  static double minvel    = degToRad(2.0);
  static double maxvel    = degToRad(100.0);

  int     joy = buttonState[ButtonIdRotBase];
  double  ratioJoy;   // joy ratio
  double  ratioReach; // reach ration
  double  pos;        // delta joint position
  double  vel;        // joint velocity

  if( joy == 0 )
  {
    return;
  }

  // no joint
  if( m_mapCurPos.find(JointNameBaseRot) == m_mapCurPos.end() )
  {
    return;
  }

  // goal position and velocity
  ratioJoy    = (double)(-joy) / (double)XBOX360_JOY_MAX;
  ratioReach  = (LEN_UPPER_ARM + LEN_LOWER_ARM - reach()) /
                                (LEN_UPPER_ARM + LEN_LOWER_ARM);
  pos         = dpos;
  vel         = m_fMoveTuning * ratioJoy * ratioReach * maxvel;

  // notch
  if( (vel < 0.0) && (vel > minvel) )
  {
    vel = -minvel;
  }
  else if( (vel > 0.0) && (vel < minvel) )
  {
    vel = minvel;
  }

  if( vel < 0.0 )
  {
    pos = -pos;
  }

  pos = m_mapCurPos[JointNameBaseRot] + pos;

  // new joint trajectory point component
  if( setJoint(JointNameBaseRot, pos, vel) >= 0 )
  {
    ROS_INFO("Rotating %s at %.1lfdeg/s.",
        JointNameBaseRot.c_str(), radToDeg(vel));
  }
}

void HekTeleop::buttonPitchWrist(ButtonState &buttonState)
{
  static double dpos      = degToRad(40.0);
  static double maxvel    = degToRad(60.0);

  int     joy = buttonState[ButtonIdPitchWrist];
  double  r;        // joy ratio
  double  pos;      // delta joint position
  double  vel;      // joint velocity

  if( joy == 0 )
  {
    return;
  }

  // no joint
  if( m_mapCurPos.find(JointNameWristPitch) == m_mapCurPos.end() )
  {
    return;
  }

  // goal position and velocity
  r   = (double)(joy) / (double)XBOX360_JOY_MAX;
  pos = dpos;
  vel = m_fMoveTuning * r * maxvel;

  if( vel < 0.0 )
  {
    pos = -pos;
  }

  pos = m_mapCurPos[JointNameWristPitch] + pos;

  // new joint trajectory point component
  if( setJoint(JointNameWristPitch, pos, vel) >= 0 )
  {
    m_fpState.m_bNewGoal       = true;
    ROS_INFO("Pitching %s at %.1lfdeg/s.",
        JointNameWristPitch.c_str(), radToDeg(vel));
  }
}

void HekTeleop::buttonRotateWristCw(ButtonState &buttonState)
{
  static double TuneDeltaPos  = degToRad(360.0);  
  static double TuneMaxVel    = degToRad(120.0);

  PosVel  goal; // new goal
  //double  pos;
  //double  vel;

  //
  // Two buttons control wrist rotation.
  //
  if( buttonState[ButtonIdRotWristCw] == 0 )
  {
    return;
  }

  // no joint
  // if( nojoint(JointNameWristRot) )
  if( m_mapCurPos.find(JointNameWristRot) == m_mapCurPos.end() )
  {
    return;
  }

  // goal position and velocity
  goal.m_fJointPos = m_mapCurPos[JointNameWristRot] - TuneDeltaPos;
  goal.m_fJointVel = m_fMoveTuning * TuneMaxVel;

  //if( isNewGoal(goal, degToRad(0.5), degToRad(5.0)) )
  //{
  // new joint trajectory point component
    //if( setJoint(JointNameWristRot, goal) >= 0 )
  if( setJoint(JointNameWristRot, goal.m_fJointPos, goal.m_fJointVel) >= 0 )
    {
      ROS_INFO("Rotating %s CW at %.1lfdeg/s.",
        JointNameWristRot.c_str(), radToDeg(goal.m_fJointVel));
    }
  //}
}

void HekTeleop::buttonRotateWristCcw(ButtonState &buttonState)
{
  static double dpos      = degToRad(360.0);
  static double maxvel    = degToRad(120.0);

  double  pos;
  double  vel;

  //
  // Two buttons control wrist rotation.
  //
  if( buttonState[ButtonIdRotWristCcw] == 0 )
  {
    return;
  }

  // no joint
  if( m_mapCurPos.find(JointNameWristRot) == m_mapCurPos.end() )
  {
    return;
  }

  // goal position and velocity
  pos = m_mapCurPos[JointNameWristRot] + dpos;
  vel = m_fMoveTuning * maxvel;

  // new joint trajectory point component
  if( setJoint(JointNameWristRot, pos, vel) >= 0 )
  {
    ROS_INFO("Rotating %s CCW at %.1lfdeg/s.", JointNameWristRot.c_str(), radToDeg(vel));
  }
}


//. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 
// Support
//. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 

void HekTeleop::gotoPause()
{
  m_eState = TeleopStatePaused;

  setRumble(0, 0);
  setLED(LEDPatPaused);
}

void HekTeleop::gotoReady()
{
  m_eState = TeleopStateReady;
  m_fpState.m_bNewGoal = true;

  setLED(LEDPatReady);
}

void HekTeleop::gotoUncalib()
{
  m_eState = TeleopStateUncalib;

  setLED(LEDPatUncalib);
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

void HekTeleop::driveLEDsRightFlashPattern()
{
  static int nLEDTimeout = -1;
  static int nLEDCounter = 0;
  static int iLED = 0;
  static int LEDPat[] =
  {
    XBOX360_LED_PAT_2_ON, XBOX360_LED_PAT_ALL_OFF
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

void HekTeleop::stopUnteleopJoints()
{
  MapBool::iterator   iter;
  MapDouble::iterator p;
  MapDouble::iterator q;
  vector<string>      vecJointNames;

  for(iter = m_mapTeleop.begin(); iter != m_mapTeleop.end(); ++iter)
  {
    // active
    if( iter->second )
    {
      continue;
    }

    p = m_mapGoalPos.find(iter->first);
    q = m_mapGoalVel.find(iter->first);

    // previously teleoperated joint
    if( (p != m_mapGoalPos.end()) && (q != m_mapGoalVel.end()) )
    {
      // non-zero velocity
      if( fabs(q->second) > 0.0 )
      {
        vecJointNames.push_back(iter->first);
      }

      m_mapGoalPos.erase(p);
      m_mapGoalVel.erase(q);
    }

    iter->second = false;
  }

  if( vecJointNames.size() > 0 )
  {
    stop(vecJointNames);
  }
}

ssize_t HekTeleop::setJoint(const string &strJointName, double pos, double vel)
{
  MapDouble::iterator p = m_mapGoalPos.find(strJointName);
  MapDouble::iterator q = m_mapGoalVel.find(strJointName);
  ssize_t             i = -1;

  // there exists a previous joint trajectory point component
  if( (p != m_mapGoalPos.end()) && (q != m_mapGoalVel.end()) )
  {
    // previous joint trajectory point is different
    if( (pos != p->second) || (vel != q->second) )
    {
      if( (i = addJointToTrajectoryPoint(strJointName)) >= 0 )
      {
        m_msgJointTrajPoint.positions[i]  = pos;
        m_msgJointTrajPoint.velocities[i] = vel;

        p->second = pos;
        q->second = vel;
      }
    }

    m_mapTeleop[strJointName] = true;
  }

  // no active joint trajectory point component
  else if( (i = addJointToTrajectoryPoint(strJointName)) >= 0 )
  {
    m_msgJointTrajPoint.positions[i]  = pos;
    m_msgJointTrajPoint.velocities[i] = vel;

    m_mapGoalPos[strJointName] = pos;
    m_mapGoalVel[strJointName] = vel;

    m_mapTeleop[strJointName] = true;
  }

  return i;
}

ssize_t HekTeleop::addJointToTrajectoryPoint(const string &strJointName)
{
  MapJointTraj::iterator  p;
  MapDouble::iterator     q;
  ssize_t                 i;

  // joint already added
  if( (p = m_mapTraj.find(strJointName)) != m_mapTraj.end() )
  {
    return p->second;
  }

  // add new joint with null trajectory
  else if( (q = m_mapCurPos.find(strJointName)) != m_mapCurPos.end() )
  {
    m_msgJointTraj.joint_names.push_back(strJointName);
    m_msgJointTrajPoint.positions.push_back(q->second);
    m_msgJointTrajPoint.velocities.push_back(m_mapCurVel[strJointName]);
    m_msgJointTrajPoint.accelerations.push_back(0.0);
    i = m_msgJointTraj.joint_names.size() - 1;
    m_mapTraj[strJointName] = (int)i;
    return i;
  }

  // unknown joint
  else
  {
    return -1;
  }
}

void HekTeleop::clearWorkingTrajectory()
{
  m_msgJointTraj.joint_names.clear();
  m_msgJointTraj.points.clear();

  m_msgJointTrajPoint.positions.clear();
  m_msgJointTrajPoint.velocities.clear();
  m_msgJointTrajPoint.accelerations.clear();

  m_mapTraj.clear();
}

void HekTeleop::clearActiveTrajectory()
{
  m_mapGoalPos.clear();
  m_mapGoalVel.clear();
}

void HekTeleop::resetActiveTeleop()
{
  MapBool::iterator iter;

  for(iter = m_mapTeleop.begin(); iter != m_mapTeleop.end(); ++iter)
  {
    iter->second = false;
  }

  m_bPreemptMove = false;
}

double HekTeleop::reach()
{
  return  LEN_UPPER_ARM * sin(m_mapCurPos[JointNameShoulder]) +
          LEN_LOWER_ARM * sin(m_mapCurPos[JointNameElbow]);
}
