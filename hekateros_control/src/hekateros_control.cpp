////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics Hekateros Robotic Manipulator ROS Package
//
// Link:      https://github.com/roadnarrows-robotics/hekateros
//
// ROS Node:  hekateros_control
//
// File:      hekateros_control.cpp
//
/*! \file
 *
 * $LastChangedDate$
 * $Rev$
 *
 * \brief The ROS hekateros_control node class implementation.
 *
 * \author Danial Packard (daniel@roadnarrows.com)
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \par Copyright:
 * (C) 2013-2016  RoadNarrows
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
#include <stdlib.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>

#include <string>
#include <map>

//
// Boost libraries
//
#include <boost/bind.hpp>

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
#include "hekateros_control/Gpio.h"

//
// ROS generatated hekateros services.
//
#include "hekateros_control/Calibrate.h"
#include "hekateros_control/ClearAlarms.h"
#include "hekateros_control/CloseGripper.h"
#include "hekateros_control/ConfigGpio.h"
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
#include "hekateros_control/ReadGpio.h"
#include "hekateros_control/Release.h"
#include "hekateros_control/ReloadConfig.h"
#include "hekateros_control/ResetEStop.h"
#include "hekateros_control/SetRobotMode.h"
#include "hekateros_control/Stop.h"
#include "hekateros_control/WriteGpio.h"

//
// ROS generated action servers.
//
#include "hekateros_control/CalibrateAction.h"

//
// RoadNarrows
//
#include "rnr/rnrconfig.h"
#include "rnr/log.h"

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
#include "hekateros_control.h"


using namespace std;
using namespace hekateros;
using namespace hekateros_control;


//------------------------------------------------------------------------------
// HekaterosControl Class
//------------------------------------------------------------------------------

HekaterosControl::HekaterosControl(ros::NodeHandle &nh, double hz) :
    m_nh(nh), m_hz(hz)
{
}

HekaterosControl::~HekaterosControl()
{
  disconnect();
}

int HekaterosControl::configure(const string &strCfgFile)
{
  HekXmlCfg xml;  // hekateros xml instance
  int       rc;   // return code

  if( (rc = xml.load(*m_robot.getHekDesc(), HekSysCfgPath, strCfgFile)) < 0 )
  {
    ROS_ERROR("Loading XML file '%s' failed.", strCfgFile.c_str());
  }

  else if( (rc = m_robot.getHekDesc()->markAsDescribed()) < 0 )
  {
    ROS_ERROR("Failed to finalize descriptions.");
  }

  else
  {
    ROS_INFO("Hekateros description loaded:\n\t %s\n\t %s",
       xml.getFileName().c_str(),
       m_robot.getHekDesc()->getFullProdBrief().c_str());
    rc = HEK_OK;
  }

  return rc;
}


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Services
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

void HekaterosControl::advertiseServices()
{
  string  strSvc;

  strSvc = "calibrate";
  m_services[strSvc] = m_nh.advertiseService(strSvc,
                                          &HekaterosControl::calibrate,
                                          &(*this));

  strSvc = "clear_alarms";
  m_services[strSvc] = m_nh.advertiseService(strSvc,
                                          &HekaterosControl::clearAlarms,
                                          &(*this));

  strSvc = "close_gripper";
  m_services[strSvc] = m_nh.advertiseService(strSvc,
                                          &HekaterosControl::closeGripper,
                                          &(*this));
 
  strSvc = "config_gpio";
  m_services[strSvc] = m_nh.advertiseService(strSvc,
                                          &HekaterosControl::configGpio,
                                          &(*this));
 
  strSvc = "estop";
  m_services[strSvc] = m_nh.advertiseService(strSvc,
                                          &HekaterosControl::estop,
                                          &(*this));

  strSvc = "freeze";
  m_services[strSvc] = m_nh.advertiseService(strSvc,
                                          &HekaterosControl::freeze,
                                          &(*this));

  strSvc = "get_product_info";
  m_services[strSvc] = m_nh.advertiseService(strSvc,
                                          &HekaterosControl::getProductInfo,
                                          &(*this));

  strSvc = "goto_balanced";
  m_services[strSvc] = m_nh.advertiseService(strSvc,
                                          &HekaterosControl::gotoBalancedPos,
                                          &(*this));

  strSvc = "goto_parked";
  m_services[strSvc] = m_nh.advertiseService(strSvc,
                                          &HekaterosControl::gotoParkedPos,
                                          &(*this));

  strSvc = "goto_zero";
  m_services[strSvc] = m_nh.advertiseService(strSvc,
                                          &HekaterosControl::gotoZeroPt,
                                          &(*this));

  strSvc = "is_alarmed";
  m_services[strSvc] = m_nh.advertiseService(strSvc,
                                          &HekaterosControl::isAlarmed,
                                          &(*this));

  strSvc = "is_calibrated";
  m_services[strSvc] = m_nh.advertiseService(strSvc,
                                          &HekaterosControl::isCalibrated,
                                          &(*this));

  strSvc = "is_desc_loaded";
  m_services[strSvc] = m_nh.advertiseService(strSvc,
                                          &HekaterosControl::isDescLoaded,
                                          &(*this));

  strSvc = "open_gripper";
  m_services[strSvc] = m_nh.advertiseService(strSvc,
                                          &HekaterosControl::openGripper,
                                          &(*this));
 
  strSvc = "read_gpio";
  m_services[strSvc] = m_nh.advertiseService(strSvc,
                                          &HekaterosControl::readGpio,
                                          &(*this));

  strSvc = "release";
  m_services[strSvc] = m_nh.advertiseService(strSvc,
                                          &HekaterosControl::release,
                                          &(*this));

  strSvc = "reload_config";
  m_services[strSvc] = m_nh.advertiseService(strSvc,
                                          &HekaterosControl::reloadConfig,
                                          &(*this));

  strSvc = "reset_estop";
  m_services[strSvc] = m_nh.advertiseService(strSvc,
                                          &HekaterosControl::resetEStop,
                                          &(*this));

  strSvc = "set_robot_mode";
  m_services[strSvc] = m_nh.advertiseService(strSvc,
                                          &HekaterosControl::setRobotMode,
                                          &(*this));

  strSvc = "stop";
  m_services[strSvc] = m_nh.advertiseService(strSvc,
                                          &HekaterosControl::stop,
                                          &(*this));

  strSvc = "write_gpio";
  m_services[strSvc] = m_nh.advertiseService(strSvc,
                                          &HekaterosControl::writeGpio,
                                          &(*this));
}

bool HekaterosControl::calibrate(Calibrate::Request  &req,
                                 Calibrate::Response &rsp)
{
  const char *svc = "calibrate";

  ROS_DEBUG("%s", svc);

  rsp.rc = m_robot.calibrate(req.force_recalib);

  if( rsp.rc == HEK_OK )
  {
    ROS_INFO("Robot calibrated.");
    return true;
  }
  else
  {
    ROS_ERROR("%s failed. %s(rc=%d).", svc, getStrError(rsp.rc), rsp.rc);
    return false;
  }
}

bool HekaterosControl::clearAlarms(ClearAlarms::Request  &req,
                                   ClearAlarms::Response &rsp)
{
  const char *svc = "clear_alarms";

  ROS_DEBUG("%s", svc);

  m_robot.clearAlarms();

  return true;
}

bool HekaterosControl::closeGripper(CloseGripper::Request  &req,
                                    CloseGripper::Response &rsp)
{
  const char *svc = "close_gripper";
  int         rc;

  ROS_DEBUG("%s", svc);

  rc = m_robot.closeGripper();

  if( rc == HEK_OK )
  {
    ROS_INFO("Closing gripper.");
    return true;
  }
  else
  {
    ROS_ERROR("%s failed. %s(rc=%d).", svc, getStrError(rc), rc);
    return false;
  }
}

bool HekaterosControl::configGpio(ConfigGpio::Request  &req,
                                  ConfigGpio::Response &rsp)
{
  const char *svc = "config_gpio";
  int         rc;

  ROS_DEBUG("%s", svc);

  // RDK rc = m_robot.x();

  if( rc == HEK_OK )
  {
    ROS_INFO("Configured GPIO.");
    return true;
  }
  else
  {
    ROS_ERROR("%s failed. %s(rc=%d).", svc, getStrError(rc), rc);
    return false;
  }
}

bool HekaterosControl::estop(EStop::Request  &req,
                             EStop::Response &rsp)
{
  const char *svc = "estop";

  ROS_DEBUG("%s", svc);

  m_robot.estop();

  ROS_INFO("ESTOPPED! You must issue a \"reset_estop\" to continue.");

  return true;
}

bool HekaterosControl::freeze(Freeze::Request  &req,
                              Freeze::Response &rsp)
{
  const char *svc = "freeze";

  ROS_DEBUG("%s", svc);

  m_robot.freeze();

  ROS_INFO("Robot position frozen.");

  return true;
}

bool HekaterosControl::getProductInfo(GetProductInfo::Request  &req,
                                      GetProductInfo::Response &rsp)
{
  const char *svc = "get_product_info";

  int   nMajor, nMinor, nRev;
  char  s[128];

  ROS_DEBUG("%s", svc);

  if( !m_robot.isDescribed() )
  {
    ROS_ERROR("%s failed: "
              "Robot description not loaded - unable to determine info.",
              svc);
    return false;
  }

  m_robot.getVersion(nMajor, nMinor, nRev);

  rsp.i.maj             = nMajor;
  rsp.i.min             = nMinor;
  rsp.i.rev             = nRev;
  rsp.i.version_string  = m_robot.getVersion();
  rsp.i.product_id      = m_robot.getProdId();
  rsp.i.product_name    = m_robot.getProdName();
  rsp.i.desc            = m_robot.getFullProdBrief();

  if( gethostname(s, sizeof(s)) < 0 )
  {
    strcpy(s, "hekateros");
  }
  s[sizeof(s)-1] = 0;

  rsp.i.hostname  = s;

  return true;
}

bool HekaterosControl::gotoBalancedPos(GotoBalancedPos::Request  &req,
                                       GotoBalancedPos::Response &rsp)
{
  const char *svc = "goto_balanced";

  ROS_DEBUG("%s", svc);

  rsp.rc = m_robot.gotoBalancedPos();

  if( rsp.rc == HEK_OK )
  {
    ROS_INFO("Moving Hekateros to balanced position.");
    return true;
  }
  else
  {
    ROS_ERROR("%s failed. %s(rc=%d).", svc, getStrError(rsp.rc), rsp.rc);
    return false;
  }
}

bool HekaterosControl::gotoParkedPos(GotoParkedPos::Request  &req,
                                     GotoParkedPos::Response &rsp)
{
  const char *svc = "goto_zero";

  ROS_DEBUG("%s", svc);

  rsp.rc = m_robot.gotoParkedPos();

  if( rsp.rc == HEK_OK )
  {
    ROS_INFO("Moving Hekateros to parked position.");
    return true;
  }
  else
  {
    ROS_ERROR("%s failed. %s(rc=%d).", svc, getStrError(rsp.rc), rsp.rc);
    return false;
  }
}

bool HekaterosControl::gotoZeroPt(GotoZeroPt::Request  &req,
                                  GotoZeroPt::Response &rsp)
{
  const char *svc = "goto_zero";

  ROS_DEBUG("%s", svc);

  rsp.rc = m_robot.gotoZeroPtPos();

  if( rsp.rc == HEK_OK )
  {
    ROS_INFO("Moving Hekateros to calibrated zero point.");
    return true;
  }
  else
  {
    ROS_ERROR("%s failed. %s(rc=%d).", svc, getStrError(rsp.rc), rsp.rc);
    return false;
  }
}

bool HekaterosControl::isAlarmed(IsAlarmed::Request  &req,
                                 IsAlarmed::Response &rsp)
{
  const char *svc = "is_alarmed";

  ROS_DEBUG("%s", svc);

  rsp.is_alarmed = m_robot.isAlarmed();

  if( rsp.is_alarmed )
  {
    ROS_WARN("Hekateros is alarmed.");
  }

  return true;
}

bool HekaterosControl::isCalibrated(IsCalibrated::Request  &req,
                                    IsCalibrated::Response &rsp)
{
  const char *svc = "is_calibrated";

  ROS_DEBUG("%s", svc);

  rsp.is_calibrated = m_robot.isCalibrated();

  if( !rsp.is_calibrated )
  {
    ROS_WARN("Hekateros is not calibrated.");
  }

  return true;
}

bool HekaterosControl::isDescLoaded(IsDescLoaded::Request  &req,
                                    IsDescLoaded::Response &rsp)
{
  const char *svc = "is_desc_loaded";

  ROS_DEBUG("%s", svc);

  rsp.is_desc_loaded = m_robot.isDescribed();

  if( !rsp.is_desc_loaded )
  {
    ROS_WARN("Hekateros description file not loaded.");
  }

  return true;
}

bool HekaterosControl::openGripper(OpenGripper::Request  &req,
                                   OpenGripper::Response &rsp)
{
  const char *svc = "open_gripper";
  int         rc;

  ROS_DEBUG("%s", svc);

  rc = m_robot.openGripper();

  if( rc == HEK_OK )
  {
    ROS_INFO("Opening gripper.");
    return true;
  }
  else
  {
    ROS_ERROR("%s failed. %s(rc=%d).", svc, getStrError(rc), rc);
    return false;
  }
}

bool HekaterosControl::readGpio(ReadGpio::Request  &req,
                                ReadGpio::Response &rsp)
{
  const char *svc = "read_gpio";
  int         rc;

  ROS_DEBUG("%s", svc);

  // RDK rc = m_robot.x();

  if( rc == HEK_OK )
  {
    ROS_INFO("Read GPIO.");
    return true;
  }
  else
  {
    ROS_ERROR("%s failed. %s(rc=%d).", svc, getStrError(rc), rc);
    return false;
  }
}

bool HekaterosControl::release(Release::Request  &req,
                               Release::Response &rsp)
{
  const char *svc = "release";

  ROS_DEBUG("%s", svc);

  m_robot.release();

  ROS_INFO("Robot released, motors are undriven.");

  return true;
}

bool HekaterosControl::reloadConfig(Release::Request  &req,
                                    Release::Response &rsp)
{
  const char *svc = "reload_config";

  ROS_DEBUG("%s", svc);

  m_robot.reload();

  ROS_INFO("Robot configuration reloaded.");

  return true;
}

bool HekaterosControl::resetEStop(ResetEStop::Request  &req,
                                  ResetEStop::Response &rsp)
{
  const char *svc = "reset_estop";

  ROS_DEBUG("%s", svc);

  m_robot.resetEStop();

  ROS_INFO("EStop reset.");

  return true;
}

bool HekaterosControl::setRobotMode(SetRobotMode::Request  &req,
                                    SetRobotMode::Response &rsp)
{
  const char *svc = "set_robot_mode";

  ROS_DEBUG("%s", svc);

  m_robot.setRobotMode((HekRobotMode)req.mode.val);

  ROS_INFO("Robot mode set to %d.", req.mode.val);

  return true;
}

bool HekaterosControl::stop(Stop::Request  &req,
                            Stop::Response &rsp)
{
  const char *svc = "stop";

  int n;

  ROS_DEBUG("%s", svc);

  ROS_INFO("Stop");

  for(int i=0; i<req.joint_names.size(); ++i)
  {
    ROS_INFO(" %-12s", req.joint_names[i].c_str());
  }

  n = m_robot.stop(req.joint_names);

  ROS_INFO("Stopped %d joints.", n);

  return true;
}

bool HekaterosControl::writeGpio(WriteGpio::Request  &req,
                                 WriteGpio::Response &rsp)
{
  const char *svc = "write_gpio";
  int         rc;

  ROS_DEBUG("%s", svc);

  // RDK rc = m_robot.x();

  if( rc == HEK_OK )
  {
    ROS_INFO("Wrote GPIO.");
    return true;
  }
  else
  {
    ROS_ERROR("%s failed. %s(rc=%d).", svc, getStrError(rc), rc);
    return false;
  }
}


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Topic Publishers
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

void HekaterosControl::advertisePublishers(int nQueueDepth)
{
  string  strPub;

  strPub = "joint_states";
  m_publishers[strPub] =
    m_nh.advertise<sensor_msgs::JointState>(strPub, nQueueDepth);

  strPub = "joint_states_ex";
  m_publishers[strPub] =
    m_nh.advertise<HekJointStateExtended>(strPub,nQueueDepth);

  strPub = "robot_status";
  m_publishers[strPub] =
    m_nh.advertise<industrial_msgs::RobotStatus>(strPub, nQueueDepth);

  strPub = "robot_status_ex";
  m_publishers[strPub] =
    m_nh.advertise<HekRobotStatusExtended>(strPub, nQueueDepth);
}

void HekaterosControl::publish()
{
  publishJointState();
  publishRobotStatus();
}

void HekaterosControl::publishJointState()
{
  HekJointStatePoint    state;

  // get robot's extended joint state.
  m_robot.getJointState(state);
  
  // update joint state message
  updateJointStateMsg(state, m_msgJointState);

  // publish joint state messages
  m_publishers["joint_states"].publish(m_msgJointState);

  // update extended joint state message
  updateExtendedJointStateMsg(state, m_msgJointStateEx);

  // publish extened joint state messages
  m_publishers["joint_states_ex"].publish(m_msgJointStateEx);
}

void HekaterosControl::publishRobotStatus()
{
  HekRobotState   status;   // really status 

  // get robot's extended status.
  m_robot.getRobotState(status);

  // update robot status message
  updateRobotStatusMsg(status, m_msgRobotStatus);

  // publish robot status message
  m_publishers["robot_status"].publish(m_msgRobotStatus);

  // update extended robot status message
  updateExtendedRobotStatusMsg(status, m_msgRobotStatusEx);

  // publish extened robot status message
  m_publishers["robot_status_ex"].publish(m_msgRobotStatusEx);
}

void HekaterosControl::updateJointStateMsg(HekJointStatePoint      &state,
                                           sensor_msgs::JointState &msg)
{
  //
  // Clear previous joint state data.
  //
  msg.name.clear();
  msg.position.clear();
  msg.velocity.clear();
  msg.effort.clear();

  //
  // Set joint state header.
  //
  msg.header.stamp    = ros::Time::now();
  msg.header.frame_id = "0";
  msg.header.seq++;

  //
  // Set joint state state values;
  //
  for(int n=0; n<state.getNumPoints(); ++n)
  {
    // joint state
    msg.name.push_back(state[n].m_strName);
    msg.position.push_back(state[n].m_fPosition);
    msg.velocity.push_back(state[n].m_fVelocity);
    msg.effort.push_back(state[n].m_fEffort);
  }
}

void HekaterosControl::updateExtendedJointStateMsg(HekJointStatePoint &state,
                                                   HekJointStateExtended &msg)
{
  HekOpState  opstate;

  // 
  // Clear previous extended joint state data.
  //
  msg.name.clear();
  msg.position.clear();
  msg.velocity.clear();
  msg.effort.clear();
  msg.master_servo_id.clear();
  msg.slave_servo_id.clear();
  msg.odometer_pos.clear();
  msg.encoder_pos.clear();
  msg.raw_speed.clear();
  msg.op_state.clear();

  //
  // Set extended joint state header.
  //
  msg.header.stamp    = ros::Time::now();
  msg.header.frame_id = "0";
  msg.header.seq++;

  //
  // Set extended joint state values;
  //
  for(int n=0; n<state.getNumPoints(); ++n)
  {
    msg.name.push_back(state[n].m_strName);
    msg.position.push_back(state[n].m_fPosition);
    msg.velocity.push_back(state[n].m_fVelocity);
    msg.effort.push_back(state[n].m_fEffort);
    msg.master_servo_id.push_back(state[n].m_nMasterServoId);
    msg.slave_servo_id.push_back(state[n].m_nSlaveServoId);
    msg.odometer_pos.push_back(state[n].m_nOdPos);
    msg.encoder_pos.push_back(state[n].m_nEncPos);
    msg.raw_speed.push_back(state[n].m_nSpeed);

    opstate.calib_state = state[n].m_eOpState;
    msg.op_state.push_back(opstate);
  }
}

void HekaterosControl::updateRobotStatusMsg(HekRobotState &status,
                                            industrial_msgs::RobotStatus &msg)
{
  //
  // Set robot status header.
  //
  msg.header.stamp    = ros::Time::now();
  msg.header.frame_id = "0";
  msg.header.seq++;

  //
  // Set industrial message compliant robot status values.
  //
  msg.mode.val            = status.m_eRobotMode;
  msg.e_stopped.val       = status.m_eIsEStopped;
  msg.drives_powered.val  = status.m_eAreDrivesPowered;
  msg.motion_possible.val = status.m_eIsMotionPossible;
  msg.in_motion.val       = status.m_eIsInMotion;
  msg.in_error.val        = status.m_eIsInError;
  msg.error_code          = status.m_nErrorCode;

}

void HekaterosControl::updateExtendedRobotStatusMsg(HekRobotState &status,
                                                    HekRobotStatusExtended &msg)
{
  ServoHealth sh;
  int         i;

  //
  // Set extended robot status header.
  //
  msg.header.stamp    = ros::Time::now();
  msg.header.frame_id = "0";
  msg.header.seq++;

  //
  // Set hekateros message extended robot status values.
  //
  msg.mode.val            = status.m_eRobotMode;
  msg.e_stopped.val       = status.m_eIsEStopped;
  msg.drives_powered.val  = status.m_eAreDrivesPowered;
  msg.motion_possible.val = status.m_eIsMotionPossible;
  msg.in_motion.val       = status.m_eIsInMotion;
  msg.in_error.val        = status.m_eIsInError;
  msg.error_code          = status.m_nErrorCode;
  msg.is_calibrated.val   = status.m_eIsCalibrated;

  // clear previous data
  msg.servo_health.clear();

  for(i=0; i<status.m_vecServoHealth.size(); ++i)
  {
    sh.servo_id = (s8_t)status.m_vecServoHealth[i].m_nServoId;
    sh.temp     = status.m_vecServoHealth[i].m_fTemperature;
    sh.voltage  = status.m_vecServoHealth[i].m_fVoltage;
    sh.alarm    = (u8_t)status.m_vecServoHealth[i].m_uAlarms;

    msg.servo_health.push_back(sh);
  }
}


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Subscribed Topics
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

void HekaterosControl::subscribeToTopics(int nQueueDepth)
{
  string  strSub;

  strSub = "joint_command";
  m_subscriptions[strSub] = m_nh.subscribe(strSub, nQueueDepth,
                                          &HekaterosControl::execJointCmd,
                                          &(*this));
}

void HekaterosControl::execJointCmd(const trajectory_msgs::JointTrajectory &jt)
{
  ROS_DEBUG("Executing joint_command.");

  HekJointTrajectoryPoint pt;

  ROS_INFO("Move");

  // load trajectory point
  for(int j=0; j<jt.joint_names.size(); ++j)
  {
    pt.append(jt.joint_names[j],
              jt.points[0].positions[j], 
              jt.points[0].velocities[j]);
    ROS_INFO(" %-12s: pos=%7.3lf(%6.2lfdeg) vel=%7.3lf(%6.2lfdeg/s)",
        jt.joint_names[j].c_str(), 
        jt.points[0].positions[j], 
        radToDeg(jt.points[0].positions[j]), 
        jt.points[0].velocities[j],
        radToDeg(jt.points[0].velocities[j]));
  }

  m_robot.moveArm(pt);
}
