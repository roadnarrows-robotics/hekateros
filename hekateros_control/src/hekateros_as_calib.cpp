////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics Hekateros Robotiic Manipulator ROS Package
//
// Link:      https://github.com/roadnarrows-robotics/hekateros
//
// ROS Node:  hekateros_control
//
// File:      hekateros_as_calib.cpp
//
/*! \file
 *
 * $LastChangedDate$
 * $Rev$
 *
 * \brief Hekateros calibration action server class implementation.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 * \author Danial Packard (daniel@roadnarrows.com)
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
// System and Boost
//
#include <unistd.h>
#include <boost/bind.hpp>

//
// ROS
//
#include "ros/ros.h"

//
// ROS generated core, industrial, and hekateros messages.
//
#include "std_msgs/String.h"
#include "hekateros_control/HekOpState.h"
#include "hekateros_control/HekJointStateExtended.h"

//
// ROS generated action servers.
//
#include "actionlib/server/simple_action_server.h"
#include "hekateros_control/CalibrateAction.h"

//
// RoadNarrows embedded hekateros library.
//
#include "Hekateros/hekateros.h"
#include "Hekateros/hekJoint.h"
#include "Hekateros/hekRobot.h"

//
// Node headers.
//
#include "hekateros_control.h"
#include "hekateros_as_calib.h"


using namespace std;
using namespace hekateros;
using namespace hekateros_control;


void ASCalibrate::execute_cb(const CalibrateGoalConstPtr &goal)
{
  HekRobot           &robot(node_.getRobot());
  HekJointStatePoint  state;
  int                 rc;

  ROS_INFO("%s: Executing calibrate action - please standby.",
      action_name_.c_str());

  //
  // Start Hekateros to calibrate asynchronously (i.e. non-blocking).
  //
  rc = robot.calibrateAsync(goal->force_recalib? true: false);

  if( rc != HEK_OK )
  {
    ROS_ERROR("%s: Failed to initiate calibration.", action_name_.c_str());
    result_.op.calib_state = HekOpState::UNCALIBRATED;
    as_.setAborted(result_);
    return;
  }

  ros::Rate r(2);

  while( robot.getAsyncState() == HekAsyncTaskStateWorking )
  {
    //
    // Action was preempted.
    //
    if( as_.isPreemptRequested() || !ros::ok() )
    {
      ROS_INFO("%s: Execution preempted.", action_name_.c_str());
      result_.op.calib_state = HekOpState::UNCALIBRATED;
      as_.setPreempted(result_); // set the action state to preempted
      return;
    }

    //
    // Keep providing feedback during calibration.
    //
    else
    {
      robot.getJointState(state);
      node_.updateExtendedJointStateMsg(state, feedback_.joint);
      as_.publishFeedback(feedback_);
      r.sleep();
    }
  }

  // check outcome of asynchronous task
  rc = robot.getAsyncRc();

  // got calibrated
  if( (rc == HEK_OK) && robot.isCalibrated() )
  {
    ROS_INFO("%s: Calibration succeeded.", action_name_.c_str());
    result_.op.calib_state = HekOpState::CALIBRATED;
    as_.setSucceeded(result_);
  }
  // preempted
  else if( as_.isPreemptRequested() || !ros::ok() )
  {
    ROS_INFO("%s: Calibration preempted.", action_name_.c_str());
    result_.op.calib_state = HekOpState::UNCALIBRATED;
    as_.setPreempted(result_); // set the action state to preempted
  }
  // failed
  else
  {
    ROS_ERROR("Calibration failed with error code %d.", rc);
    result_.op.calib_state = HekOpState::UNCALIBRATED;
    as_.setAborted(result_);
  }
}

void ASCalibrate::preempt_cb()
{
  ROS_INFO("%s: Preempt calibration.", action_name_.c_str());
  node_.getRobot().cancelAsyncTask();
}
