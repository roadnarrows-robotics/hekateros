////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics Hekateros Robotiic Manipulator ROS Package
//
// Link:      https://github.com/roadnarrows-robotics/hekateros
//
// ROS Node:  hekateros_control
//
// File:      hekateros_as_follow_traj.cpp
//
/*! \file
 *
 * $LastChangedDate$
 * $Rev$
 *
 * \brief Hekateros follow joint trajectory action server class implementation.
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
#include <sys/types.h>
#include <unistd.h>
#include <math.h>
#include <boost/bind.hpp>
#include <sstream>
#include <string>

//
// ROS
//
#include "ros/ros.h"

//
// ROS generated core, industrial, and hekateros messages.
//
#include "std_msgs/String.h"
#include "control_msgs/FollowJointTrajectoryAction.h"

//
// ROS generated action servers.
//
#include "actionlib/server/simple_action_server.h"

//
// RoadNarrows embedded hekateros library.
//
#include "Hekateros/hekateros.h"
#include "Hekateros/hekJoint.h"
#include "Hekateros/hekTraj.h"
#include "Hekateros/hekRobot.h"

//
// Node headers.
//
#include "hekateros_control.h"
#include "hekateros_as_follow_traj.h"


using namespace std;
using namespace hekateros;
using namespace control_msgs;
using namespace trajectory_msgs;
using namespace hekateros_control;

//
// Tuning
//
const double ASFollowTrajectory::TolWaypoint = 0.01745; // ~1.0 degrees
const double ASFollowTrajectory::TolGoal     = 0.00087; // ~0.05 degrees
const int    ASFollowTrajectory::MaxIters    = 10;      // ~1 second

void ASFollowTrajectory::execute_cb(
                                  const FollowJointTrajectoryGoalConstPtr &goal)
{
  JointTrajectory   jt;         // the joint trajectory path
  ssize_t           iWaypoint;  // working waypoint index
  int               rc;         // return code

  ROS_INFO("%s: Executing FollowJointTrajectory goal of %zu waypoints.",
      action_name_.c_str(), goal->trajectory.points.size());

  //
  // Initialize.
  //

  // The joint trajectory path.
  jt = goal->trajectory;

  // No path. Is this an error or a null success?
  if( jt.points.size() == 0 )
  {
    ROS_ERROR("%s: No joint trajectory path.", action_name_.c_str());
    result_.error_code = FollowJointTrajectoryResult::INVALID_GOAL;
    as_.setAborted(result_);
    return;
  }

  // Fixed number of joints in fixed order per each way point, so preload names.
  feedback_.joint_names = jt.joint_names;

  iWaypoint = -1;

  // control, monitor, and feedback rate
  ros::Rate ctl_rate(10);

  //
  // Follow joint trajectory path.
  //
  while( (iWaypoint < jt.points.size()) && ros::ok() )
  {
    //
    // Action was preempted.
    //
    if( as_.isPreemptRequested() )
    {
      ROS_INFO("%s: Action execution preempted on waypoint %zd.",
          action_name_.c_str(), iWaypoint);
      result_.error_code = FollowJointTrajectoryResult::INVALID_GOAL;
      as_.setPreempted(result_);
      return;
    }

    //
    // At start/current waypoint. Now move to new waypoint.
    //
    else if( (iWaypoint == -1) || atWaypoint(jt, iWaypoint) )
    {
      ++iWaypoint;  // index of next waypoint

      // no more waypoints
      if( iWaypoint >= jt.points.size() )
      {
        break;
      }

      if( (rc = moveToWaypoint(jt, iWaypoint)) == HEK_OK )
      {
        ROS_INFO("%s: Moving to trajectory waypoint %zd.",
            action_name_.c_str(), iWaypoint);
        m_fTolerance  = iWaypoint < jt.points.size()-1? TolWaypoint: TolGoal;
        m_iterMonitor = 0;
        feedback_.desired = jt.points[iWaypoint];
      }
      else
      {
        ROS_ERROR("%s: Failed moving arm to trajectory waypoint %zd.",
              action_name_.c_str(), iWaypoint);
        result_.error_code = FollowJointTrajectoryResult::INVALID_JOINTS;
        as_.setAborted(result_);
        return;
      }
    }

    //
    // Failed to reach waypoint. Abort.
    //
    else if ( failedWaypoint(jt, iWaypoint) )
    {
      ROS_ERROR("%s: Failed to reach waypoint %zd.",
          action_name_.c_str(), iWaypoint);
      result_.error_code = FollowJointTrajectoryResult::INVALID_GOAL;
      as_.setAborted(result_);
      return;
    }

    //
    // Monitor movement and provide feedback.
    //
    monitorMove(jt, iWaypoint);

    ctl_rate.sleep();
  }

  //
  // Success.
  //
  if( ros::ok() )
  {
    ROS_INFO("%s: Follow trajectory succeeded.", action_name_.c_str());
    result_.error_code = FollowJointTrajectoryResult::SUCCESSFUL;
    as_.setSucceeded(result_);
  }

  //
  // Odd failure.
  //
  else
  {
    ROS_ERROR("%s: Follow trajectory failed.", action_name_.c_str());
    result_.error_code = FollowJointTrajectoryResult::GOAL_TOLERANCE_VIOLATED;
    as_.setAborted(result_);
  }
}

int ASFollowTrajectory::moveToWaypoint(JointTrajectory &jt, ssize_t iWaypoint)
{
  HekRobot                &robot(node_.getRobot());  // hekateros
  HekJointTrajectoryPoint  pt;                       // hekateros joint point
  int                      j;                        // working index

  //
  // Load next trajectory point.
  //
  for(j=0; j<jt.joint_names.size(); ++j)
  {
    pt.append(jt.joint_names[j],
              jt.points[iWaypoint].positions[j], 
              jt.points[iWaypoint].velocities[j]);
    ROS_DEBUG("%s: pos=%5.3f speed=%2.1f",  jt.joint_names[j].c_str(), 
                                            jt.points[iWaypoint].positions[j], 
                                            jt.points[iWaypoint].velocities[j]);
  }

  //
  // Start moving the arm to the trajectory waypoint.
  //
  return robot.moveArm(pt);
}

void ASFollowTrajectory::monitorMove(JointTrajectory &jt, ssize_t iWaypoint)
{
  HekRobot           &robot(node_.getRobot());  // hekateros
  HekJointStatePoint  jointCurState;            // current joint state point
  string              jointName;                // joint name
  double              jointWpPos;               // joint waypoint position
  double              jointWpVel;               // joint waypoint velocity
  double              jointCurPos;              // joint current position
  double              jointCurVel;              // joint current velocity
  double              jointDist;                // joint distance
  double              waypointDist;             // current waypoint distance
  int                 j;                        // working index

  robot.getJointState(jointCurState);

  m_fWaypointDist = 1000.0;    // make large
  waypointDist    = 0.0;        

  // 
  // Calculate distance from current position to target waypoint.
  //
  // Distance metric: Linf of joint positions.
  // Alternerates:    L1 or L2 of joint positions, or zero point (end effector)
  //                  attachment point) Euclidean distance.
  //
  for(j=0; j<jt.joint_names.size(); ++j)
  {
    jointName   = jt.joint_names[j];
    jointWpPos  = jt.points[iWaypoint].positions[j]; 
    jointWpVel  = jt.points[iWaypoint].velocities[j]; 

    if( jointCurState.hasJoint(jointName) )
    {
      jointCurPos = jointCurState[jointName].m_fPosition;
      jointCurVel = jointCurState[jointName].m_fVelocity;

      jointDist = fabs(jointWpPos - jointCurPos);

      if( jointDist > waypointDist )
      {
        waypointDist = jointDist;
      }

      feedback_.actual.positions[j]  = jointCurPos;
      feedback_.actual.velocities[j] = jointCurVel;
      feedback_.error.positions[j]   = jointDist;
      feedback_.error.velocities[j]  = jointWpVel - jointCurVel;
    }
    else
    {
      feedback_.actual.positions[j]  = jointWpPos;
      feedback_.actual.velocities[j] = jointWpVel;
      feedback_.error.positions[j]   = 0.0;
      feedback_.error.velocities[j]  = 0.0;
    }
  }

  publishFeedback(iWaypoint);

  m_fWaypointDist = waypointDist;

  ++m_iterMonitor;
}


bool ASFollowTrajectory::failedWaypoint(JointTrajectory &jt, ssize_t iWaypoint)
{
  HekRobot           &robot(node_.getRobot());  // hekateros

  if( robot.isEStopped() || robot.isAlarmed() || !robot.areServosPowered() )
  {
    return true;
  }

  else if( atWaypoint(jt, iWaypoint) )
  {
    return false;
  }

  else if( (m_iterMonitor > MaxIters) && !robot.isInMotion() )
  {
    return true;
  }

  else
  {
    return false;
  }
}

void ASFollowTrajectory::publishFeedback(ssize_t iWaypoint)
{
  stringstream ss;

  ss << iWaypoint;

  //
  // Feedback header.
  //
  feedback_.header.stamp    = ros::Time::now();
  feedback_.header.frame_id = ss.str();
  feedback_.header.seq++;

  as_.publishFeedback(feedback_);
}

void ASFollowTrajectory::preempt_cb()
{
  ROS_INFO("%s: Preempt trajectory following.", action_name_.c_str());
  node_.getRobot().freeze();
}
