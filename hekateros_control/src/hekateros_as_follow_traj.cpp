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
#include "Hekateros/hekUtils.h"
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


void ASFollowTrajectory::execute_cb(const FollowJointTrajectoryGoalConstPtr&
                                                                          goal)
{
  double    fHz = 10.0;   // feedback fequency
  size_t    numWaypoints; // number of  waypoints
  size_t    iEndpoint;    // end point (last point) index
  size_t    iWaypoint;    // working waypoint index

  ROS_INFO("%s: Executing FollowJointTrajectory goal of %zu waypoints.",
      action_name_.c_str(), goal->trajectory.points.size());

  // get current trajectory parameters (can change, so alwas refetch) 
  m_robot.getTrajectoryParams(m_eNorm, m_fEpsilon);

  // the joint goal trajectory path
  m_traj = goal->trajectory;

  // number of waypoints
  numWaypoints = m_traj.points.size();

  // No path. Is this an error or a null success?
  if( numWaypoints == 0 )
  {
    ROS_ERROR("%s: No joint trajectory path.", action_name_.c_str());
    result_.error_code = FollowJointTrajectoryResult::INVALID_GOAL;
    as_.setAborted(result_);
    return;
  }

  m_nMaxIters       = (int)ceil(fHz * MaxSecs); // max iters to a waypoint
  m_eState          = ExecStateStartMove;   // initial execution state
  m_bTrajCompleted  = false;                // trajectory not completed to end
  iEndpoint         = numWaypoints - 1;     // end point index

  // control, monitor, and feedback rate
  ros::Rate ctl_rate(fHz);

  for(iWaypoint = 0;
      (iWaypoint < numWaypoints) && (m_eState != ExecStateTerminate);
      ++iWaypoint)
  {
    if( !ros::ok() )
    {
      break;
    }

    switch( m_eState )
    {
      case ExecStateStartMove:
        m_eState = startMoveToPoint(iWaypoint);
        break;
      case ExecStateMonitorMove:
        if( iWaypoint < iEndpoint )
        {
          m_eState = monitorMoveToWaypoint(iWaypoint);
        }
        else
        {
          m_eState = monitorMoveToEndpoint(iWaypoint);
        }
        break;
      case ExecStateTerminate:
        break;
    }

    ctl_rate.sleep();
  }

  //
  // No more ROS services, simply freeze the robot.
  //
  if( !ros::ok() )
  {
    m_robot.freeze();
  }

  //
  // Success.
  //
  else if( m_bTrajCompleted )
  {
    ROS_INFO("%s: Follow trajectory succeeded.", action_name_.c_str());
    result_.error_code = FollowJointTrajectoryResult::SUCCESSFUL;
    as_.setSucceeded(result_);
  }

  //
  // Failure. (Result and state set at point of error.)
  //
  else
  {
    ROS_ERROR("%s: Follow trajectory failed.", action_name_.c_str());
    m_robot.freeze();
  }
}

ASFollowTrajectory::ExecState
                        ASFollowTrajectory::startMoveToPoint(size_t iWaypoint)
{
  HekJointTrajectoryPoint pt;   // hekateros joint point
  int                     j;    // working index
  int                     rc;   // return code

  //
  // Load next trajectory point.
  //
  for(j=0; j<m_traj.joint_names.size(); ++j)
  {
    pt.append(m_traj.joint_names[j],
              m_traj.points[iWaypoint].positions[j], 
              m_traj.points[iWaypoint].velocities[j]);

    ROS_DEBUG("%s: pos=%6.3lf(%6.2lf deg) vel=%6.3lf(%6.1lf deg/sec)",
      m_traj.joint_names[j].c_str(), 
      m_traj.points[iWaypoint].positions[j], 
      radToDeg(m_traj.points[iWaypoint].positions[j]), 
      m_traj.points[iWaypoint].velocities[j],
      radToDeg(m_traj.points[iWaypoint].velocities[j]));
  }

  //
  // Start moving the arm to the trajectory waypoint.
  //
  rc = m_robot.moveArm(pt);

  if( rc == HEK_OK )
  {
    ROS_INFO("%s: Moving to trajectory waypoint %zu.",
            action_name_.c_str(), iWaypoint);
    m_iterMonitor = 0;
    return ExecStateMonitorMove;
  }
  else
  {
    ROS_ERROR("%s: Failed moving arm to trajectory waypoint %zu.",
              action_name_.c_str(), iWaypoint);
    result_.error_code = FollowJointTrajectoryResult::INVALID_JOINTS;
    as_.setAborted(result_);
    return ExecStateTerminate;
  }
}

ASFollowTrajectory::ExecState
                    ASFollowTrajectory::monitorMoveToWaypoint(size_t iWaypoint)
{
  double fDist;

  //
  // Action was preempted.
  //
  if( as_.isPreemptRequested() )
  {
    ROS_INFO("%s: Action execution preempted on waypoint %zu.",
          action_name_.c_str(), iWaypoint);
    result_.error_code = FollowJointTrajectoryResult::INVALID_GOAL;
    as_.setPreempted(result_);
    return ExecStateTerminate;
  }

  //
  // Failed to reach waypoint.
  //
  if( failedWaypoint() )
  {
    ROS_ERROR("%s: Failed to reach waypoint %zu.",
          action_name_.c_str(), iWaypoint);
    result_.error_code = FollowJointTrajectoryResult::INVALID_GOAL;
    as_.setAborted(result_);
    return ExecStateTerminate;
  }

  // measure move and publish feedback
  fDist = measureMove(iWaypoint);

  //
  // Successfully reach waypoint.
  //
  if( fabs(fDist) < m_fEpsilon )
  {
    ROS_DEBUG("Reached waypoint %zu in %d iterations, dist=%lf (%lf deg).",
      iWaypoint, m_iterMonitor, fDist, radToDeg(fDist));
    return ExecStateStartMove;
  }

  //
  // Keep monitoring.
  //
  ++m_iterMonitor;

  return ExecStateMonitorMove;
}

ASFollowTrajectory::ExecState
                    ASFollowTrajectory::monitorMoveToEndpoint(size_t iWaypoint)
{
  //
  // Action was preempted.
  //
  if( as_.isPreemptRequested() )
  {
    ROS_INFO("%s: Action execution preempted on waypoint %zu.",
          action_name_.c_str(), iWaypoint);
    result_.error_code = FollowJointTrajectoryResult::INVALID_GOAL;
    as_.setPreempted(result_);
    return ExecStateTerminate;
  }

  //
  // Failed to reach waypoint.
  //
  if( failedWaypoint() )
  {
    ROS_ERROR("%s: Failed to reach waypoint %zu.",
          action_name_.c_str(), iWaypoint);
    result_.error_code = FollowJointTrajectoryResult::INVALID_GOAL;
    as_.setAborted(result_);
    return ExecStateTerminate;
  }

  //
  // Successfully reach waypoint.
  //
  if( !m_robot.isInMotion() )
  {
    ROS_DEBUG("Reached endpoint %zu in %d iterations.",
      iWaypoint, m_iterMonitor);
    m_bTrajCompleted = true;
    return ExecStateTerminate;
  }

  //
  // Keep monitoring.
  //
  ++m_iterMonitor;

  return ExecStateMonitorMove;
}

double ASFollowTrajectory::measureMove(size_t iWaypoint)
{
  HekJointStatePoint    jointCurState;  // current joint state point
  string                jointName;      // joint name
  double                jointWpPos;     // joint waypoint position (radians)
  double                jointWpVel;     // joint waypoint velocity (radians/sec)
  double                jointCurPos;    // joint current position (radians)
  double                jointCurVel;    // joint current velocity (radians/sec)
  double                fWaypointDist;  // joint current velocity (radians/sec)
  size_t                j;              // working index

  clearFeedback();

  m_robot.getJointState(jointCurState);

  fWaypointDist = 0.0;

  // 
  // Calculate distance from current position to target waypoint.
  //
  // Distance metric: Linf of joint positions.
  // Alternerates:    L1 or L2 of joint positions, or zero point (end effector)
  //                  attachment point) Euclidean distance.
  //
  for(j=0; j<m_traj.joint_names.size(); ++j)
  {
    jointName   = m_traj.joint_names[j];
    jointWpPos  = m_traj.points[iWaypoint].positions[j]; 
    jointWpVel  = m_traj.points[iWaypoint].velocities[j]; 
    jointCurPos = jointWpPos;
    jointCurVel = jointWpVel;

    if( jointCurState.hasJoint(jointName) )
    {
      jointCurPos = jointCurState[jointName].m_fPosition;
      jointCurVel = jointCurState[jointName].m_fVelocity;

      switch( m_eNorm )
      {
        case HekNormL2:
          fWaypointDist += pow(jointWpPos - jointCurPos, 2.0);
          break;
        case HekNormL1:
          fWaypointDist += fabs(jointWpPos - jointCurPos);
          break;
        case HekNormLinf:
          double delta = fabs(jointWpPos - jointCurPos);
          if( delta > fWaypointDist )
          {
            fWaypointDist = delta;
          }
          break;
      }
    }

    // add point to feedback
    addFeedbackPoint(jointName,
                     jointWpPos,  jointWpVel,
                     jointCurPos, jointCurVel);
  }

  publishFeedback(iWaypoint);

  if( m_eNorm == HekNormL2 )
  {
    fWaypointDist = sqrt(fWaypointDist);
  }

  return fWaypointDist;
}

bool ASFollowTrajectory::failedWaypoint()
{
  return  m_robot.isEStopped() ||
          m_robot.isAlarmed() ||
         !m_robot.areServosPowered() ||
         (m_iterMonitor > m_nMaxIters);
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

void ASFollowTrajectory::clearFeedback()
{
  feedback_.joint_names.clear();
  feedback_.desired.positions.clear();
  feedback_.desired.velocities.clear();
  feedback_.desired.accelerations.clear();
  feedback_.actual.positions.clear();
  feedback_.actual.velocities.clear();
  feedback_.actual.accelerations.clear();
  feedback_.error.positions.clear();
  feedback_.error.velocities.clear();
  feedback_.error.accelerations.clear();
}

void ASFollowTrajectory::addFeedbackPoint(const string &jointName,
                                          const double  jointWpPos,
                                          const double  jointWpVel,
                                          const double  jointCurPos,
                                          const double  jointCurVel)
{
  double jointErrPos;    // joint position delta (radians)
  double jointErrVel;    // joint velocity delta (radians/sec)


  jointErrPos = jointWpPos - jointCurPos;
  jointErrVel = jointWpVel - jointCurVel;

  feedback_.joint_names.push_back(jointName);
  feedback_.desired.positions.push_back(jointWpPos);
  feedback_.desired.velocities.push_back(jointWpVel);
  feedback_.desired.accelerations.push_back(0.0);
  feedback_.actual.positions.push_back(jointCurPos);
  feedback_.actual.velocities.push_back(jointCurVel);
  feedback_.actual.accelerations.push_back(0.0);
  feedback_.error.positions.push_back(jointErrPos);
  feedback_.error.velocities.push_back(jointErrVel);
  feedback_.error.accelerations.push_back(0.0);
}

void ASFollowTrajectory::preempt_cb()
{
  ROS_INFO("%s: Preempt trajectory following.", action_name_.c_str());
  m_robot.freeze();
}
