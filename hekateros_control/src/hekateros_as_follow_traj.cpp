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
 * \author Daniel Packard (daniel@roadnarrows.com)
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

/*!
 * \brief Convert norm enum to string name.
 *
 * \param eNorm   Norm.
 *
 * \return Name string.
 */
static const char *NormName(HekNorm eNorm)
{
  switch( eNorm )
  {
    case HekNormL1:
      return "L1";
    case HekNormL2:
      return "L2";
    case HekNormLinf:
      return "Linf";
    default:
      return "?";
  }
}

void ASFollowTrajectory::execute_cb(const FollowJointTrajectoryGoalConstPtr&
                                                                          goal)
{
  double    fHz = 10.0;   // feedback fequency
  double    fDistOrigin;  // distance from first waypoint to robot position
  ssize_t   iWaypoint;    // working waypoint index

  ROS_INFO("--- Executing FollowJointTrajectory goal of %zu waypoints. ---",
      goal->trajectory.points.size());

  // get current trajectory parameters (can change, so alwas refetch) 
  m_robot.getTrajectoryParams(m_eNorm, m_fEpsilon);

  ROS_INFO("  norm=%s, epsilon=%6.3lf(%7.2lf deg).",
      NormName(m_eNorm), m_fEpsilon, radToDeg(m_fEpsilon));

  // the joint goal trajectory path
  m_traj = goal->trajectory;

  // number of waypoints
  m_iNumWaypoints = (ssize_t)m_traj.points.size();

  // No path. Is this an error or a null success?
  if( m_iNumWaypoints == 0 )
  {
    ROS_ERROR("No joint trajectory path.");
    result_.error_code = FollowJointTrajectoryResult::INVALID_GOAL;
    as_.setAborted(result_);
    return;
  }

  // too far away
  else if( (fDistOrigin = measureDist(0)) > m_fEpsilon )
  {
    ROS_ERROR("Starting waypoint is not near the current robot position.");
    ROS_ERROR("%s distance from current position is %6.3lf(%7.2lf deg) > "
              "epsilon=%6.3lf(%7.2lf deg).",
              NormName(m_eNorm),
              fDistOrigin, radToDeg(fDistOrigin),
              m_fEpsilon,  radToDeg(m_fEpsilon));
    result_.error_code = FollowJointTrajectoryResult::INVALID_GOAL;
    as_.setAborted(result_);
    return;
  }

  m_nMaxIters       = (int)ceil(fHz * MaxSecs); // max iters to a waypoint
  m_eState          = ExecStateStartMove;   // initial execution state
  m_bTrajCompleted  = false;                // trajectory not completed to end
  m_iEndpoint       = m_iNumWaypoints - 1;  // endpoint index
  iWaypoint         = 0;                    // skip first waypoint

  // control, monitor, and feedback rate
  ros::Rate ctl_rate(fHz);

  while( ros::ok() &&
        (iWaypoint < m_iNumWaypoints) &&
        (m_eState != ExecStateTerminate) )
  {
    switch( m_eState )
    {
      // Start move to next waypoint.
      case ExecStateStartMove:
        iWaypoint     = nextWaypoint(iWaypoint);
        m_iterMonitor = 0;
        m_eState      = startMoveToPoint(iWaypoint);
        break;

      // Monitor move to waypoint and provide feedback.
      case ExecStateMonitorMove:
        if( iWaypoint < m_iEndpoint )
        {
          m_eState = monitorMoveToWaypoint(iWaypoint);
        }
        else
        {
          m_eState = monitorMoveToEndpoint(iWaypoint);
        }
        break;

      // Terminate trajectory action.
      case ExecStateTerminate:
        break;

      // Oops.
      default:
        ROS_ERROR("BUG: unknown state=%d.", m_eState);
        m_eState = ExecStateTerminate;
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
    ROS_INFO("* Follow trajectory succeeded.");
    result_.error_code = FollowJointTrajectoryResult::SUCCESSFUL;
    as_.setSucceeded(result_);
  }

  //
  // Failure.
  // Note: The result error code and state are set at the point of error.
  //
  else
  {
    ROS_ERROR("* Follow trajectory failed.");
    m_robot.freeze();
    as_.setAborted(result_);
  }
}

ssize_t ASFollowTrajectory::nextWaypoint(ssize_t iWaypoint)
{
  while( iWaypoint < m_iEndpoint )
  {
    ++iWaypoint;

    // waypoint sufficiently far away.
    if( measureDist(iWaypoint) >= m_fEpsilon )
    {
      break;
    }
    
    else if( iWaypoint < m_iEndpoint )
    {
      ROS_INFO("+ Skipping waypoint %zd of %zd.", iWaypoint, m_iNumWaypoints);
    }
  }

  return iWaypoint;
}

void ASFollowTrajectory::groomWaypoint(ssize_t iWaypoint)
{
  static double TuneNonZeroVel= degToRad(0.2);    // non-zero velocity threshold
  static double TuneMinVel    = degToRad(5.0);    // minimum absolute velocity
  static double TuneOptMaxVel = degToRad(60.0);   // optimal max abs velocity
  static double TuneMaxVel    = degToRad(150.0);  // maximum absolute velocity

  size_t  len;        // vector length
  size_t  j;          // working index
  double  v;          // [absolute] joint velocity
  double  fMinVel;    // V non-zero minimum element
  double  fMaxVel;    // V non-zero maximum element
  double  scale;      // V scaling

  len     = m_traj.joint_names.size();
  fMinVel = TuneMinVel;
  fMaxVel = 0.0;

  //
  // Characterize velocity V
  //
  for(j=0; j<len; ++j)
  {
    v = fabs(m_traj.points[iWaypoint].velocities[j]);

    // special endpoint case
    if( (iWaypoint == m_iEndpoint) && (iWaypoint > 0) && (v < TuneNonZeroVel) )
    {
      // copy velocity from previous waypoint
      m_traj.points[iWaypoint].velocities[j] =
                                    m_traj.points[iWaypoint-1].velocities[j];
      v = fabs(m_traj.points[iWaypoint].velocities[j]);
    }

    // test for new non-zero minimum and maximum
    if( v >= TuneNonZeroVel )
    {
      if( (v < fMinVel) )
      {
        fMinVel = v;
      }
      if( (v > fMaxVel) )
      {
        fMaxVel = v;
      }
    }
  }

  // 
  // Scale up V to acceptable robot optimal velocities.
  //
  if( (fMaxVel >= TuneNonZeroVel) && (fMaxVel < TuneOptMaxVel) )
  {
    scale = TuneOptMaxVel / fMaxVel;
    for(j=0; j<len; ++j)
    {
      m_traj.points[iWaypoint].velocities[j] *= scale;
    }
  }
  
  // 
  // Scale up V to acceptable robot minimum velocities.
  // Note: An alternative to scaling to optimal.
  //
  //if( fMinVel < TuneMinVel )
  //{
  //  scale = TuneMinVel / fMinVel;
  //  for(j=0; j<len; ++j)
  //  {
  //    m_traj.points[iWaypoint].velocities[j] *= scale;
  //  }
  //}
  
  //
  // Finally cap V velocities at a maximum. Note that this can lead to slight
  // trajectory distortions since the velocities in V are no longer in the
  // original proportions. Should be okay though.
  //
  for(j=0; j<len; ++j)
  {
    v = m_traj.points[iWaypoint].velocities[j];
    m_traj.points[iWaypoint].velocities[j] = fcap(v, -TuneMaxVel, TuneMaxVel);
  }
}

ASFollowTrajectory::ExecState ASFollowTrajectory::startMoveToPoint(
                                                        ssize_t iWaypoint)
{
  HekJointTrajectoryPoint pt;   // hekateros joint point
  size_t                  j;    // working index
  int                     rc;   // return code

  ROS_INFO("+ Waypoint %zd of %zd", iWaypoint, m_iNumWaypoints);

  //
  // Load trajectory point.
  //
  for(j=0; j<m_traj.joint_names.size(); ++j)
  {
    groomWaypoint(iWaypoint);

    pt.append(m_traj.joint_names[j],
              m_traj.points[iWaypoint].positions[j], 
              m_traj.points[iWaypoint].velocities[j]);

    // RDK really ROS_DEBUG
    ROS_INFO("    %-12s: pos=%6.3lf(%7.2lf deg) vel=%6.3lf(%6.1lf deg/sec)",
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
    ROS_INFO("  Moving to trajectory waypoint %zd.", iWaypoint);
    return ExecStateMonitorMove;
  }
  else
  {
    ROS_ERROR("Failed moving arm to trajectory waypoint %zd.", iWaypoint);
    result_.error_code = FollowJointTrajectoryResult::INVALID_JOINTS;
    as_.setAborted(result_);
    return ExecStateTerminate;
  }
}

ASFollowTrajectory::ExecState
                    ASFollowTrajectory::monitorMoveToWaypoint(ssize_t iWaypoint)
{
  double fDist;

  //
  // Action was preempted.
  //
  if( as_.isPreemptRequested() )
  {
    ROS_WARN("Action execution preempted on waypoint %zd.", iWaypoint);
    result_.error_code = FollowJointTrajectoryResult::INVALID_GOAL;
    as_.setPreempted(result_);
    return ExecStateTerminate;
  }

  //
  // Failed to reach waypoint.
  //
  if( failedWaypoint() )
  {
    ROS_ERROR("Failed to reach waypoint %zd.", iWaypoint);
    result_.error_code = FollowJointTrajectoryResult::INVALID_GOAL;
    as_.setAborted(result_);
    return ExecStateTerminate;
  }

  // measure move and publish feedback
  fDist = provideFeedback(iWaypoint);

  //
  // Successfully reached waypoint.
  //
  if( fDist < m_fEpsilon )
  {
    ROS_INFO("  Reached waypoint %zd in %d iterations",
      iWaypoint, m_iterMonitor+1);
    ROS_INFO("    %s dist=%lf(%lf deg).",
      NormName(m_eNorm), fDist, radToDeg(fDist));
    ROS_INFO("    worst joint=%s, %s dist=%lf(%lf deg).",
      m_strWorstJointName.c_str(),
      NormName(m_eNorm), m_fWorstJointDist, radToDeg(m_fWorstJointDist));

    return ExecStateStartMove;
  }

  //
  // Keep monitoring.
  //
  ++m_iterMonitor;

  return ExecStateMonitorMove;
}

ASFollowTrajectory::ExecState
                    ASFollowTrajectory::monitorMoveToEndpoint(ssize_t iWaypoint)
{
  //
  // Action was preempted.
  //
  if( as_.isPreemptRequested() )
  {
    ROS_WARN("Action execution preempted on waypoint %zd.", iWaypoint);
    result_.error_code = FollowJointTrajectoryResult::INVALID_GOAL;
    as_.setPreempted(result_);
    return ExecStateTerminate;
  }

  //
  // Failed to reach waypoint.
  //
  if( failedWaypoint() )
  {
    ROS_ERROR("Failed to reach waypoint %zd.", iWaypoint);
    result_.error_code = FollowJointTrajectoryResult::INVALID_GOAL;
    as_.setAborted(result_);
    return ExecStateTerminate;
  }

  // measure move and publish feedback
  provideFeedback(iWaypoint);

  //
  // Successfully reached waypoint.
  //
  if( !m_robot.isInMotion() )
  {
    ROS_INFO("  Reached endpoint %zd in %d iterations.",
      iWaypoint, m_iterMonitor+1);
    m_bTrajCompleted = true;
    return ExecStateTerminate;
  }

  //
  // Keep monitoring.
  //
  ++m_iterMonitor;

  return ExecStateMonitorMove;
}

bool ASFollowTrajectory::failedWaypoint()
{
  return  m_robot.isEStopped() ||
          m_robot.isAlarmed() ||
         !m_robot.areServosPowered() ||
         (m_iterMonitor > m_nMaxIters);
}

double ASFollowTrajectory::measureDist(ssize_t iWaypoint)
{
  HekJointStatePoint    jointCurState;  // current joint state point
  string                jointName;      // joint name
  double                jointWpPos;     // joint waypoint position (radians)
  double                jointWpVel;     // joint waypoint velocity (radians/sec)
  double                jointCurPos;    // joint current position (radians)
  double                jointCurVel;    // joint current velocity (radians/sec)
  double                fJointDist;     // joint distance
  double                fWaypointDist;  // waypoint distance
  size_t                j;              // working index

  m_robot.getJointState(jointCurState);

  m_fWorstJointDist = 0.0;
  m_strWorstJointName.clear();

  fWaypointDist = 0.0;

  // 
  // Calculate distance from current position to target waypoint.
  //
  // Distance metric: L1, L2, or Linf of joint positions.
  // Alternerates:    L2 of zero point (end effector)
  //                  attachment point) in Cartesian space. 
  //
  for(j=0; j<m_traj.joint_names.size(); ++j)
  {
    jointName   = m_traj.joint_names[j];
    jointWpPos  = m_traj.points[iWaypoint].positions[j]; 
    jointWpVel  = m_traj.points[iWaypoint].velocities[j]; 

    if( jointCurState.hasJoint(jointName) )
    {
      jointCurPos = jointCurState[jointName].m_fPosition;
      jointCurVel = jointCurState[jointName].m_fVelocity;

      fJointDist = fabs(jointWpPos - jointCurPos);

      switch( m_eNorm )
      {
        case HekNormL2:
          fWaypointDist += pow(fJointDist, 2.0);
          break;
        case HekNormL1:
          fWaypointDist += fJointDist;
          break;
        case HekNormLinf:
          if( fJointDist > fWaypointDist )
          {
            fWaypointDist = fJointDist;
          }
          break;
      }

      if( fJointDist > m_fWorstJointDist )
      {
        m_fWorstJointDist   = fJointDist;
        m_strWorstJointName = jointName;
      }
    }
  }

  if( m_eNorm == HekNormL2 )
  {
    fWaypointDist = sqrt(fWaypointDist);
  }

  return fWaypointDist;
}

double ASFollowTrajectory::provideFeedback(ssize_t iWaypoint)
{
  HekJointStatePoint    jointCurState;  // current joint state point
  string                jointName;      // joint name
  double                jointWpPos;     // joint waypoint position (radians)
  double                jointWpVel;     // joint waypoint velocity (radians/sec)
  double                jointCurPos;    // joint current position (radians)
  double                jointCurVel;    // joint current velocity (radians/sec)
  double                fJointDist;     // joint distance
  double                fWaypointDist;  // waypoint distance
  size_t                j;              // working index

  clearFeedback();

  m_robot.getJointState(jointCurState);

  m_fWorstJointDist = 0.0;
  m_strWorstJointName.clear();

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

      fJointDist = fabs(jointWpPos - jointCurPos);

      switch( m_eNorm )
      {
        case HekNormL2:
          fWaypointDist += pow(fJointDist, 2.0);
          break;
        case HekNormL1:
          fWaypointDist += fJointDist;
          break;
        case HekNormLinf:
          if( fJointDist > fWaypointDist )
          {
            fWaypointDist = fJointDist;
          }
          break;
      }

      if( fJointDist > m_fWorstJointDist )
      {
        m_fWorstJointDist   = fJointDist;
        m_strWorstJointName = jointName;
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
  ROS_WARN("Received trajectory following preempt.");
  m_robot.freeze();
}
