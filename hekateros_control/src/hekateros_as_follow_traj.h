////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics Hekateros Robotiic Manipulator ROS Package
//
// Link:      https://github.com/roadnarrows-robotics/hekateros
//
// ROS Node:  hekateros_control
//
// File:      hekateros_as_follow_traj.h
//
/*! \file
 *
 * $LastChangedDate$
 * $Rev$
 *
 * \brief Hekateros follow joint trajectory action server class interface.
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

#ifndef _HEKATEROS_AS_TRAJECTORY_H
#define _HEKATEROS_AS_TRAJECTORY_H

//
// System and Boost
//
#include <sys/types.h>
#include <unistd.h>
#include <math.h>
#include <boost/bind.hpp>
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
#include "Hekateros/hekTraj.h"
#include "Hekateros/hekRobot.h"

//
// Node headers.
//
#include "hekateros_control.h"


namespace hekateros_control
{
  /*!
   * \brief Hekateros follow joint trajectory action server class.
   */
  class ASFollowTrajectory
  {
  public:
    // Monitoring tuning. Position tolerances are in radians.
    static const double TolWaypoint;  ///< intermediate waypoint position
    static const double TolGoal;      ///< final goal position
    static const int    MaxIters;     ///< maximum iterations w/o motion

    /*!
     * \brief Initialization constructor.
     *
     * \param name    Action server name.
     * \param node    Node-specific class instance.
     */
    ASFollowTrajectory(std::string name, HekaterosControl &node) :
      action_name_(name),       // action name
      node_(node),              // hekateros node
      as_(node.getNodeHandle(), // simple action server
          name,                       // action name
          boost::bind(&ASFollowTrajectory::execute_cb, this, _1),
                                      // execute callback
          false)                      // don't auto-start
    {
      // 
      // Optionally register the goal and feeback callbacks
      //
      as_.registerPreemptCallback(boost::bind(&ASFollowTrajectory::preempt_cb,
                                  this));

      // start the action server
      start();
    }

    /*!
     * \brief Destructor.
     */
    virtual ~ASFollowTrajectory()
    {
    }

    /*!
     * \brief Start the action server.
     */
    void start()
    {
      as_.start();
    }

    /*!
     * \brief ROS callback to execute action.
     *
     * The callback is executed in a separate ROS-created thread so that it
     * can block.Typically, the callback is invoked within a ROS spin.
     *
     * \param goal  Goal joint trajectory path.
     */
    void execute_cb(
                  const control_msgs::FollowJointTrajectoryGoalConstPtr &goal);

    /*!
     * \brief ROS callback to preempt action.
     *
     * This is only needed if actions are required outside of the blocking
     * execution callback thread.
     */
    void preempt_cb();

  protected:
    std::string       action_name_;         ///< action name
    HekaterosControl &node_;                ///< hekateros control node instance
    actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction>
                                      as_;  ///< action simple server
    control_msgs::FollowJointTrajectoryFeedback feedback_;
                                            ///< progress feedback
    control_msgs::FollowJointTrajectoryResult   result_;
                                            ///< action results

    double  m_fTolerance;                   ///< waypoint tolerance
    double  m_fWaypointDist;                ///< current waypoint distance
    int     m_iterMonitor;                  ///< monitoring iteration count

    /*!
     * \brief Move to next waypoint.
     *
     * \param jt        Joint trajectory path.
     * \param iWaypoint Current waypoint along the trajectoy path.
     *
     * \return Returns HEK_OK on success, \h_lt 0 on failure.
     */
    int moveToWaypoint(trajectory_msgs::JointTrajectory &jt, ssize_t iWaypoint);

    /*!
     * \brief Monitor and provide feedback of current waypoint move.
     *
     * \param jt        Joint trajectory path.
     * \param iWaypoint Current waypoint along the trajectoy path.
     */
    void monitorMove(trajectory_msgs::JointTrajectory &jt, ssize_t iWaypoint);

    /*!
     * \brief Test if current move is at the target waypoint.
     *
     * \param jt        Joint trajectory path.
     * \param iWaypoint Current waypoint along the trajectoy path.
     *
     * \return Returns true or false.
     */
    bool atWaypoint(trajectory_msgs::JointTrajectory &jt, ssize_t iWaypoint)
    {
      return fabs(m_fWaypointDist) < m_fTolerance;
    }

    /*!
     * \brief Test if current move to the the target waypoint failed.
     *
     * \param jt        Joint trajectory path.
     * \param iWaypoint Current waypoint along the trajectoy path.
     *
     * \return Returns true or false.
     */
    bool failedWaypoint(trajectory_msgs::JointTrajectory &jt,
                        ssize_t iWaypoint);

    /*!
     * \brief Publish feedback.
     *
     * \param iWaypoint Current waypoint along the trajectoy path.
     */
    void publishFeedback(ssize_t iWaypoint);
  };

} // namespace hekateros_control

#endif // _HEKATEROS_AS_TRAJECTORY_H
