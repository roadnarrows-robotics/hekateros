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
    static const double MaxSecs = 10.0; ///< maximum seconds to reach a waypoint

    /*!
     * \brief Trajectory execution states.
     */
    enum ExecState
    {
      ExecStateStartMove,     ///< start move to next waypoint
      ExecStateMonitorMove,   ///< monitor move
      ExecStateTerminate      ///< terminate 
    };

    /*!
     * \brief Initialization constructor.
     *
     * \param name    Action server name.
     * \param node    Node-specific class instance.
     */
    ASFollowTrajectory(std::string name, HekaterosControl &node) :
      action_name_(name),       // action name
      node_(node),              // hekateros node
      m_robot(node.getRobot()),
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
    std::string       action_name_; ///< action name
    HekaterosControl &node_;        ///< hekateros control node instance
    actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction>
                                      as_;  ///< action simple server
    control_msgs::FollowJointTrajectoryFeedback feedback_;
                                    ///< progress feedback
    control_msgs::FollowJointTrajectoryResult result_;
                                    ///< action results

    hekateros::HekRobot &m_robot;         ///< hekateros robot
    hekateros::HekNorm  m_eNorm;          ///< waypoint distance norm
    double              m_fEpsilon;       ///< waypoint distance epsilon
    trajectory_msgs::JointTrajectory m_traj; ///< goal trajectory
    ExecState           m_eState;         ///< execution state
    bool                m_bTrajCompleted; ///< trajectory [not] completed to end
    int                 m_nMaxIters;      ///< maximum iterations per waypoint
    int                 m_iterMonitor;    ///< monitoring iteration count

    /*!
     * \brief Start move to next waypoint or endpoint.
     *
     * \param iWaypoint Current waypoint along the trajectoy path.
     *
     * \return Returns next execution state.
     */
    ExecState startMoveToPoint(size_t iWaypoint);

    /*!
     * \brief Monitor move to intermediate waypoint.
     *
     * \param iWaypoint Current waypoint along the trajectoy path.
     *
     * \return Returns next execution state.
     */
    ExecState monitorMoveToWaypoint(size_t iWaypoint);

    /*!
     * \brief Monitor move to endpoint (last waypoint).
     *
     * \param iWaypoint Current waypoint along the trajectoy path.
     *
     * \return Returns next execution state.
     */
    ExecState monitorMoveToEndpoint(size_t iWaypoint);

    /*!
     * \brief Measure waypoint move from current position and provide feedback.
     *
     * \param iWaypoint Current waypoint along the trajectoy path.
     *
     * \return Distance from waypoint.
     */
    double measureMove(size_t iWaypoint);

    /*!
     * \brief Test if current move to the the target waypoint failed.
     *
     * \return Returns true or false.
     */
    bool failedWaypoint();

    /*!
     * \brief Publish feedback.
     *
     * \param iWaypoint Current waypoint along the trajectoy path.
     */
    void publishFeedback(ssize_t iWaypoint);

    /*!
     * \brief Clear feedback data.
     */
    void clearFeedback();

    /*!
     * \brief Add point to feedback.
     *
     * \param jointName     Joint name.
     * \param jointWpPos    Joint waypoint position (radians).
     * \param jointWpVel    Joint waypoint velocity (radians/second).
     * \param jointCurPos   Joint current position (radians).
     * \param jointCurVel   Joint current velocity (radians/second).
     */
    void addFeedbackPoint(const std::string &jointName,
                          const double jointWpPos,  const double jointWpVel,
                          const double jointCurPos, const double jointCurVel);
  };

} // namespace hekateros_control

#endif // _HEKATEROS_AS_TRAJECTORY_H
