////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics Hekateros Robotiic Manipulator ROS Package
//
// Link:      https://github.com/roadnarrows-robotics/hekateros
//
// ROS Node:  hekateros_control
//
// File:      hekateros_as_calib.h
//
/*! \file
 *
 * $LastChangedDate$
 * $Rev$
 *
 * \brief Hekateros calibration action server class interface.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 * \author Danial Packard (daniel@roadnarrows.com)
 *
 * \par Copyright:
 * (C) 2013-2018  RoadNarrows
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

#ifndef _HEKATEROS_AS_CALIB_H
#define _HEKATEROS_AS_CALIB_H

//
// System and Boost
//
#include <unistd.h>
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
#include "sensor_msgs/JointState.h"
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
#include "Hekateros/hekRobot.h"

//
// Node headers.
//
#include "hekateros_control.h"


namespace hekateros_control
{
  /*!
   * \brief Calibrate the Hekateros robotic manipulator action server class.
   */
  class ASCalibrate
  {
  public:
    /*!
     * \brief Initialization constructor.
     *
     * \param name    Action server name.
     * \param node    Node-specific class instance.
     */
    ASCalibrate(std::string name, HekaterosControl &node) :
      action_name_(name),       // action name
      node_(node),              // hekateros node
      as_(node.getNodeHandle(), // simple action server
          name,                       // action name
          boost::bind(&ASCalibrate::execute_cb, this, _1),
                                      // execute callback
          false)                      // don't auto-start
    {
      // 
      // Optionally register the goal and feeback callbacks
      //
      as_.registerPreemptCallback(boost::bind(&ASCalibrate::preempt_cb, this));

      // start the action server
      start();
    }

    /*!
     * \brief Destructor.
     */
    virtual ~ASCalibrate()
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
     */
    void execute_cb(const CalibrateGoalConstPtr &goal);

    /*!
     * \brief ROS callback to preempt action.
     *
     * This is only needed if actions are required outside of the blocking
     * execution callback thread.
     */
    void preempt_cb();

  protected:
    std::string       action_name_;     ///< action name
    HekaterosControl &node_;            ///< hekateros control node instance

    actionlib::SimpleActionServer<CalibrateAction> as_;
                                        ///< action simple server
    CalibrateFeedback feedback_;        ///< progress feedback
    CalibrateResult   result_;          ///< action results
  };

} // namespace hekateros_control

#endif // _HEKATEROS_AS_CALIB_H
