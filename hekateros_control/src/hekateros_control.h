////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics Hekateros Robotiic Manipulator ROS Package
//
// Link:      https://github.com/roadnarrows-robotics/hekateros
//
// ROS Node:  pan_tilt_control
//
// File:      hekateros_control.h
//
/*! \file
 *
 * $LastChangedDate$
 * $Rev$
 *
 * \brief The ROS hekateros_control node class interface.
 *
 * \author Danial Packard (daniel@roadnarrows.com)
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \par Copyright:
 * (C) 2014  RoadNarrows
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

#ifndef _PAN_TILT_CONTROL_H
#define _PAN_TILT_CONTROL_H

#include <string>
#include <map>

//
// Includes for boost libraries
//
#include <boost/bind.hpp>

//
// ROS
//
#include "ros/ros.h"
#include "actionlib/server/simple_action_server.h"

//
// ROS generated core, industrial, and pan-tilt messages.
//
#include "trajectory_msgs/JointTrajectory.h"
#include "sensor_msgs/JointState.h"
#include "industrial_msgs/RobotStatus.h"
#include "pan_tilt_control/JointStateExtended.h"
#include "pan_tilt_control/RobotStatusExtended.h"

//
// ROS generatated pan-tilt services.
//
#include "pan_tilt_control/ClearAlarms.h"
#include "pan_tilt_control/EStop.h"
#include "pan_tilt_control/Freeze.h"
#include "pan_tilt_control/GetProductInfo.h"
#include "pan_tilt_control/GotoZeroPt.h"
#include "pan_tilt_control/IsAlarmed.h"
#include "pan_tilt_control/IsCalibrated.h"
#include "pan_tilt_control/Pan.h"
#include "pan_tilt_control/Release.h"
#include "pan_tilt_control/ResetEStop.h"
#include "pan_tilt_control/SetRobotMode.h"
#include "pan_tilt_control/Stop.h"
#include "pan_tilt_control/Sweep.h"

//
// ROS generated action servers.
//
#include "pan_tilt_control/CalibrateAction.h"

//
// RoadNarrows embedded pan-tilt library.
//
#include "pan_tilt/pan_tilt.h"
#include "pan_tilt/ptRobot.h"

//
// Node headers.
//
#include "pan_tilt_control.h"


namespace hc
{
  /*!
   * \brief The class embodiment of the pan_tilt_control ROS node.
   */
  class HekaterosControl
  {
  public:
    /*! map of ROS server services type */
    typedef std::map<std::string, ros::ServiceServer> MapServices;

    /*! map of ROS publishers type */
    typedef std::map<std::string, ros::Publisher> MapPublishers;

    /*! map of ROS subscriptions type */
    typedef std::map<std::string, ros::Subscriber> MapSubscriptions;

    /*!
     * \brief Default initialization constructor.
     *
     * \param nh  Bound node handle.
     */
    HekaterosControl(ros::NodeHandle &nh);

    /*!
     * \brief Destructor.
     */
    virtual ~HekaterosControl();

    /*!
     * \brief Advertise all services.
     */
    virtual void advertiseServices();

    /*!
     * \brief Advertise all publishers.
     *
     * \param nQueueDepth   Maximum queue depth.
     */
    virtual void advertisePublishers(int nQueueDepth=10);

    /*!
     * \brief Subscribe to all topics.
     *
     * \param nQueueDepth   Maximum queue depth.
     */
    virtual void subscribeToTopics(int nQueueDepth=10);

    /*!
     * \brief Publish.
     *
     * Call in main loop.
     */
    virtual void publish();

    /*!
     * \brief Get bound node handle.
     *
     * \return Node handle.
     */
    ros::NodeHandle &getNodeHandle()
    {
      return m_nh;
    }

    /*!
     * \brief Get bound embedded robot instance.
     *
     * \return Robot instance.
     */
    PanTiltRobot &getRobot()
    {
      return m_robot;
    }

    /*!
     * \brief Update joint state message from current robot joint state.
     *
     * \param [in] state  Robot joint state.
     * \param [out] msg   Joint state message.
     */
    void updateJointStateMsg(PanTiltJointStatePoint &state,
                             sensor_msgs::JointState &msg);

    /*!
     * \brief Update extended joint state message from current robot joint
     * state.
     *
     * \param [in] state  Robot joint state.
     * \param [out] msg   Extended joint state message.
     */
    void updateExtendedJointStateMsg(PanTiltJointStatePoint &state,
                                     pan_tilt_control::JointStateExtended &msg);

    /*!
     * \brief Update robot status message from current robot status.
     *
     * \param [in] status Robot status.
     * \param [out] msg   Robot status message.
     */
    void updateRobotStatusMsg(PanTiltRobotStatus &status,
                              industrial_msgs::RobotStatus &msg);

    /*!
     * \brief Update extended robot status message from current robot status.
     *
     * \param [in] status Robot status.
     * \param [out] msg   Extended roobt status message.
     */
    void updateExtendedRobotStatusMsg(PanTiltRobotStatus &status,
                                   pan_tilt_control::RobotStatusExtended &msg);

  protected:
    ros::NodeHandle  &m_nh;       ///< the node handler bound to this instance
    PanTiltRobot     m_robot;     ///< real-time, pan-tilt robot mechanism

    // ROS services, publishers, subscriptions.
    MapServices       m_services;       ///< pan-tilt control services
    MapPublishers     m_publishers;     ///< pan-tilt control publishers
    MapSubscriptions  m_subscriptions;  ///< pan-tilt control subscriptions

    // Messages for published data.
    sensor_msgs::JointState               m_msgJointState;
                                              ///< joint state message
    pan_tilt_control::JointStateExtended  m_msgJointStateEx;
                                              ///< extended joint state message
    industrial_msgs::RobotStatus          m_msgRobotStatus;
                                              ///< robot status message
    pan_tilt_control::RobotStatusExtended m_msgRobotStatusEx;
                                              ///< extended robot status message

    //..........................................................................
    // Service callbacks
    //..........................................................................

    /*!
     * \brief Clear robot alarms service callback.
     *
     * \param req   Service request.
     * \param rsp   Service response.
     *
     * \return Returns true on success, false on failure.
     */
    bool clearAlarms(pan_tilt_control::ClearAlarms::Request  &req,
                     pan_tilt_control::ClearAlarms::Response &rsp);

    /*!
     * \brief Emergency stop robot service callback.
     *
     * \param req   Service request.
     * \param rsp   Service response.
     *
     * \return Returns true on success, false on failure.
     */
    bool estop(pan_tilt_control::EStop::Request  &req,
               pan_tilt_control::EStop::Response &rsp);

    /*!
     * \brief Freeze (stop) robot service callback.
     *
     * \param req   Service request.
     * \param rsp   Service response.
     *
     * \return Returns true on success, false on failure.
     */
    bool freeze(pan_tilt_control::Freeze::Request  &req,
                pan_tilt_control::Freeze::Response &rsp);

    /*!
     * \brief Get robot product information service callback.
     *
     * \param req   Service request.
     * \param rsp   Service response.
     *
     * \return Returns true on success, false on failure.
     */
    bool getProductInfo(pan_tilt_control::GetProductInfo::Request  &req,
                        pan_tilt_control::GetProductInfo::Response &rsp);

    /*!
     * \brief Go to robot's zero point (home) position service callback.
     *
     * \param req   Service request.
     * \param rsp   Service response.
     *
     * \return Returns true on success, false on failure.
     */
    bool gotoZeroPt(pan_tilt_control::GotoZeroPt::Request  &req,
                    pan_tilt_control::GotoZeroPt::Response &rsp);

    /*!
     * \brief Test if robot is alarmed service callback.
     *
     * \param req   Service request.
     * \param rsp   Service response.
     *
     * \return Returns true on success, false on failure.
     */
    bool isAlarmed(pan_tilt_control::IsAlarmed::Request  &req,
                   pan_tilt_control::IsAlarmed::Response &rsp);

    /*!
     * \brief Test if robot is calibrated service callback.
     *
     * \param req   Service request.
     * \param rsp   Service response.
     *
     * \return Returns true on success, false on failure.
     */
    bool isCalibrated(pan_tilt_control::IsCalibrated::Request  &req,
                      pan_tilt_control::IsCalibrated::Response &rsp);

    /*!
     * \brief Continuously pan service callback.
     *
     * \param req   Service request.
     * \param rsp   Service response.
     *
     * \return Returns true on success, false on failure.
     */
    bool pan(pan_tilt_control::Pan::Request  &req,
             pan_tilt_control::Pan::Response &rsp);

    /*!
     * \brief Release drive power to robot motors service callback.
     *
     * \param req   Service request.
     * \param rsp   Service response.
     *
     * \return Returns true on success, false on failure.
     */
    bool release(pan_tilt_control::Release::Request  &req,
                 pan_tilt_control::Release::Response &rsp);

    /*!
     * \brief Release robot's emergency stop condition service callback.
     *
     * \param req   Service request.
     * \param rsp   Service response.
     *
     * \return Returns true on success, false on failure.
     */
    bool resetEStop(pan_tilt_control::ResetEStop::Request  &req,
                    pan_tilt_control::ResetEStop::Response &rsp);

    /*!
     * \brief Set robot's manual/auto mode service callback.
     *
     * \param req   Service request.
     * \param rsp   Service response.
     *
     * \return Returns true on success, false on failure.
     */
    bool setRobotMode(pan_tilt_control::SetRobotMode::Request  &req,
                      pan_tilt_control::SetRobotMode::Response &rsp);

    /*!
     * \brief Stop (freeze) robot service callback.
     *
     * \param req   Service request.
     * \param rsp   Service response.
     *
     * \return Returns true on success, false on failure.
     */
    bool stop(pan_tilt_control::Stop::Request  &req,
              pan_tilt_control::Stop::Response &rsp);

    /*!
     * \brief Continuously sweep service callback.
     *
     * \param req   Service request.
     * \param rsp   Service response.
     *
     * \return Returns true on success, false on failure.
     */
    bool sweep(pan_tilt_control::Sweep::Request  &req,
               pan_tilt_control::Sweep::Response &rsp);


    //..........................................................................
    // Topic Publishers
    //..........................................................................

    /*!
     * \brief Publish joint state and extended joint state topics.
     */
    void publishJointState();

    /*!
     * \brief Publish robot status and extended robot status topics.
     */
    void publishRobotStatus();


    //..........................................................................
    // Subscribed Topic Callbacks
    //..........................................................................

    /*!
     * \brief Execute joint trajectory subscibed topic callback.
     *
     * \param jt  Joint trajectory message.
     */
    void execJointCmd(const trajectory_msgs::JointTrajectory &jt);
  };

} // namespace hc


#endif // _PAN_TILT_CONTROL_H
