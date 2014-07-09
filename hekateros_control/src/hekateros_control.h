////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics Hekateros Robotiic Manipulator ROS Package
//
// Link:      https://github.com/roadnarrows-robotics/hekateros
//
// ROS Node:  hekateros_control
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

#ifndef _HEKATEROS_CONTROL_H
#define _HEKATEROS_CONTROL_H

//
// System
//
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
#include "control_msgs/FollowJointTrajectoryAction.h"
#include "control_msgs/FollowJointTrajectoryAction.h"

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
#include "hekateros_control/Calibrate.h"
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

//
// Node headers.
//
#include "hekateros_control.h"


namespace hekateros_control
{
  /*!
   * \brief The class embodiment of the hekateros_control ROS node.
   */
  class HekaterosControl
  {
  public:
    /*! map of ROS server services type */
    typedef std::map<std::string, ros::ServiceServer> MapServices;

    /*! map of ROS client services type */
    typedef std::map<std::string, ros::ServiceClient> MapClientServices;
    
    /*! map of ROS publishers type */
    typedef std::map<std::string, ros::Publisher> MapPublishers;

    /*! map of ROS subscriptions type */
    typedef std::map<std::string, ros::Subscriber> MapSubscriptions;

    /*!
     * \brief Default initialization constructor.
     *
     * \param nh  Bound node handle.
     * \param hz  Application nominal loop rate in Hertz.
     */
    HekaterosControl(ros::NodeHandle &nh, double hz);

    /*!
     * \brief Destructor.
     */
    virtual ~HekaterosControl();

    /*!
     * \brief Configure Hekateros product specifics.
     *
     * \param strCfgFile    XML configuration file name.
     *
     * \return Returns HEK_OK of success, \h_lt 0 on failure.
     */
    virtual int configure(const std::string &strCfgFile);

    /*!
     * \brief Connect to Hekateros hardware.
     *
     * \param strDevDynabus     Dynabus serial device name.
     * \param nBaudRateDynabus  Dynabus baud rate.
     * \param strDevArduino     Arduino serial device name.
     * \param nBaudRateArduino  Arduino baud rate.
     *
     * \return Returns HEK_OK of success, \h_lt 0 on failure.
     */
    int connect(const std::string &strDevDynabus,
                int                nBaudRateDynabus,
                const std::string &strDevArduino,
                int                nBaudRateArduino)
    {
      m_robot.connect(strDevDynabus, nBaudRateDynabus,
                      strDevArduino, nBaudRateArduino);
    }

    /*!
     * \brief Disconnect from Hekateros.
     *
     * \return Returns HEK_OK of success, \h_lt 0 on failure.
     */
    int disconnect()
    {
      m_robot.disconnect();
    }

    /*!
     * \brief Advertise all server services.
     */
    virtual void advertiseServices();

    /*!
     * \brief Initialize client services.
     */
    virtual void clientServices()
    {
      // No client services
    }

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
    hekateros::HekRobot &getRobot()
    {
      return m_robot;
    }

    /*!
     * \brief Update joint state message from current robot joint state.
     *
     * \param [in] state  Robot joint state.
     * \param [out] msg   Joint state message.
     */
    void updateJointStateMsg(hekateros::HekJointStatePoint &state,
                             sensor_msgs::JointState       &msg);

    /*!
     * \brief Update extended joint state message from current robot joint
     * state.
     *
     * \param [in] state  Robot joint state.
     * \param [out] msg   Extended joint state message.
     */
    void updateExtendedJointStateMsg(hekateros::HekJointStatePoint &state,
                                     HekJointStateExtended         &msg);

    /*!
     * \brief Update robot status message from current robot status.
     *
     * \param [in] status Robot status.
     * \param [out] msg   Robot status message.
     */
    void updateRobotStatusMsg(hekateros::HekRobotState     &status,
                              industrial_msgs::RobotStatus &msg);

    /*!
     * \brief Update extended robot status message from current robot status.
     *
     * \param [in] status Robot status.
     * \param [out] msg   Extended roobt status message.
     */
    void updateExtendedRobotStatusMsg(hekateros::HekRobotState &status,
                                      HekRobotStatusExtended   &msg);

  protected:
    ros::NodeHandle    &m_nh;     ///< the node handler bound to this instance
    double              m_hz;     ///< application nominal loop rate
    hekateros::HekRobot m_robot;  ///< real-time, Hekateros robotic arm

    // ROS services, publishers, subscriptions.
    MapServices       m_services;       ///< Hekateros control server services
    MapClientServices m_clientServices; ///< Hekateros control client services
    MapPublishers     m_publishers;     ///< Hekateros control publishers
    MapSubscriptions  m_subscriptions;  ///< Hekateros control subscriptions

    // Messages for published data.
    sensor_msgs::JointState       m_msgJointState;  ///< joint state message
    HekJointStateExtended         m_msgJointStateEx;
                                              ///< extended joint state message
    industrial_msgs::RobotStatus  m_msgRobotStatus; ///< robot status message
    HekRobotStatusExtended        m_msgRobotStatusEx;
                                              ///< extended robot status message

    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    // Service callbacks
    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    /*!
     * \brief Calibrate robot service callback.
     *
     * \param req   Service request.
     * \param rsp   Service response.
     *
     * \return Returns true on success, false on failure.
     */
    bool calibrate(hekateros_control::Calibrate::Request  &req,
                   hekateros_control::Calibrate::Response &rsp);

    /*!
     * \brief Clear robot alarms service callback.
     *
     * \param req   Service request.
     * \param rsp   Service response.
     *
     * \return Returns true on success, false on failure.
     */
    bool clearAlarms(hekateros_control::ClearAlarms::Request  &req,
                     hekateros_control::ClearAlarms::Response &rsp);

    /*!
     * \brief Close end-effector gripper service callback.
     *
     * \param req   Service request.
     * \param rsp   Service response.
     *
     * \return Returns true on success, false on failure.
     */
    bool closeGripper(hekateros_control::CloseGripper::Request  &req,
                      hekateros_control::CloseGripper::Response &rsp);

    /*!
     * \brief Emergency stop robot service callback.
     *
     * \param req   Service request.
     * \param rsp   Service response.
     *
     * \return Returns true on success, false on failure.
     */
    bool estop(hekateros_control::EStop::Request  &req,
               hekateros_control::EStop::Response &rsp);

    /*!
     * \brief Freeze (stop) robot service callback.
     *
     * \param req   Service request.
     * \param rsp   Service response.
     *
     * \return Returns true on success, false on failure.
     */
    bool freeze(hekateros_control::Freeze::Request  &req,
                hekateros_control::Freeze::Response &rsp);

    /*!
     * \brief Get robot product information service callback.
     *
     * \param req   Service request.
     * \param rsp   Service response.
     *
     * \return Returns true on success, false on failure.
     */
    bool getProductInfo(hekateros_control::GetProductInfo::Request  &req,
                        hekateros_control::GetProductInfo::Response &rsp);

    /*!
     * \brief Go to robot's balanced postion service callback.
     *
     * \param req   Service request.
     * \param rsp   Service response.
     *
     * \return Returns true on success, false on failure.
     */
    bool gotoBalancedPos(hekateros_control::GotoBalancedPos::Request  &req,
                         hekateros_control::GotoBalancedPos::Response &rsp);

    /*!
     * \brief Go to robot' parked postion service callback.
     *
     * \param req   Service request.
     * \param rsp   Service response.
     *
     * \return Returns true on success, false on failure.
     */
    bool gotoParkedPos(hekateros_control::GotoParkedPos::Request  &req,
                       hekateros_control::GotoParkedPos::Response &rsp);

    /*!
     * \brief Go to robot's zero point (home) position service callback.
     *
     * \param req   Service request.
     * \param rsp   Service response.
     *
     * \return Returns true on success, false on failure.
     */
    bool gotoZeroPt(hekateros_control::GotoZeroPt::Request  &req,
                    hekateros_control::GotoZeroPt::Response &rsp);

    /*!
     * \brief Test if robot is alarmed service callback.
     *
     * \param req   Service request.
     * \param rsp   Service response.
     *
     * \return Returns true on success, false on failure.
     */
    bool isAlarmed(hekateros_control::IsAlarmed::Request  &req,
                   hekateros_control::IsAlarmed::Response &rsp);

    /*!
     * \brief Test if robot is calibrated service callback.
     *
     * \param req   Service request.
     * \param rsp   Service response.
     *
     * \return Returns true on success, false on failure.
     */
    bool isCalibrated(hekateros_control::IsCalibrated::Request  &req,
                      hekateros_control::IsCalibrated::Response &rsp);

    /*!
     * \brief Test if robot description has been loaded service callback.
     *
     * \param req   Service request.
     * \param rsp   Service response.
     *
     * \return Returns true on success, false on failure.
     */
    bool isDescLoaded(hekateros_control::IsDescLoaded::Request  &req,
                      hekateros_control::IsDescLoaded::Response &rsp);

    /*!
     * \brief Open end-effector gripper service callback.
     *
     * \param req   Service request.
     * \param rsp   Service response.
     *
     * \return Returns true on success, false on failure.
     */
    bool openGripper(hekateros_control::OpenGripper::Request  &req,
                     hekateros_control::OpenGripper::Response &rsp);

    /*!
     * \brief Release drive power to robot motors service callback.
     *
     * \param req   Service request.
     * \param rsp   Service response.
     *
     * \return Returns true on success, false on failure.
     */
    bool release(hekateros_control::Release::Request  &req,
                 hekateros_control::Release::Response &rsp);

    /*!
     * \brief Release robot's emergency stop condition service callback.
     *
     * \param req   Service request.
     * \param rsp   Service response.
     *
     * \return Returns true on success, false on failure.
     */
    bool resetEStop(hekateros_control::ResetEStop::Request  &req,
                    hekateros_control::ResetEStop::Response &rsp);

    /*!
     * \brief Set robot's manual/auto mode service callback.
     *
     * \param req   Service request.
     * \param rsp   Service response.
     *
     * \return Returns true on success, false on failure.
     */
    bool setRobotMode(hekateros_control::SetRobotMode::Request  &req,
                      hekateros_control::SetRobotMode::Response &rsp);

    /*!
     * \brief Stop a set of joints robot service callback.
     *
     * \param req   Service request.
     * \param rsp   Service response.
     *
     * \return Returns true on success, false on failure.
     */
    bool stop(hekateros_control::Stop::Request  &req,
              hekateros_control::Stop::Response &rsp);


    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    // Topic Publishers
    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    /*!
     * \brief Publish joint state and extended joint state topics.
     */
    void publishJointState();

    /*!
     * \brief Publish robot status and extended robot status topics.
     */
    void publishRobotStatus();


    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    // Subscribed Topic Callbacks
    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    /*!
     * \brief Execute joint trajectory subscibed topic callback.
     *
     * \param jt  Joint trajectory message.
     */
    void execJointCmd(const trajectory_msgs::JointTrajectory &jt);
  };

} // namespace hc


#endif // _HEKATEROS_CONTROL_H
