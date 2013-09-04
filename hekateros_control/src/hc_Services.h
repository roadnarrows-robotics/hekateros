#ifndef _HC_SERVICES
#define _HC_SERVICES

#include <string>

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

#include "Hekateros/hekateros.h"
#include "Hekateros/hekXmlCfg.h"
#include "Hekateros/hekRobot.h"

#include "hekateros_control.h"

/*!
 *  \brief Request calibrate.
 */
bool Calibrate(hekateros_control::Calibrate::Request  &req,
               hekateros_control::Calibrate::Response &res)
{
  if(pRobot==NULL)
    fprintf(stderr, "\n\nUHOH\n\n");
  //if( pRobot->calibrate() < 0 )
  if( pRobot->calibrateAsync() < 0 )
  {
    ROS_ERROR("Calibration failed.");
    return false;
  }
  else
  {
    ROS_INFO("Robot calibrated.");
  }

  return true;
}

/*!
 *  \brief Request clear alarms
 */
bool ClearAlarms(hekateros_control::ClearAlarms::Request  &req,
                 hekateros_control::ClearAlarms::Response &res)
{
  pRobot->clearAlarms();
  return true;
}

/*!
 *  \brief Request close gripper
 */
bool CloseGripper(hekateros_control::CloseGripper::Request  &req,
                  hekateros_control::CloseGripper::Response &res)
{
  pRobot->closeGripper();
  return true;
}

/*!
 *  \brief Request estop
 */
bool EStop(hekateros_control::EStop::Request  &req,
               hekateros_control::EStop::Response &res)
{
  ROS_INFO("ESTOP! You must issue a clear_estop request to continue.");
  pRobot->estop();
  return true;
}

/*!
 *  \brief Request freeze
 */
bool Freeze(hekateros_control::Freeze::Request  &req,
            hekateros_control::Freeze::Response &res)
{
  ROS_INFO("FREEZE!");
  pRobot->freeze();
  return true;
}

/*!
 *  \brief Get the hekateros version and configuration
 */
bool GetProductInfo(hekateros_control::GetProductInfo::Request  &req,
                    hekateros_control::GetProductInfo::Response &res)
{
  ROS_INFO("Retrieving product info!");
  int maj, min, rev;
  string verString;

  // TODO DHP if(!pRobot->isDescLoaded())
  if(false)
  {
    ROS_ERROR("Robot desc file not loaded - unable to determine version.");
    return false;
  }

  pRobot->getVersion(maj, min, rev);
  verString = pRobot->getVersion();

  res.i.version_string = verString;
  res.i.maj=maj;
  res.i.min=min;
  res.i.rev=rev;

  res.i.product_id = pRobot->getProdId();
  res.i.product_name = pRobot->getProdName();
  res.i.desc = pRobot->getFullProdBrief();

  return true;
}

/*!
 * \brief Go to hekateros balanced position
 */
bool GotoBalancedPos(hekateros_control::GotoBalancedPos::Request &req,
                     hekateros_control::GotoBalancedPos::Response &rsp)
{
  ROS_INFO("Moving Hekateros to a balanced position.");
  pRobot->gotoBalancedPos();
  return true;
}

/*!
 * \brief Go to hekateros parked position
 */
bool GotoParkedPos(hekateros_control::GotoParkedPos::Request &req,
                   hekateros_control::GotoParkedPos::Response &rsp)
{
  ROS_INFO("Moving Hekateros to a parked position.");
  pRobot->gotoParkedPos();
  return true;
}

/*!
 * \brief Go to hekateros balanced position
 */
bool GotoZeroPt(hekateros_control::GotoZeroPt::Request &req,
                hekateros_control::GotoZeroPt::Response &rsp)
{
  ROS_INFO("Moving Hekateros to calibrated zero point.");
  pRobot->gotoZeroPtPos();
  return true;
}

/*!
 *  \brief Check if Hekateros is [not] alarmed.
 */
bool IsAlarmed(hekateros_control::IsAlarmed::Request  &req,
               hekateros_control::IsAlarmed::Response &res)
{
  ROS_INFO("Checking Hekateros alarm state");
  if(res.is_alarmed = pRobot->isAlarmed())
  {
    ROS_WARN("Hekateros Robot is alarmed.");
  }
  
  return true;
}

/*!
 *  \brief Check if Hekateros is [not] calibrated.
 */
bool IsCalibrated(hekateros_control::IsCalibrated::Request  &req,
                  hekateros_control::IsCalibrated::Response &res)
{
  ROS_INFO("Checking Hekateros calibration state");
  if( !(res.is_calibrated = pRobot->isCalibrated()) )
  {
    ROS_WARN("Hekateros Robot is uncalibrated.");
  }
  
  return true;
}

/*!
 *  \brief Check if Hekateros description is [not] loaded.
 */
bool IsDescLoaded(hekateros_control::IsDescLoaded::Request  &req,
                  hekateros_control::IsDescLoaded::Response &res)
{
  ROS_INFO("Checking Hekateros description state");
  if( !(res.is_desc_loaded = pRobot->isDescribed()) )
  {
    ROS_WARN("Hekateros Robot description file not loaded.");
  }
  
  return true;
}

/*!
 *  \brief Request open gripper
 */
bool OpenGripper(hekateros_control::OpenGripper::Request  &req,
                 hekateros_control::OpenGripper::Response &res)
{
  pRobot->openGripper();
  return true;
}

/*!
 *  \brief Request release servos
 */
bool Release(hekateros_control::Release::Request  &req,
             hekateros_control::Release::Response &res)
{
  ROS_INFO("Releasing Hekateros servos");
  pRobot->release();
  return true;
}

/*!
 *  \brief Request release estop
 */
bool ResetEStop(hekateros_control::ResetEStop::Request  &req,
                hekateros_control::ResetEStop::Response &res)
{
  ROS_INFO("Clearing ESTOP condition");
  pRobot->resetEStop();
  return true;
}

/*!
 *  \brief Request set robot mode.
 */
bool SetRobotMode(hekateros_control::SetRobotMode::Request  &req,
                  hekateros_control::SetRobotMode::Response &res)
{
  ROS_INFO("Setting robot mode.");
  pRobot->setRobotMode((HekRobotMode)req.mode.val);
  return false;
}

#endif // _HC_SERVICES
