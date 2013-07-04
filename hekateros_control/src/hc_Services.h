#ifndef _HC_SERVICES
#define _HC_SERVICES

#include <string>

#include "hekateros_control/Calibrate.h"
#include "hekateros_control/ClearAlarms.h"
#include "hekateros_control/EStop.h"
#include "hekateros_control/GetProductInfo.h"
#include "hekateros_control/IsAlarmed.h"
#include "hekateros_control/IsCalibrated.h"
#include "hekateros_control/IsDescLoaded.h"
#include "hekateros_control/Release.h"
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
  // DHP - use nonblocking calb  - if( pRobot->nb_calibrate() < 0 )
  if( pRobot->calibrate() < 0 )
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
  // TODO DHP - pRobot->ClearAlarms();
  ROS_WARN("ClearAlarm not yet implemented");
  return false;

}

/*!
 *  \brief Request estop
 */
bool EStop(hekateros_control::EStop::Request  &req,
               hekateros_control::EStop::Response &res)
{
  ROS_INFO("ESTOP!");
  pRobot->estop();
  return true;
}

/*!
 *  \brief Get the hekateros version and configuration
 */
bool GetProductInfo(hekateros_control::GetProductInfo::Request  &req,
                    hekateros_control::GetProductInfo::Response &res)
{
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
 *  \brief Check if Hekateros is [not] alarmed.
 */
bool IsAlarmed(hekateros_control::IsAlarmed::Request  &req,
               hekateros_control::IsAlarmed::Response &res)
{
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
  if( !(res.is_desc_loaded = pRobot->isDescribed()) )
  {
    ROS_WARN("Hekateros Robot description file not loaded.");
  }
  
  return true;
}

/*!
 *  \brief Request release servos
 */
bool Release(hekateros_control::Release::Request  &req,
             hekateros_control::Release::Response &res)
{
  pRobot->release();
  return true;
}

/*!
 *  \brief Request set robot mode.
 */
bool SetRobotMode(hekateros_control::SetRobotMode::Request  &req,
                  hekateros_control::SetRobotMode::Response &res)
{
  // TODO DHP - pRobot->setRobotMode();
  ROS_WARN("SetRobotMode not yet implemented");
  return false;
}

/*!
 *  \brief Request stop.
 */
bool Stop(hekateros_control::Stop::Request  &req,
               hekateros_control::Stop::Response &res)
{
  // TODO DHP - pRobot->stop();
  ROS_WARN("Stop not yet implemented");
  return false;
}

#endif // _HC_SERVICES
