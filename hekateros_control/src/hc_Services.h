#ifndef _HC_SERVICES
#define _HC_SERVICES

#include <string>

#include "hekateros_control/GetVersion.h"
#include "hekateros_control/IsCalibrated.h"

#include "Hekateros/hekateros.h"
#include "Hekateros/hekXmlCfg.h"
#include "Hekateros/hekRobot.h"

#include "hekateros_control.h"

/*!
 *  \brief Get the hekateros version and configuration
 */
bool GetVersion(hekateros_control::GetVersion::Request  &req,
                hekateros_control::GetVersion::Response &res)
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

  res.v.maj=maj;
  res.v.min=min;
  res.v.rev=rev;
  res.v.configuration = verString;

  return true;
}

/*!
 *  \brief Get the hekateros version and configuration
 */
bool IsCalibrated(hekateros_control::IsCalibrated::Request  &req,
                  hekateros_control::IsCalibrated::Response &res)
{
  int maj, min, rev;
  string verString;


  if((res.is_calibrated = pRobot->isCalibrated()) == false)
  {
    ROS_WARN("Hekateros Robot is uncalibrated.");
  }
  
  return true;
}

#endif // _HC_SERVICES
