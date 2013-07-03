#ifndef _HC_SERVICES
#define _HC_SERVICES

#include <string>

#include "hekateros_control/Calibrate.h"
#include "hekateros_control/GetProductInfo.h"
#include "hekateros_control/IsCalibrated.h"
#include "hekateros_control/IsDescLoaded.h"

#include "Hekateros/hekateros.h"
#include "Hekateros/hekXmlCfg.h"
#include "Hekateros/hekRobot.h"

#include "hekateros_control.h"

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
 *  \brief Check if Hekateros is [not] calibrated.
 */
bool IsCalibrated(hekateros_control::IsCalibrated::Request  &req,
                  hekateros_control::IsCalibrated::Response &res)
{
  if((res.is_calibrated = pRobot->isCalibrated()) == false)
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
  if((res.is_desc_loaded = pRobot->isDescribed()) == false)
  {
    ROS_WARN("Hekateros Robot description file not loaded.");
  }
  
  return true;
}

/*!
 *  \brief Request calibrate.
 */
bool Calibrate(hekateros_control::Calibrate::Request  &req,
               hekateros_control::Calibrate::Response &res)
{
  if( pRobot->calibrate() < 0 )
  {
    printf("Error: Calibration failed.\n");
    return false;
  }
  else
  {
    printf("Robot calibrated.\n");
  }

  return true;
}


#endif // _HC_SERVICES
