#ifndef _HC_BRINGUP_H
#define _HC_BRINGUP_H

#include <string>

#include "Hekateros/hekateros.h"
#include "Hekateros/hekXmlCfg.h"
#include "Hekateros/hekRobot.h"

#include "hekateros_control.h"

using namespace ::std;
using namespace ::hekateros;

/*!
 * \brief Load Hekateros XML configuration file
 */
  // hek bringup: load and parse hekateros.conf
  // DHP TODO - make config_fn configurable as input param
int loadHekXml(string filename)
{
  int rc=0; // return code

  string config_fn = "/etc/hekateros.conf"; 
  HekXmlCfg xml;
  if( xml.loadFile(config_fn) < 0 )
  {
    ROS_ERROR("Loading XML file '%s' failed.", config_fn.c_str());
    rc = -1; // DHP TODO - find suitable error code
  }
  else if( xml.setHekDescFromDOM(*pRobot->getHekDesc()) < 0 )
  {
    ROS_ERROR("Setting robot description failed.");
    rc = -1; // DHP TODO - find suitable error code
  }
  else if( pRobot->getHekDesc()->markAsDescribed() < 0 )
  {
    ROS_ERROR("Failed to finalize descriptions.");
    rc = -1; // DHP TODO - find suitable error code
  }
  else
  {
    ROS_INFO("Robot description loaded for %s - check!",
       pRobot->getHekDesc()->getFullProdBrief().c_str());

    rc = 0;
  }

  return rc;
}

int connect(string dev_name)
{
  int rc=0; // return code
  if( pRobot->connect(dev_name) < 0 )
  {
    ROS_ERROR("Failed to connect to %s", dev_name.c_str());
    return -1; // DHP TODO - find suitable error code
  }
  else
  {
    ROS_INFO("Robot connected - check!");
  }

  return rc;

}

#endif // HC_BRINGUP_H
