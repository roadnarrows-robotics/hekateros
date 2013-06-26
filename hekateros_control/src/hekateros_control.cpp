//
//  TODO (dhp) - set header
//
#include "ros/ros.h"
#include "actionlib/server/simple_action_server.h"

#include "rnr/rnrconfig.h"
#include "rnr/log.h"

#include "Hekateros/hekateros.h"
#include "Hekateros/hekXmlCfg.h"
#include "Hekateros/hekRobot.h"

#include "hekateros_control/CalibrateAction.h"
#include "hekateros_control/QueryVersion.h"
#include "hekateros_control/Version.h"

using namespace ::hekateros;
HekRobot *pRobot = NULL;

typedef actionlib::SimpleActionServer<hekateros_control::CalibrateAction> 
                                                                   CalibrateAS;

void CalibrateCB(const hekateros_control::CalibrateGoalConstPtr& goal, 
                  CalibrateAS* as)
{
  // do awesome calibration stuff here
  ROS_INFO("Calibrating Hekateros... It's a Hek of a good time!");
  // DHP - TODO calibration stuff here
  as->setSucceeded();
}

bool hek_version(hekateros_control::QueryVersion::Request &req,
                 hekateros_control::QueryVersion::Response &rsp)
{
  ROS_INFO("Serving 'QueryVersion' request (DUMMY)");
  rsp.v.maj=0;
  rsp.v.min=0;
  rsp.v.rev=0;

  rsp.v.configuration="Deluxe";

  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "hek_control_server");

  ros::NodeHandle nh_;

  ros::ServiceServer versionService = 
       nh_.advertiseService("hek_version", hek_version);

  // action servers
  CalibrateAS calibAS(nh_, "calibrate", 
                       boost::bind(&CalibrateCB, _1, &calibAS), 
                       false);
  calibAS.start();

  ROS_INFO("Hekateros Control Server - ready for action!");
  ros::spin();

  return 0;
}
