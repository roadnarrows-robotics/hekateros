//
//  TODO (dhp) - set header
//
#include <string>
#include "rnr/rnrconfig.h"
#include "rnr/log.h"

#include "ros/ros.h"
#include "actionlib/server/simple_action_server.h"

#include "control_msgs/FollowJointTrajectoryFeedback.h"
#include "hekateros_control/CalibrateAction.h"

#include "Hekateros/hekateros.h"
#include "Hekateros/hekXmlCfg.h"
#include "Hekateros/hekRobot.h"


using namespace ::hekateros;
HekRobot *pRobot = NULL;

/*!
 *  \brief Calibrate Action Server
 */
typedef actionlib::SimpleActionServer<hekateros_control::CalibrateAction> 
                                                                   CalibrateAS;

/*!
 * \brief Calibrate Action Callback
 */
void CalibrateCB(const hekateros_control::CalibrateGoalConstPtr& req, 
                  CalibrateAS* as)
{
  ROS_INFO("Calibrating Hekateros... please stand by.");

  // do awesome stuff here

  as->setSucceeded();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "hek_control_server");
  ros::NodeHandle nh_;

  pRobot = new HekRobot;

  // hek bringup: load and parse hekateros.conf
  // DHP TODO - make config_fn configurable as input param
  string config_fn = "/etc/hekateros.conf"; 
  HekXmlCfg xml;
  if( xml.loadFile(config_fn) < 0 )
  {
    ROS_ERROR("Error: Loading XML file '%s' failed.\n", config_fn.c_str());
    return -1; // DHP TODO - find suitable error code
  }
  else if( xml.setHekDescFromDOM(*pRobot->getHekDesc()) < 0 )
  {
    ROS_ERROR("Error: Setting robot description failed.\n");
    return -1; // DHP TODO - find suitable error code
  }
  else if( pRobot->getHekDesc()->markAsDescribed() < 0 )
  {
    ROS_ERROR("Error: Failed to finalize descriptions.\n");
    return -1; // DHP TODO - find suitable error code
  }
  else
  {
    ROS_INFO("Robot description complete for %s.\n",
       pRobot->getHekDesc()->getFullProdBrief().c_str());
  }

  //hek bringup: connect to robot
  // DHP TODO - make dev_name configurable as input param
  string dev_name = "/dev/ttyUSB0";
  if( pRobot->connect(dev_name) < 0 )
  {
    ROS_ERROR("Error: Failed to connnect to '%s'.\n", dev_name.c_str());
    return -1; // DHP TODO - find suitable error code
  }
  else
  {
    ROS_INFO("Robot connected.\n");
  }

  // setup calibrate action server
  CalibrateAS calibAS(nh_, "calibrate", 
                       boost::bind(&CalibrateCB, _1, &calibAS), 
                       false);
  calibAS.start();

  ROS_INFO("Hekateros Control Server - ready for action!");
  ros::spin();

  return 0;
}
