//
//  TODO (dhp) - set header
//
#include <string>

#include "ros/ros.h"

#include "actionlib/server/simple_action_server.h"
#include "control_msgs/FollowJointTrajectoryFeedback.h"

#include "rnr/rnrconfig.h"
#include "rnr/log.h"

#include "hekateros_control/GetVersion.h"

#include "Hekateros/hekateros.h"
#include "Hekateros/hekXmlCfg.h"
#include "Hekateros/hekRobot.h"


using namespace ::hekateros;
HekRobot *pRobot = NULL;

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
 * \brief Update the current joint feedback states 
 */
int updateJointTrajectoryFeedback(
    control_msgs::FollowJointTrajectoryFeedback &feedback_states)
{
  int n=0; // number of joints reported

  if(!pRobot->isCalibrated())
  {
    ROS_WARN("Hekateros not calibrated - joint states cannot be published.");
    DHP return -1;
  }

  HekJointStatePoint states;
  pRobot->getJointState(states);
  
  feedback_states.actual.positions.clear();
  feedback_states.actual.velocities.clear();
  for (n=0; n<states.getNumPoints(); ++n)
  {
    feedback_states.actual.positions.push_back(states[n].m_fPosition);
    feedback_states.actual.velocities.push_back(states[n].m_fVelocity);
  }

  return n;
}

/*!
 *  \brief Hekatero ROS control node main 
 */
int main(int argc, char **argv)
{
  // set loglevel for RN libs
  LOG_SET_THRESHOLD(LOG_LEVEL_DIAG1);

  ros::init(argc, argv, "hek_control_server");
  ros::NodeHandle nh_;

  pRobot = new HekRobot;

  // hek bringup: load and parse hekateros.conf
  // DHP TODO - make config_fn configurable as input param
  string config_fn = "/etc/hekateros.conf"; 
  HekXmlCfg xml;
  if( xml.loadFile(config_fn) < 0 )
  {
    ROS_ERROR("Error: Loading XML file '%s' failed.", config_fn.c_str());
    return -1; // DHP TODO - find suitable error code
  }
  else if( xml.setHekDescFromDOM(*pRobot->getHekDesc()) < 0 )
  {
    ROS_ERROR("Error: Setting robot description failed.");
    return -1; // DHP TODO - find suitable error code
  }
  else if( pRobot->getHekDesc()->markAsDescribed() < 0 )
  {
    ROS_ERROR("Error: Failed to finalize descriptions.");
    return -1; // DHP TODO - find suitable error code
  }
  else
  {
    ROS_INFO("Robot description loaded for %s - check!",
       pRobot->getHekDesc()->getFullProdBrief().c_str());
  }

  //hek bringup: connect to robot
  // DHP TODO - make dev_name configurable as input param
  string dev_name = "/dev/ttyUSB0";
  if( pRobot->connect(dev_name) < 0 )
  {
    ROS_ERROR("Error: Failed to connnect to '%s'.", dev_name.c_str());
    return -1; // DHP TODO - find suitable error code
  }
  else
  {
    ROS_INFO("Robot connected - check!");
  }


  //
  // services 
  ros::ServiceServer versionS = nh_.advertiseService("get_version", GetVersion);
  ROS_INFO("get_version service - check!");


  //
  // published topics
  ros::Publisher feedback_states_pub = 
    nh_.advertise<control_msgs::FollowJointTrajectoryFeedback>("joint_states", 
                                                               15);

  ROS_INFO("All systems go. Hekateros Control Server - ready for action!");
  control_msgs::FollowJointTrajectoryFeedback feedback_states;
  ros::Rate loop_rate(30);

  int seq=0;
  while(ros::ok())
  {

    int n;
    if((n = updateJointTrajectoryFeedback(feedback_states)) > 0)
    {
      ROS_INFO("Publishing state for %d joints", n);
      feedback_states.header.seq=seq;
      feedback_states.header.stamp=ros::Time::now();
      feedback_states.header.frame_id="0";
      feedback_states_pub.publish(feedback_states);
    }

    ros::spinOnce(); 
    loop_rate.sleep();
    ++seq;
  }

  return 0;
}
