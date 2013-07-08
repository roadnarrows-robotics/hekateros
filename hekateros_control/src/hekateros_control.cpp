//
//  TODO (dhp) - set file header
//
#include <string>

#include "ros/ros.h"
#include "rnr/rnrconfig.h"
#include "rnr/log.h"

#include "sensor_msgs/JointState.h"
#include "industrial_msgs/RobotStatus.h"
#include "hekateros_control/HekJointStateExtended.h"
#include "hekateros_control/HekRobotStatusExtended.h"

#include "Hekateros/hekateros.h"
#include "Hekateros/hekXmlCfg.h"
#include "Hekateros/hekRobot.h"

#include "hekateros_control.h"
#include "hc_Bringup.h"
#include "hc_Services.h"
#include "hc_StatePub.h"
#include "hc_Subscriptions.h"
#include "hc_FollowJointTrajectoryAS.h"

using namespace ::std;
using namespace ::hekateros;

/*!
 *  \brief Hekatero ROS control node main 
 */
int main(int argc, char **argv)
{
  // set loglevel for RN libs
  LOG_SET_THRESHOLD(LOG_LEVEL_DIAG1);

  ros::init(argc, argv, "hekateros_control");
  ros::NodeHandle n("hekateros_control");

  //pRobot = new HekRobot; // intialize global control robot

  // TODO DHP -accept config filename as param
  string config_fn = "/etc/hekateros.conf";
  if( loadHekXml(config_fn) != 0)
  {
    ROS_FATAL("Unable to load Hekateros cfg file. Aborting node: %s", argv[0]);
    return -1;
  }

  // DHP TODO - make dev_name configurable as input param
  string dev_name = "/dev/ttyUSB0";
  if( connect(dev_name) != 0)
  {
    ROS_FATAL("Unable to connect to Hekateros. Aborting node: %s.", argv[0]);
    return -1; // DHP
  }

  //
  // services 
  ros::ServiceServer calibrate_ser     = n.advertiseService("calibrate", 
                                                            Calibrate);
  // TODO DHP - reimplement calibrate as an action server / non-blocking

  ros::ServiceServer clear_alarms_ser   = n.advertiseService("clear_alarms", 
                                                             ClearAlarms);
 
  ros::ServiceServer estop_srv          = n.advertiseService("estop", 
                                                             EStop);
 
  ros::ServiceServer get_prod_info_ser  = n.advertiseService("get_product_info",
                                                             GetProductInfo);

  ros::ServiceServer is_alarmed_ser     = n.advertiseService("is_alarmed", 
                                                             IsAlarmed);

  ros::ServiceServer is_calibrated_ser  = n.advertiseService("is_calibrated", 
                                                             IsCalibrated);

  ros::ServiceServer is_desc_ser        = n.advertiseService("is_desc_loaded", 
                                                             IsDescLoaded);
 
  ros::ServiceServer release_srv        = n.advertiseService("release", 
                                                             Release);
 
  ros::ServiceServer set_robot_mode_srv = n.advertiseService("set_robot_mode", 
                                                             SetRobotMode);
 
  ros::ServiceServer stop_srv           = n.advertiseService("stop", 
                                                             Stop);
  ROS_INFO("services registered - check!");

  // services TODO DHP -
  //    set_robot_mode
  //    clear_alarms

  //
  // published topics
  ros::Publisher joint_states_pub = 
    n.advertise<sensor_msgs::JointState>("joint_states", 10);

  ros::Publisher joint_states_ex_pub = 
    n.advertise<hekateros_control::HekJointStateExtended>(
                                          "joint_states_ex", 10);

  ros::Publisher robot_status_pub = 
    n.advertise<industrial_msgs::RobotStatus>("robot_status", 10);

  ros::Publisher robot_status_ex_pub = 
    n.advertise<hekateros_control::HekRobotStatusExtended>(
                                          "robot_status_ex", 10);
  ROS_INFO("published topics registered - check!");

  //
  // subscribed topics
  ros::Subscriber joint_command_sub = n.subscribe("joint_command", 1, 
                                                  joint_commandCB);
  ROS_INFO("subscribed topics registered - check!");

  //
  // Action Servers
  FollowJointTrajectoryAS follow_joint_traj_as("follow_joint_traj_as", n);
  ROS_INFO("FollowJointTrajectory Action Server registered - check!");

  // containers for published data
  sensor_msgs::JointState joint_states;
  hekateros_control::HekJointStateExtended joint_states_ex;
  industrial_msgs::RobotStatus robot_status;
  hekateros_control::HekRobotStatusExtended robot_status_ex;

  int seq=0;
  ros::Rate loop_rate(10);
  while(ros::ok())
  {

    int n;
    if((n = updateJointStates(joint_states, joint_states_ex)) > 0)
    {
      joint_states.header.seq=seq;
      joint_states_pub.publish(joint_states);

      joint_states_ex.header.seq=seq;
      joint_states_ex_pub.publish(joint_states_ex);
    }

    if(updateRobotStatus(robot_status, robot_status_ex) == 0)
    {
      robot_status.header.seq=seq;
      robot_status_pub.publish(robot_status);

      robot_status.header.seq=seq;
      robot_status_ex_pub.publish(robot_status_ex);
    }


    ros::spinOnce(); 
    loop_rate.sleep();
    ++seq;
  }

  return 0;
}
