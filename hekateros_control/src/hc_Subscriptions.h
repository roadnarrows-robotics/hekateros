#ifndef _HC_SUBSCRIPTIONS
#define _HC_SUBSCRIPTIONS

#include "ros/ros.h"

#include "trajectory_msgs/JointTrajectoryPoint.h"
#include "Hekateros/hekRobot.h"

#include "hekateros_control.h"

void joint_commandCB(const trajectory_msgs::JointTrajectoryPoint &joints)
{
  ROS_WARN("joint_commandCB not yet implemented");
  return;
}

#endif // _HC_SUBSCRIPTIONS
