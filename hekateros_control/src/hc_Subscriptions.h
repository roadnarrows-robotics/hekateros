#ifndef _HC_SUBSCRIPTIONS
#define _HC_SUBSCRIPTIONS

#include "trajectory_msgs/JointTrajectoryPoint.h"
#include "Hekateros/hekRobot.h"

#include "hekateros_control.h"

void joint_commandCB(const trajectory_msgs::JointTrajectoryPoint &joints)
{
  fprintf(stderr, "todo dhp - implement joint_commandCB\n");
  return;
}

#endif // _HC_SUBSCRIPTIONS
