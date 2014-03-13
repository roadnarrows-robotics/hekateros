#ifndef _HC_SUBSCRIPTIONS
#define _HC_SUBSCRIPTIONS

#include "ros/ros.h"

#include "trajectory_msgs/JointTrajectory.h"
#include "Hekateros/hekRobot.h"

#include "hekateros_control_tmp.h"

void joint_commandCB(const trajectory_msgs::JointTrajectory &jt)
{
  ROS_DEBUG("Executing joint_command callback");

  HekJointTrajectoryPoint pt;
  // load trajectory point
  for(int j=0; j<jt.joint_names.size(); ++j)
  {
    pt.append(jt.joint_names[j],
              jt.points[0].positions[j], 
              jt.points[0].velocities[j]);
    ROS_INFO("j = %d pos=%2.1f speed=%2.1f", j, 
                                            jt.points[0].positions[j], 
                                            jt.points[0].velocities[j]);
  }

  pRobot->moveArm(pt);
  return;
}

#endif // _HC_SUBSCRIPTIONS
