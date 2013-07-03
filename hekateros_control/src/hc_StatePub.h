#ifndef _HC_STATE_PUB
#define _HC_STATE_PUB
#include "sensor_msgs/JointState.h"
#include "hekateros_control/HekJointStateExtended.h"

#include "industrial_msgs/RobotStatus.h"
#include "hekateros_control/HekRobotStatusExtended.h"

using namespace hekateros;

/*!
 * \brief Update the current robot status 
 */
int updateRobotStatus(
    industrial_msgs::RobotStatus &robot_status, 
    hekateros_control::HekRobotStatusExtended &robot_status_ex)
{
  HekRobotState status;
  pRobot->getRobotState(status);

  // set robot status time stamps
  robot_status.header.stamp       = ros::Time::now();
  robot_status.header.frame_id    = "0";
  robot_status_ex.header.stamp    = ros::Time::now();
  robot_status_ex.header.frame_id = "0";

  robot_status.mode.val              = status.m_eRobotMode;
  robot_status.e_stopped.val         = status.m_eIsEStopped;
  robot_status.drives_powered.val    = status.m_eAreDrivesPowered;
  robot_status.motion_possible.val   = status.m_eIsMotionPossible;
  robot_status.in_motion.val         = status.m_eIsInMotion;
  robot_status.in_error.val          = status.m_eIsInError;
  robot_status.error_code            = status.m_nErrorCode;

  robot_status_ex.mode.val            = status.m_eRobotMode;
  robot_status_ex.e_stopped.val       = status.m_eIsEStopped;
  robot_status_ex.drives_powered.val  = status.m_eAreDrivesPowered;
  robot_status_ex.motion_possible.val = status.m_eIsMotionPossible;
  robot_status_ex.in_motion.val       = status.m_eIsInMotion;
  robot_status_ex.in_error.val        = status.m_eIsInError;
  robot_status_ex.error_code          = status.m_nErrorCode;
  robot_status_ex.is_calibrated.val   = status.m_eIsCalibrated;

  for (int i=0; i<status.m_vecServoHealth.size(); ++i)
  {
    hekateros_control::ServoHealth sh;
    sh.servo_id = status.m_vecServoHealth[i].m_nServoId;
    sh.temp     = status.m_vecServoHealth[i].m_fTemperature;
    sh.voltage  = status.m_vecServoHealth[i].m_fVoltage;
    sh.alarm    = status.m_vecServoHealth[i].m_uAlarms;
    robot_status_ex.servo_health.push_back(sh);
  }

}

/*!
 * \brief Update the current joint states 
 */
int updateJointStates(
    sensor_msgs::JointState &joint_states, 
    hekateros_control::HekJointStateExtended &joint_states_ex)
{
  int n=0; // number of joints reported

  if(!pRobot->isCalibrated())
  {
    ROS_WARN("Hekateros not calibrated - joint states should not be trusted.");
fprintf(stderr, "(!)-- dhp:but we're doing it anyway, until our testing arm is fixed\n\n");
    // TODO DHP - return -1;
  }

  HekJointStatePoint states;
  pRobot->getJointState(states);
  
  // set time stamps
  joint_states.header.stamp       = ros::Time::now();
  joint_states.header.frame_id    = "0";
  joint_states_ex.header.stamp    = ros::Time::now();
  joint_states_ex.header.frame_id = "0";

  // clear previous joint_state data
  joint_states.name.clear();
  joint_states.position.clear();
  joint_states.velocity.clear();
  joint_states.effort.clear();

  // clear previous joint_states_ex data
  joint_states_ex.position.clear();
  joint_states_ex.velocity.clear();
  joint_states_ex.effort.clear();
  joint_states_ex.master_servo_id.clear();
  joint_states_ex.slave_servo_id.clear();
  joint_states_ex.odometer_pos.clear();
  joint_states_ex.encoder_pos.clear();
  joint_states_ex.raw_speed.clear();

  for (n=0; n<states.getNumPoints(); ++n)
  {
    joint_states.name.push_back(states[n].m_strName);
    joint_states.position.push_back(states[n].m_fPosition);
    joint_states.velocity.push_back(states[n].m_fVelocity);
    joint_states.effort.push_back(states[n].m_fEffort);

    joint_states_ex.name.push_back(states[n].m_strName);
    joint_states_ex.position.push_back(states[n].m_fPosition);
    joint_states_ex.velocity.push_back(states[n].m_fVelocity);
    joint_states_ex.effort.push_back(states[n].m_fEffort);
    joint_states_ex.master_servo_id.push_back(states[n].m_nMasterServoId);
    joint_states_ex.slave_servo_id.push_back(states[n].m_nSlaveServoId);
    joint_states_ex.odometer_pos.push_back(states[n].m_nOdPos);
    joint_states_ex.encoder_pos.push_back(states[n].m_nEncPos);
    joint_states_ex.raw_speed.push_back(states[n].m_nSpeed);
  }

  return n;
}

#endif // _HC_STATE_PUB
