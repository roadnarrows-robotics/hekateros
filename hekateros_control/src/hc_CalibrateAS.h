#ifndef _HC_CALIBRATE_AS_H
#define _HC_CALIBRATE_AS_H

#include <unistd.h>

#include "ros/ros.h"

#include "actionlib/server/simple_action_server.h"
#include "hekateros_control/CalibrateAction.h"

#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"
#include "hekateros_control/HekJointStateExtended.h"

#include "hekateros_control.h"
#include "hc_StatePub.h"

class CalibrateAS
{
public:
  CalibrateAS(std::string name, ros::NodeHandle n):
  as_(n, name, boost::bind(&CalibrateAS::execute_cb, this, _1), false),
  action_name_(name)
  {
    //register the goal and feeback callbacks
    as_.registerPreemptCallback(
        boost::bind(&CalibrateAS::preempt_cb, this));

    as_.start();
  }

  ~CalibrateAS(void)
  {
  }

  void execute_cb(const hekateros_control::CalibrateGoalConstPtr &goal)
  {
    int rc;

    ROS_INFO(
        "Executing Calibrate action - please standby.");

    pRobot->calibrateAsync(goal->force_recalib? true: false);

    while( pRobot->getAsyncState() == HekAsyncTaskStateWorking )
    {
      if( as_.isPreemptRequested() )
      {
        break;
      }

      updateOpState(names_, feedback_);
      as_.publishFeedback(feedback_);
      sleep(1);
    }

    rc = pRobot->getAsyncRc();
    //result_.rc = rc;
    
    if( rc == HEK_OK )
    {
      ROS_INFO("Calibration complete.");
      result_.op.calib_state = hekateros_control::HekOpState::CALIBRATED;
      as_.setSucceeded(result_);
      as_.publishFeedback(feedback_);
    }
    else if( rc != -HEK_ECODE_INTR )
    {
      ROS_ERROR("Calibration aborted with error code %d.", rc);
      result_.op.calib_state = hekateros_control::HekOpState::UNCALIBRATED;
      as_.setAborted(result_);
    }
  }

  void preempt_cb()
  {
    ROS_INFO("Calibration goal cancelled.");
    pRobot->cancelAsyncTask();
    as_.setPreempted();
    as_.acceptNewGoal();
  }

protected:
  string action_name_;
  actionlib::SimpleActionServer<hekateros_control::CalibrateAction> as_;

  // feedback data
  std_msgs::String                     names_;
  hekateros_control::CalibrateFeedback feedback_;
  hekateros_control::CalibrateResult   result_;

};

#endif // _HC_CALIBRATE_AS_H
