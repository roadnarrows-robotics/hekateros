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
    as_.registerGoalCallback(
        boost::bind(&CalibrateAS::goal_cb, this));
    as_.registerPreemptCallback(
        boost::bind(&CalibrateAS::preempt_cb, this));

    as_.start();
  }

  ~CalibrateAS(void)
  {
  }

  void execute_cb(const hekateros_control::CalibrateGoalConstPtr &goal)
  {
    ROS_INFO("Executing Calibrate goal.");
    pRobot->calibrateAsync();

    while(pRobot->getAsyncState() && !(as_.isPreemptRequested()))
    {
      updateOpState(names_, feedback_);
      as_.publishFeedback(feedback_);
      sleep(1);
    }

    ROS_WARN("Calibration complete.");
    as_.setSucceeded();

    return;
  }

  void goal_cb()
  {
    ROS_INFO("New calibration goal received.");
    pRobot->cancelAsyncTask();
    as_.setPreempted();
    as_.acceptNewGoal();
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

};

#endif // _HC_CALIBRATE_AS_H
