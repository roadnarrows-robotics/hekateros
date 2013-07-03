#ifndef _HC_FOLLOW_JOINT_TRAJECTORY_AS
#define _HC_FOLLOW_JOINT_TRAJECTORY_AS
#include "ros/ros.h"

#include "actionlib/server/simple_action_server.h"
#include "control_msgs/FollowJointTrajectoryAction.h"
#include "std_msgs/Duration.h"

#include "hekateros_control.h"

class FollowJointTrajectoryAS
{
public:
  FollowJointTrajectoryAS(std::string name, ros::NodeHandle n):
  as_(n, name, false),
  action_name_(name)
  {
    //register the goal and feeback callbacks
    as_.registerGoalCallback(
        boost::bind(&FollowJointTrajectoryAS::set_goal_cb, this));
    as_.registerPreemptCallback(
        boost::bind(&FollowJointTrajectoryAS::preempt_cb, this));

    as_.start();
  }

  ~FollowJointTrajectoryAS(void)
  {
  }

  void set_goal_cb()
  {
    //goal_=as_acceptNewGoal()->samples
  }

  void preempt_cb()
  {
  }

protected:
  string action_name_;
  actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> as_;
  trajectory_msgs::JointTrajectory goal_;
  control_msgs::JointTolerance path_tol_;
  control_msgs::JointTolerance goal_tol_;

  std_msgs::Duration goal_time_tol_;



};

#endif // _HC_FOLLOW_JOINT_TRAJECTORY_AS
