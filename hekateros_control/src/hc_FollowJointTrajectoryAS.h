#ifndef _HC_FOLLOW_JOINT_TRAJECTORY_AS
#define _HC_FOLLOW_JOINT_TRAJECTORY_AS
#include "ros/ros.h"

#include "actionlib/server/simple_action_server.h"
#include "control_msgs/FollowJointTrajectoryAction.h"

#include "hekateros_control.h"

class FollowJointTrajectoryAS
{
public:
  FollowJointTrajectoryAS(std::string name, ros::NodeHandle n):
  as_(n, name, 
       boost::bind(&FollowJointTrajectoryAS::execute_cb, this, _1), 
       false),
  action_name_(name)
  {
    //register the goal and feeback callbacks
    as_.registerGoalCallback(
        boost::bind(&FollowJointTrajectoryAS::goal_cb, this));
    as_.registerPreemptCallback(
        boost::bind(&FollowJointTrajectoryAS::preempt_cb, this));

    as_.start();
  }

  ~FollowJointTrajectoryAS(void)
  {
  }

  void execute_cb(const control_msgs::FollowJointTrajectoryGoalConstPtr &goal)
  {
    ROS_INFO("Executing FollowJointTrajectory goal.");
    
    //goal.m;

      /*
    for(int i=0; i<jt.points.size(); ++i)
    {
      ROS_INFO("moving to trajectory point %d", i);
      //pRobot->moveTo();
    }
      */

  }

  void goal_cb()
  {
    ROS_INFO("New FollowJointTrajectory goal received.");
    pRobot->cancelAsyncTask();
    as_.setPreempted();
    as_.acceptNewGoal();
  }

  void preempt_cb()
  {
    ROS_INFO("FollowJointTrajectory goal was preempted.");
    pRobot->cancelAsyncTask();
    as_.setPreempted();
    as_.acceptNewGoal();
  }

protected:
  string action_name_;
  actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> as_;
  trajectory_msgs::JointTrajectory goal_;
  control_msgs::JointTolerance path_tol_;
  control_msgs::JointTolerance goal_tol_;

  //std_msgs::Duration goal_time_tol_;



};

#endif // _HC_FOLLOW_JOINT_TRAJECTORY_AS
