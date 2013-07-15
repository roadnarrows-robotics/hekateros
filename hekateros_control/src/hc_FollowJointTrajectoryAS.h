#ifndef _HC_FOLLOW_JOINT_TRAJECTORY_AS
#define _HC_FOLLOW_JOINT_TRAJECTORY_AS

#include <unistd.h>

#include "ros/ros.h"

#include "actionlib/server/simple_action_server.h"
#include "control_msgs/FollowJointTrajectoryAction.h"

#include "Hekateros/hekTraj.h"

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
    
    trajectory_msgs::JointTrajectory jt = goal->trajectory;

    for(int i=0; i<jt.points.size(); ++i)
    {
      HekJointTrajectoryPoint pt;

      // load trajectory point
      for(int j=0; j<jt.joint_names.size(); ++j)
      {
        pt.append(jt.joint_names[j],
                  jt.points[i].positions[j], 
                  jt.points[i].velocities[j]);
        ROS_INFO("j = %d pos=%2.1f speed=%2.1f", j, 
                                                jt.points[i].positions[j], 
                                                jt.points[i].velocities[j]);
      }
      
      ROS_INFO("moving to trajectory point %d", i);
      pRobot->moveArm(pt);

      // temp fix - wait 2 seconds to reach each trajectory waypoint
      sleep(4);

      // real fix - check delta from goal, send next goal when near waypoint)
      //  while (delta > tol)
      //  { 
      //    sleep 0.1
      //    update delta
      //  }
      // piece of cake, right?

    }

    as_.setSucceeded();

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
