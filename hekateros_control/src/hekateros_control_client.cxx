//
//  TODO (dhp) - set header
//

#include <string>
#include "ros/ros.h"

#include "hekateros_control/QueryVersion.h"
#include "hekateros_control/CalibrateAction.h"
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<hekateros_control::CalibrateAction> Client;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "hek_control_client");
  ROS_INFO("here1");
  ros::NodeHandle n;

  Client client("calibrate", true); // true -> don't need ros::spin()
  ROS_INFO("here1");
  client.waitForServer();
  ROS_INFO("here1");
  //hekateros_control::CalibrateGoal goal;
  //client.sendGoal(goal);
  //client.waitForResult(ros::Duration(5.0));
  //if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    //printf("Yay! the robot is calibrated!");
  //printf("Current State: %s\n", client.getState().toString().c_str());
  
  ros::ServiceClient versionClient = 
       n.serviceClient<hekateros_control::QueryVersion>("hek_version");

  hekateros_control::QueryVersion srv;
  if(versionClient.call(srv))
  {
    ROS_INFO("Hekateros Version: %d.%d.%d-%s", (int)srv.response.v.maj, (int)srv.response.v.min, (int)srv.response.v.rev, srv.response.v.configuration.c_str());
  }

  return 0;
}
