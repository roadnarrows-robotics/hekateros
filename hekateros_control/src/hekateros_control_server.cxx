//
//  TODO (dhp) - set header
//

#include "ros/ros.h"

#include "hekateros_control/QueryVersion.h"
#include "hekateros_control/Version.h"

bool hek_version(hekateros_control::QueryVersion::Request &req,
                 hekateros_control::QueryVersion::Response &rsp)
{
  ROS_INFO("Serving 'QueryVersion' request (DUMMY)");
  rsp.v.maj=0;
  rsp.v.min=0;
  rsp.v.rev=0;

  rsp.v.configuration="Deluxe";

  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "hek_control_server");
  ros::NodeHandle n;

  ros::ServiceServer versionService = 
       n.advertiseService("hek_version", hek_version);

  ROS_INFO("Hekateros Server up and running - Get your arm on!");
  ros::spin();

  return 0;
}
