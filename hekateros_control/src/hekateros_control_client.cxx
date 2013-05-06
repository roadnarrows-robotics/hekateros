//
//  TODO (dhp) - set header
//

#include <string>
#include "ros/ros.h"


#include "hekateros_control/QueryVersion.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "hek_control_client");
  ros::NodeHandle n;

  ros::ServiceClient versionClient = 
       n.serviceClient<hekateros_control::QueryVersion>("hek_version");

  hekateros_control::QueryVersion srv;
  if(versionClient.call(srv))
  {
    ROS_INFO("Hekateros Version: %d.%d.%d-%s", (int)srv.response.v.maj, (int)srv.response.v.min, (int)srv.response.v.rev, srv.response.v.configuration.c_str());
  }

  return 0;
}
