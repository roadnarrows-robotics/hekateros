////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics Hekateros Robotic Manipulator ROS Package
//
// Link:      https://github.com/roadnarrows-robotics/hekateros
//
// ROS Node:  hek_teleop
//
// File:      hek_teleop_main.cpp
//
/*! \file
 *
 * $LastChangedDate$
 * $Rev$
 *
 * \brief The ROS Hekateros teleoperation node main.
 *
 * \author Daniel Packard (daniel@roadnarrows.com)
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \par Copyright:
 * (C) 2013-2014  RoadNarrows
 * (http://www.roadnarrows.com)
 * \n All Rights Reserved
 */
/*
 * @EulaBegin@
 * 
 * Permission is hereby granted, without written agreement and without
 * license or royalty fees, to use, copy, modify, and distribute this
 * software and its documentation for any purpose, provided that
 * (1) The above copyright notice and the following two paragraphs
 * appear in all copies of the source code and (2) redistributions
 * including binaries reproduces these notices in the supporting
 * documentation.   Substantial modifications to this software may be
 * copyrighted by their authors and need not follow the licensing terms
 * described here, provided that the new terms are clearly indicated in
 * all files where they apply.
 * 
 * IN NO EVENT SHALL THE AUTHOR, ROADNARROWS LLC, OR ANY MEMBERS/EMPLOYEES
 * OF ROADNARROW LLC OR DISTRIBUTORS OF THIS SOFTWARE BE LIABLE TO ANY
 * PARTY FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
 * DAMAGES ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION,
 * EVEN IF THE AUTHORS OR ANY OF THE ABOVE PARTIES HAVE BEEN ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 * 
 * THE AUTHOR AND ROADNARROWS LLC SPECIFICALLY DISCLAIM ANY WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
 * FITNESS FOR A PARTICULAR PURPOSE. THE SOFTWARE PROVIDED HEREUNDER IS ON AN
 * "AS IS" BASIS, AND THE AUTHORS AND DISTRIBUTORS HAVE NO OBLIGATION TO
 * PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.
 * 
 * @EulaEnd@
 */
////////////////////////////////////////////////////////////////////////////////


//
// System
// 
#include <unistd.h>
#include <signal.h>
#include <string>

//
// ROS 
//
#include "ros/ros.h"

//
// RoadNarrows
//
#include "rnr/rnrconfig.h"
#include "rnr/log.h"
#include "rnr/opts.h"

//
// Node headers.
//
#include "hek_teleop.h"

using namespace ::std;
using namespace hekateros;
using namespace hekateros_control;


//. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 
// Node Specific Defines and Data
//. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 

//
// Application exit codes
//
#define APP_EC_OK   0   ///< success
#define APP_EC_INIT 2   ///< initialization fatal error
#define APP_EC_EXEC 4   ///< execution fatal error

#define NO_SIGNAL   0   ///< no signal receieved value

//
// Data
//
const char *NodeName  = "hek_teleop";  ///< this ROS node's name
static int  RcvSignal = NO_SIGNAL;      ///< received 'gracefull' signal



//. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 
// RoadNarrows Specific Defines and Data
//. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 

//
// Options
//

/*!
 * \brief The package information.
 *
 * For ROS nodes, RN package information is equivalent to this ROS application
 * information.
 * */
static const PkgInfo_T PkgInfo =
{
  NodeName,                       ///< package name
  "1.1.0",                        ///< package version
  "2014.03.18",                   ///< date (and time)
  "2014",                         ///< year
  NodeName,                       ///< package full name
  "Robin Knight, Daniel Packard", ///< authors
  "RoadNarrows LLC",              ///< owner
  "(C) 2013-2014 RoadNarrows LLC" ///< disclaimer
};

/*!
 * \brief Program information.
 */
static OptsPgmInfo_T AppPgmInfo =
{
  // usage_args
  "[ROSOPTIONS]",

  // synopsis
  "The %P ROS node provides ROS interfaces to the embedded Hekateros "
  "robotic manipulator.",
  
  // long_desc 
  "",
 
  // diagnostics
  NULL
};

/*!
 * \brief Command line options information.
 */
static OptsInfo_T AppOptsInfo[] =
{

  {NULL, }
};

/*!
 * \brief Signal handler to allow graceful shutdown of ROS node.
 *
 * \note This handler overrides the roscpp SIGINT handler.
 *
 * \param sig   Signal number.
 */
static void sigHandler(int sig)
{
  RcvSignal = sig;

  // All the default sigint handler does is call shutdown()
  //ros::shutdown();
}

/*!
 *  \brief ROS Hekateros teleoperation node main.
 *
 * \param argc    Command-line argument count.
 * \param argv    Command-line argument list.
 *
 * \return Returns exit code.
 */
int main(int argc, char *argv[])
{
  string  strNodeName;  // ROS-given node name
  double  hz = 30;
  int     rc;

  // 
  // Initialize the node. Parse the command line arguments and environment to
  // determine ROS options such as node name, namespace and remappings.
  // This call does not contact the master. This lets you use
  // ros::master::check() and other ROS functions after calling ros::init()
  // to check on the status of the master.
  //
  ros::init(argc, argv, NodeName);

  //
  // Parse node-specific options and arguments (from librnr).
  //
  OptsGet(NodeName, &PkgInfo, &AppPgmInfo, AppOptsInfo, true, &argc, argv);
 
  //
  //
  // A ctrl-c interrupt will stop attempts to connect to the ROS core.
  //
  ros::NodeHandle nh(NodeName);

  // actual ROS-given node name
  strNodeName = ros::this_node::getName();

  //
  // Failed to connect.
  //
  if( !ros::master::check() )
  {
    // add optional non-ROS unit tests here, then simply exit.
    return APP_EC_OK;
  }

  //
  // Signals
  //

  // Override the default ros sigint handler. This must be set after the first
  // NodeHandle is created.
  signal(SIGINT, sigHandler);

  // try to end safely with this signal
  signal(SIGTERM, sigHandler);

  ROS_INFO("%s: Node started.", strNodeName.c_str());
  
  //
  // Create a Hekateros teleoperation node object.
  //
  HekTeleop  teleop(nh, hz);

  //
  // Advertise server services.
  //
  teleop.advertiseServices();

  ROS_INFO("%s: Server services registered.", strNodeName.c_str());

  //
  // Initializedd client services.
  //
  teleop.clientServices();

  ROS_INFO("%s: Client services initialized.", strNodeName.c_str());

  //
  // Advertise publishers.
  //
  teleop.advertisePublishers();
  
  ROS_INFO("%s: Publishers registered.", strNodeName.c_str());
  
  //
  // Subscribed to topics.
  //
  teleop.subscribeToTopics();
  
  ROS_INFO("%s: Subscribed topics registered.", strNodeName.c_str());

  // set loop rate in Hertz
  ros::Rate loop_rate(hz);

  ROS_INFO("%s: Ready.", strNodeName.c_str());

  //
  // Main loop.
  //
  while( (RcvSignal == NO_SIGNAL) && ros::ok() )
  {
    // make any callbacks on pending ROS events
    ros::spinOnce(); 

    // check integrity of communications
    teleop.commCheck();

    // sleep to keep at loop rate
    loop_rate.sleep();
  }

  if( ros::ok() )
  {
    teleop.putRobotInSafeMode(true);
    usleep(1000000);
  }

  return APP_EC_OK;
}
