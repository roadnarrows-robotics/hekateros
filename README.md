Hekateros
=============

A collection of [ROS](http://ros.org) packages for the Hekateros family of Robotic Manipulators.

Learn more about Hekateros on the [RoadNarrows R&D - Hekateros](http://roadnarrows.com/r-and-d/Hekateros/) page.

See the [RoadNarrows Store](http://www.roadnarrows-store.com/hekateros-arm.html) to get your very own!

#Quick Start:
These packages have been developed and tested under _Ubuntu 12.04 64-bit_.

* Install ROS on your system as described here: 
  * [http://www.ros.org/wiki/groovy/Installation/Ubuntu](http://www.ros.org/wiki/groovy/Installation/Ubuntu)
* Be sure to install utilities for the new _catkin_ build system
  * sudo apt-get install python-catkin python-wstool

* Install required RoadNarrows library dependencies:
  * TODO(dhp) - add instructions for roadnarrows apt repo)

* Create a catkin workspace somewhere on your system (e.g. in your home directory) and add the Hekateros ROS packages to the workspace:
  * mkdir ~/catkin_ws/src
  * cd ~/catkin_ws/src
  * catkin_init_workspace
  * wstool init
  * wstool set hekateros --git http://github.com/roadnarrows-robotics/hekateros

* Build the hekateros ROS packages:
  * cd ~/catkin_ws
  * catkin_make

* Source devel/setup.bash to add this workspace to your ROS search paths:
  * source devel/setup.bash

* Try some of the examples:
  * sim
  * minimal launch
  * ??? Chess ??? Dance ???




