Hekateros
=============

A collection of [ROS](http://ros.org) packages for the Hekateros family of Robotic Manipulators.

![Hekateros 5DOF Manipulator](http://www.roadnarrows.com/r-and-d/Hekateros/img/Hek_Reflect.png)

Learn more about Hekateros on the [RoadNarrows R&D - Hekateros](http://roadnarrows.com/r-and-d/Hekateros/) page.

Visit the [RoadNarrows Store](http://www.roadnarrows-store.com/hekateros-arm.html) to get your very own!

#Quick Start:
The hekateros ROS packages have been developed using:
 * _Ubuntu 12.04 64-bit_
 * _ROS Groovy Galapagos_ 

To get started with your robotic manipulator, or to try out some fun simulations:

* Install Ubuntu 12.04 (Long Term Support LTS) 64-Bit:
  * [ubuntu.com](http://www.ubuntu.com/download/desktop)

* Install supporting libraries and applications:

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

* Contribute!
  * When developing your own software for Hekateros, we recommend forking the official github repository. If you develop some new application or functionality, we would greatly appreciate notifying RoadNarrows by issuing a "pull request"



