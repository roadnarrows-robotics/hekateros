Hekateros Control Package
=============

The hekateros_control node provides the core functionality for the **ROS**
framework to interface with Hekateros robotic manipulator. The
hekateros_control node runs on target as a deamon process (along with roscore).

In addition to hekateros_control, two fundamental python application are also
included with this package:
* hek_panel - a GUI application that provides basic hekateros control and
monitoring.
* hek_teleop - a command-line application that provides teleoperation of the
Hekateros through an Xbox360 controller.
