##
## Calibrate action client
##
import roslib; roslib.load_manifest('hekateros_control')
import rospy
import actionlib

import hekateros_control.msg

class calibrate_ac(object):

    def __init__(self, timeout=3):
        rospy.init_node("calibrate_ac")
        self.c = actionlib.SimpleActionClient(
                    'hekateros_control/calibrate_as',
                    hekateros_control.msg.CalibrateAction)
                    
    def request_calib(self, feedback_handler = None, timeout=1):
        rospy.loginfo("Connecting to calibrate action server...")
        if self.c.wait_for_server(rospy.Duration(timeout)):
            rospy.loginfo("Connected to calibrate action server!")
        else:
            rospy.logwarn("Unable to connect after %d seconds.")
            rospy.logwarn("Is hekateros_control node running???")
            return false;

        rospy.loginfo("Requesting calibrate action - please standby...")
        goal = hekateros_control.msg.CalibrateGoal()
        self.c.send_goal(goal, feedback_cb=feedback_handler)
        return True

    def cancel(self):
        c.cancel_goal()

    def get_action_state(self):
        return self.c.get_state()
        pass

