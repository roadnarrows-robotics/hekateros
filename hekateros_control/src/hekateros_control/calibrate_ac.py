##
## Calibrate action client
##
import roslib; roslib.load_manifest('hekateros_control')
import rospy
import actionlib

import hekateros_control.msg

class calibrate_ac(object):

    def __init__(self):
        self.c = actionlib.SimpleActionClient(
                    'hekateros_control/calibrate_as',
                    hekateros_control.msg.CalibrateAction)
                    
    def exec_calib(self, feedback_handler=None, timeout=1, force_recalib=True):
        if self.c.wait_for_server(rospy.Duration(timeout)):
            rospy.loginfo("Connected to calibrate action server!")
        else:
            rospy.logwarn("Unable to connect after {} seconds.".format(timeout))
            rospy.logwarn("Is hekateros_control node running???")
            return False;

        rospy.loginfo("Requesting calibrate action")
        goal = hekateros_control.msg.CalibrateGoal()
        if force_recalib:
          goal.force_recalib = 1
        else:
          goal.force_recalib = 0
        self.c.send_goal(goal, feedback_cb=feedback_handler)
        rospy.loginfo("Calibrating - ")
        return True

    def cancel(self):
        self.c.cancel_goal()

    def get_action_state(self):
        return self.c.get_state()
        pass

    def get_result(self):
        return self.c.get_result()
        pass

