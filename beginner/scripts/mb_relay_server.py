#!/usr/bin/env python
#
# @author Jorge Santos
# License: 3-Clause BSD
# TODO how to credit 

import actionlib
import rospy
import mbf_msgs.msg as mbf_msgs
import move_base_msgs.msg as mb_msgs

def mb_execute_cb(msg):
    mbf_mb_ac.send_goal(mbf_msgs.MoveBaseGoal(target_pose=msg.target_pose),
                        feedback_cb=mbf_feedback_cb)

    rospy.logdebug("Relaying move_base goal to mbf")
    mbf_mb_ac.wait_for_result()

    status = mbf_mb_ac.get_state()
    result = mbf_mb_ac.get_result()

    rospy.logdebug("MBF execution completed with result [%d]: %s", result.outcome, result.message)
    if result.outcome == mbf_msgs.MoveBaseResult.SUCCESS:
        mb_as.set_succeeded(mb_msgs.MoveBaseResult(), "Goal reached.")
    else:
        mb_as.set_aborted(mb_msgs.MoveBaseResult(), result.message)

def mbf_feedback_cb(feedback):
    mb_as.publish_feedback(mb_msgs.MoveBaseFeedback(base_position=feedback.current_pose))

if __name__ == '__main__':
    rospy.init_node("move_base")

    # move_base_flex get_path and move_base action clients
    mbf_mb_ac = actionlib.SimpleActionClient("move_base_flex/move_base", mbf_msgs.MoveBaseAction)
    mbf_mb_ac.wait_for_server(rospy.Duration(10))

    mb_as = actionlib.SimpleActionServer('move_base', mb_msgs.MoveBaseAction, mb_execute_cb, auto_start=False)
    mb_as.start()

    rospy.spin()
