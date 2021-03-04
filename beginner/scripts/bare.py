#!/usr/bin/env python

import random

import rospy
import actionlib

from geometry_msgs.msg import PoseStamped

from mag_common_py_libs.geometry import quaternion_from_yaw, pose2str

from mbf_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseResult

def create_geo_pose_stamped(x, y, yaw, frame_id='map'):
    pose = PoseStamped()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.orientation = quaternion_from_yaw(yaw)
    pose.header.frame_id = frame_id
    return pose

def nav_goal_cb(msg):
    goal = MoveBaseGoal(target_pose=msg)
    move_base_ac.send_goal(goal, done_cb=move_done_cb)
    rospy.loginfo("Calling MBF's move_base action with target pose %s", pose2str(msg))
    # TODO handle result

def move_done_cb(status, result):
    if result.outcome == MoveBaseResult.SUCCESS:
        rospy.loginfo("Follow path action succeeded")
    else:
        rospy.logerr("Follow path action failed with error code [%d]: %s", result.outcome, result.message)

if __name__ == '__main__':
    rospy.init_node("call_move_base_action")

    move_base_ac = actionlib.SimpleActionClient("/move_base_flex/move_base", MoveBaseAction)
    move_base_ac.wait_for_server(rospy.Duration(5))

    rospy.Subscriber('/move_base_simple/goal', PoseStamped, nav_goal_cb)

    rospy.spin()