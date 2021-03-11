#! /usr/bin/env python

import rospy
import actionlib
import mbf_msgs.msg as mbf_msgs
import move_base_msgs.msg as mb_msgs
from actionlib_msgs.msg import GoalStatus

def create_goal(x, y, z, xx, yy, zz, ww):
    goal = mb_msgs.MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.position.z = z
    goal.target_pose.pose.orientation.x = xx
    goal.target_pose.pose.orientation.y = yy
    goal.target_pose.pose.orientation.z = zz
    goal.target_pose.pose.orientation.w = ww
    return goal

def move(goal):
    client.send_goal(goal)
    client.wait_for_result()
    return client.get_state() == GoalStatus.SUCCEEDED


def drive_circle():
    goals = [   create_goal(-1.75, 0.74, 0, 0, 0, 0.539, 0.843),
                create_goal(-0.36, 1.92, 0, 0, 0, -0.020, 0.999),
                create_goal(0.957, 1.60, 0, 0, 0, -0.163, 0.987),
                create_goal(1.8741, 0.3830, 0, 0, 0, -0.70, 0.711),
                create_goal(1.752, -0.928, 0, 0, 0, -0.856, 0.517),
                create_goal(0.418, -2.116, 0, 0, 0, 0.998, 0.0619),
                create_goal(-0.775, -1.80, 0, 0, 0, 0.954, 0.300),
                create_goal(-1.990, -0.508, 0, 0, 0, -0.112, 0.999)
    ]

    for goal in goals:
        rospy.loginfo("Attempting to drive to %s %s", goal.target_pose.pose.position.x, goal.target_pose.pose.position.y)
        if not move(goal):
            return False

    return True

if __name__ == '__main__':
    try:
        rospy.init_node('mb_relay_client')
        
        client = actionlib.SimpleActionClient('move_base', mb_msgs.MoveBaseAction)
        client.wait_for_server(rospy.Duration(10))
        rospy.loginfo("Connected to SimpleActionServer 'move_base'")

        result = drive_circle()
        rospy.loginfo("Drove circle with result: %s", result)
        
    except rospy.ROSInterruptException:
        rospy.logerror("program interrupted before completion")