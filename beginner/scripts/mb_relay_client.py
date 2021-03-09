#! /usr/bin/env python

import rospy
import actionlib
import mbf_msgs.msg as mbf_msgs
import move_base_msgs.msg as mb_msgs

def mb_relay_client():
    client = actionlib.SimpleActionClient('move_base', mb_msgs.MoveBaseAction)
    client.wait_for_server(rospy.Duration(10))

    rospy.loginfo("Connected to SimpleActionServer 'move_base'")

    goal = mb_msgs.MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = -1.990
    goal.target_pose.pose.position.y = -0.508
    goal.target_pose.pose.orientation.z = -0.112
    goal.target_pose.pose.orientation.w = 1.0

    client.send_goal(goal)
    client.wait_for_result()

    return client.get_result() 

if __name__ == '__main__':
    try:
        rospy.init_node('mb_relay_client')
        result = mb_relay_client()
        rospy.loginfo("MBF get_path execution completed with result [%d]: %s", result.outcome, result.message)
        
    except rospy.ROSInterruptException:
        rospy.logerror("program interrupted before completion")