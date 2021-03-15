#!/usr/bin/env python
#
# @author Julian Gaal
# License: 3-Clause BSD 

import roslib
import rospy
import smach
import tf
import actionlib
import mbf_msgs.msg as mbf_msgs
import geometry_msgs.msg as geometry_msgs


def create_pose_goal(x, y, z, xx, yy, zz, ww):
    goal = mbf_msgs.MoveBaseGoal()
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

def create_path_goal(path, tolerance_from_action, dist_tolerance, angle_tolerance):
    goal = mbf_msgs.ExePathGoal()
    goal.path = path
    goal.tolerance_from_action = tolerance_from_action
    goal.dist_tolerance = dist_tolerance
    goal.angle_tolerance = angle_tolerance
    return goal

def iterate_goals():
    goals = [   create_pose_goal(-1.75, 0.74, 0, 0, 0, 0.539, 0.843),
                create_pose_goal(-0.36, 1.92, 0, 0, 0, -0.020, 0.999),
                create_pose_goal(0.957, 1.60, 0, 0, 0, -0.163, 0.987),
                create_pose_goal(1.8741, 0.3830, 0, 0, 0, -0.70, 0.711),
                create_pose_goal(1.752, -0.928, 0, 0, 0, -0.856, 0.517),
                create_pose_goal(0.418, -2.116, 0, 0, 0, 0.998, 0.0619),
                create_pose_goal(-0.775, -1.80, 0, 0, 0, 0.954, 0.300),
                create_pose_goal(-1.990, -0.508, 0, 0, 0, -0.112, 0.999)
    ]

    for goal in goals:
        yield goal



class ExePath(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['success','fail'],
                             input_keys=['path'])

        self.mbf_ac = actionlib.SimpleActionClient("move_base_flex/exe_path", mbf_msgs.ExePathAction)
        self.mbf_ac.wait_for_server(rospy.Duration(10))
        rospy.loginfo("Connected to Move Base Flex exe_path server!")

    def move(self, planned_path):
        self.mbf_ac.send_goal(planned_path)
        self.mbf_ac.wait_for_result()
        return self.mbf_ac.get_result()

    def execute(self, userdata):
        planned_path = userdata.path
        target_pose = planned_path.poses[-1].pose

        rospy.loginfo("Attempting to reach (%1.3f, %1.3f)", target_pose.position.x, target_pose.position.y)

        result = self.move(create_path_goal(planned_path, True, 0.5, 3.14/18.0))

        if result.outcome != mbf_msgs.MoveBaseResult.SUCCESS:
            rospy.loginfo("Unable to complete path: %s", result.message)
            return 'fail'
    
        return 'success'

    def __del__(self):
        self.mbf_ac.cancel_all_goals()


class GetPath(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['success', 'fail', 'nogoalleft'],
                             input_keys=['goals','path'],
                             output_keys=['path'])
        self.listener = tf.TransformListener()
        self.mbf_gp_ac = actionlib.SimpleActionClient("move_base_flex/get_path", mbf_msgs.GetPathAction)

        
    def get_current_pos(self):
        now = rospy.Time()
        self.listener.waitForTransform("map", "base_link", now, rospy.Duration(3.0))
        (trans, rot) = self.listener.lookupTransform('map', 'base_link', now)
        
        pose = geometry_msgs.PoseStamped()
        pose.header.stamp = rospy.Time()
        pose.header.frame_id = 'map'
        pose.pose.position = geometry_msgs.Vector3(trans[0], trans[1], trans[2])
        pose.pose.orientation = geometry_msgs.Quaternion(rot[0], rot[1], rot[2], rot[3])
        return pose

    def get_path(self, goal):  
        target_pose = geometry_msgs.PoseStamped(goal.target_pose.header, goal.target_pose.pose)

        self.mbf_gp_ac.send_goal(
            mbf_msgs.GetPathGoal(start_pose=self.get_current_pos(), 
                                                target_pose=target_pose,
                                                use_start_pose=True, 
                                                tolerance=0.5))
        self.mbf_gp_ac.wait_for_result()
        result = self.mbf_gp_ac.get_result()

        rospy.loginfo("MBF get_path execution completed with result [%d]: %s", result.outcome, result.message)
        if result.outcome == mbf_msgs.GetPathResult.SUCCESS:
            return result.path
        
        return None

    def execute(self, userdata):
        try:
            goal = next(userdata.goals)
            
            userdata.path = self.get_path(goal)
            if not userdata.path:
                return 'fail'

            return 'success'

        except StopIteration:
            return 'nogoalleft'



if __name__ == '__main__':
    rospy.init_node('smach_circle_mbf')

    sm = smach.StateMachine(outcomes=['end'])
    sm.userdata.goals = iterate_goals()
    sm.userdata.path = None

    with sm:
        smach.StateMachine.add('GetPath', GetPath(), 
                        transitions={'success': 'ExePath',
                                     'fail': 'end',
                                     'nogoalleft': 'end'})

        smach.StateMachine.add('ExePath', ExePath(), 
                               transitions={'success':'GetPath', 
                                            'fail':'end'})


    outcome = sm.execute()
    rospy.signal_shutdown("All done.")

