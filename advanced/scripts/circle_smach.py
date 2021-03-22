import rospy
import smach
import smach_ros

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

from mbf_msgs.msg import ExePathAction
from mbf_msgs.msg import GetPathAction
from mbf_msgs.msg import RecoveryAction


def create_pose(x, y, z, xx, yy, zz, ww):
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.header.stamp = rospy.Time.now()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = z
    pose.pose.orientation.x = xx
    pose.pose.orientation.y = yy
    pose.pose.orientation.z = zz
    pose.pose.orientation.w = ww
    return pose


def iterate_target_poses():
    target_poses = [   
        create_pose(-1.75, 0.74, 0, 0, 0, 0.539, 0.843),
        create_pose(-0.36, 1.92, 0, 0, 0, -0.020, 0.999),
        create_pose(0.957, 1.60, 0, 0, 0, -0.163, 0.987),
        create_pose(1.8741, 0.3830, 0, 0, 0, -0.70, 0.711),
        create_pose(1.752, -0.928, 0, 0, 0, -0.856, 0.517),
        create_pose(0.418, -2.116, 0, 0, 0, 0.998, 0.0619),
        create_pose(-0.775, -1.80, 0, 0, 0, 0.954, 0.300),
        create_pose(-1.990, -0.508, 0, 0, 0, -0.112, 0.999)
    ]

    for target_pose in target_poses:
        yield target_pose

def create_path_goal(path, tolerance_from_action, dist_tolerance, angle_tolerance):
    goal = mbf_msgs.ExePathGoal()
    goal.path = path
    goal.tolerance_from_action = tolerance_from_action
    goal.dist_tolerance = dist_tolerance
    goal.angle_tolerance = angle_tolerance
    return goal

def main():
    rospy.init_node('mbf_state_machine')

    target_poses = iterate_target_poses()

    # Create SMACH state machine
    sm = smach.StateMachine(outcomes=['succeeded', 'aborted', 'preempted'])

    # Define userdata
    sm.userdata.goal = None
    sm.userdata.path = None
    sm.userdata.error = None
    sm.userdata.clear_costmap_flag = False
    sm.userdata.error_status = None

    with sm:
        # path callback
        def get_path_callback(userdata, goal):
            try:
                goal.target_pose = next(target_poses)
            except StopIteration:
                rospy.logwarn("Reached last target pose")
                rospy.signal_shutdown("Last goal reached. Shutting down")

        # Get path
        smach.StateMachine.add(
            'GET_PATH',
            smach_ros.SimpleActionState(
                '/move_base_flex/get_path',
                GetPathAction,
                goal_cb=get_path_callback,
                goal_slots=['target_pose'],
                result_slots=['path']
            ),
            transitions={
                'succeeded': 'EXE_PATH',
                'aborted': 'aborted',
                'preempted': 'preempted'
            },
            remapping={
                'target_pose': 'goal'
            }
        )

        def path_callback(userdata, goal):
            target_pose = goal.path.poses[-1].pose
            rospy.loginfo("Attempting to reach (%1.3f, %1.3f)", target_pose.position.x, target_pose.position.y)

        # Execute path
        smach.StateMachine.add(
            'EXE_PATH',
            smach_ros.SimpleActionState(
                '/move_base_flex/exe_path',
                ExePathAction,
                goal_cb=path_callback,
                goal_slots=['path']
            ),
            transitions={
                'succeeded': 'GET_PATH',
                'aborted': 'aborted',
                'preempted': 'preempted'
            }
        )

    # Execute SMACH plan
    # Create a thread to execute the smach container
    smach_thread = threading.Thread(target=sm.execute)
    smach_thread.start()

    # Wait for ctrl-c
    rospy.spin()

    # Request the container to preempt
    my_smach_con.request_preempt()

    # Block until everything is preempted 
    smach_thread.join()

if __name__=="__main__":
    main()