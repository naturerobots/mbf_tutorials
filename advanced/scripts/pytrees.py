"""
MBF BT Demo: Behavior tree implementing a really basic navigation strategy,
even simpler than the move_base hardcoded FSM, as it lacks:

* continuous replanning
* oscillation detection

We create on the first place action client behaviors for MBF's planner, controller and recovery action servers
On this simple demo we need to add pretty little additional code to the base ActionClient class
"""

##############################################################################
# Imports
##############################################################################

import functools
import py_trees
import py_trees_ros
import py_trees.console as console
import rospy
import sys

import geometry_msgs.msg as geometry_msgs
import mbf_msgs.msg as mbf_msgs


##############################################################################
# Actions
##############################################################################

class GetPath(py_trees_ros.actions.ActionClient):

    def initialise(self):
        """
        Get target pose from the blackboard to create an action goal
        """
        self.action_goal = mbf_msgs.GetPathGoal(target_pose=py_trees.blackboard.Blackboard().get("target_pose"))
        super(GetPath, self).initialise()

    def update(self):
        """
        On success, set the resulting path on the blackboard, so ExePath can use it
        """
        status = super(GetPath, self).update()
        if status == py_trees.Status.SUCCESS:
            py_trees.blackboard.Blackboard().set("path", self.action_client.get_result().path)
        return status

class ExePath(py_trees_ros.actions.ActionClient):

    def initialise(self):
        """
        Get path from the blackboard to create an action goal
        """
        self.action_goal = mbf_msgs.ExePathGoal(path=py_trees.blackboard.Blackboard().get("path"))
        super(ExePath, self).initialise()

class Recovery(py_trees_ros.actions.ActionClient):
    def setup(self, timeout):
        """
        Read the list of available recovery behaviors so we can try them in sequence
        """
        self._behaviors = rospy.get_param("/move_base_flex/recovery_behaviors")
        return super(Recovery, self).setup(timeout)

    def update(self):
        """
        Try the next recovery behavior, dropping it from the list
        """
        try:
            self.action_goal = mbf_msgs.RecoveryGoal(behavior=self._behaviors.pop(0)["name"])
            return super(Recovery, self).update()
        except IndexError:
            # recovery behaviors exhausted; fail to abort navigation but restore the list for the next goal
            # TODO: this means that we won't reset the list after a successful recovery, so the list keeps shrinking
            # until fully exhausted; that's clearly not the expected operation, so I need to find a better solution
            self._behaviors = rospy.get_param("/move_base_flex/recovery_behaviors")
            return py_trees.Status.FAILURE


##############################################################################
# Behaviours
##############################################################################

def create_root():
    # Create all behaviours
    bt_root = py_trees.composites.Sequence("MBF BT Demo")
    get_goal = py_trees.composites.Selector("GetGoal")
    fallback = py_trees.composites.Selector("Fallback")
    navigate = py_trees.composites.Sequence("Navigate")
    new_goal = py_trees_ros.subscribers.ToBlackboard(name="NewGoal",
                                                     topic_name="/move_base_simple/goal",
                                                     topic_type=geometry_msgs.PoseStamped,
                                                     blackboard_variables = {'target_pose': None})
    have_goal = py_trees.blackboard.CheckBlackboardVariable(name="HaveGoal", variable_name="target_pose")
    clr_goal1 = py_trees.blackboard.ClearBlackboardVariable(name="ClearGoal", variable_name="target_pose")
    clr_goal2 = py_trees.blackboard.ClearBlackboardVariable(name="ClearGoal", variable_name="target_pose")
    get_path = GetPath(name="GetPath",
                       action_namespace="/move_base_flex/get_path",
                       action_spec=mbf_msgs.GetPathAction)
    exe_path = ExePath(name="ExePath",
                       action_namespace="/move_base_flex/exe_path",
                       action_spec=mbf_msgs.ExePathAction)
    recovery = Recovery(name="Recovery",
                        action_namespace="/move_base_flex/recovery",
                        action_spec=mbf_msgs.RecoveryAction)

    # Compose tree
    bt_root.add_children([get_goal, fallback])
    get_goal.add_children([have_goal, new_goal])
    navigate.add_children([get_path, exe_path, clr_goal1])
    fallback.add_children([navigate, recovery, clr_goal2])
    return bt_root


def shutdown(behaviour_tree):
    behaviour_tree.interrupt()

if __name__ == '__main__':
    rospy.init_node("mbf_bt_demo")
    root = create_root()
    behaviour_tree = py_trees_ros.trees.BehaviourTree(root)
    rospy.on_shutdown(functools.partial(shutdown, behaviour_tree))
    if not behaviour_tree.setup(timeout=15):
        console.logerror("failed to setup the tree, aborting.")
        sys.exit(1)

    behaviour_tree.tick_tock(500)