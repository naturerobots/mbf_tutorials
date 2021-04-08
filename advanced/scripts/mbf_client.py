import rospy
import mbf_msgs.msg as mbf_msgs

def create_goal(x, y, z, xx, yy, zz, ww):
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


def goal_generator():
    target_poses = [   
        create_goal(-1.75, 0.74, 0, 0, 0, 0.539, 0.843),
        create_goal(-0.36, 1.92, 0, 0, 0, -0.020, 0.999),
        create_goal(0.957, 1.60, 0, 0, 0, -0.163, 0.987),
        create_goal(1.8741, 0.3830, 0, 0, 0, -0.70, 0.711),
        create_goal(1.752, -0.928, 0, 0, 0, -0.856, 0.517),
        create_goal(0.418, -2.116, 0, 0, 0, 0.998, 0.0619),
        create_goal(-0.775, -1.80, 0, 0, 0, 0.954, 0.300),
        create_goal(-1.990, -0.508, 0, 0, 0, -0.112, 0.999)
    ]

    for target_pose in target_poses:
        yield target_pose


class Goals:
    def __init__(self):
        self.goals = [   
            create_goal(-1.75, 0.74, 0, 0, 0, 0.539, 0.843),
            create_goal(-0.36, 1.92, 0, 0, 0, -0.020, 0.999),
            create_goal(0.957, 1.60, 0, 0, 0, -0.163, 0.987),
            create_goal(1.8741, 0.3830, 0, 0, 0, -0.70, 0.711),
            create_goal(1.752, -0.928, 0, 0, 0, -0.856, 0.517),
            create_goal(0.418, -2.116, 0, 0, 0, 0.998, 0.0619),
            create_goal(-0.775, -1.80, 0, 0, 0, 0.954, 0.300),
            create_goal(-1.990, -0.508, 0, 0, 0, -0.112, 0.999)
        ]

        self.iterator = 0
        self.next_called = False

    def next(self):
        goal = self.goals[self.iterator]
        self.iterator += 1
        self.next_called = True

        if self.iterator > len(self.goals)-1:
            raise IndexError

        return goal

    def prev(self):
        if self.next_called:
            self.iterator -= 2
            self.next_called = False
        else:
            self.iterator -= 1
        
        if self.iterator < 0:
            raise IndexError

        return self.goals[self.iterator]

class MBFClient:
    def __init__(self):
        self.goals = Goals()

    def next(self):
        return self.goals.next()
       
    def prev(self):
        return self.goals.prev()

if __name__ == '__main__':
    rospy.init_node("move_base_flex_client")
    mbf = MBFClient()