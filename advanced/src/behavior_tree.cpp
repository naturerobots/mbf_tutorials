#include <fstream>
#include <ros/ros.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <actionlib/client/simple_action_client.h>
#include <mbf_msgs/MoveBaseAction.h>
#include <mbf_msgs/MoveBaseGoal.h>
#include <mbf_advanced/helpers.h>


BT::NodeStatus reachedHome()
{
    ROS_INFO_STREAM("Reached Home");
    return BT::NodeStatus::SUCCESS;
}

// We want to wrap into an ActionNode the methods open() and close()
class GripperInterface
{
public:
    GripperInterface(): _open(true) {}

    BT::NodeStatus open() {
        _open = true;
        std::cout << "GripperInterface::open" << std::endl;
        return BT::NodeStatus::SUCCESS;
    }

    BT::NodeStatus close() {
        std::cout << "GripperInterface::close" << std::endl;
        _open = false;
        return BT::NodeStatus::SUCCESS;
    }

private:
    bool _open; // shared information
};

class ApproachObject : public BT::SyncActionNode
{
  public:
    ApproachObject(const std::string& name) :
        BT::SyncActionNode(name, {})
    {
    }

    // You must override the virtual function tick()
    BT::NodeStatus tick() override
    {
        std::cout << "ApproachObject: " << this->name() << std::endl;
        return BT::NodeStatus::SUCCESS;
    }
};

struct MBFClient
{
    MBFClient(std::vector<mbf_msgs::MoveBaseGoal> pose_goals)
    : pose_goals(std::move(pose_goals))
    , ac("move_base_flex/move_base", true)
    {
        ac.waitForServer();
        ROS_INFO("Connected to MBF action server");
    }

    bool perform()
    {
        for (const auto& goal: pose_goals)
        {
            ROS_INFO_STREAM("Attempting to reach " << goal);
            auto result = *mbf_advanced::move(ac, goal);
            if (result.outcome != mbf_msgs::MoveBaseResult::SUCCESS)
            {
                ROS_ERROR_STREAM("Couldn't reach " << goal);
                return false;
            }
        }

        return true;
    }

    std::vector<mbf_msgs::MoveBaseGoal> pose_goals;
    actionlib::SimpleActionClient<mbf_msgs::MoveBaseAction> ac;
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "behavior_tree");
    ros::NodeHandle n;

    MBFClient mbfclient(std::move(mbf_advanced::loadPoseGoals(POSE_PATH)));
    mbfclient.perform();

    BT::BehaviorTreeFactory factory;

    factory.registerNodeType<ApproachObject>("ApproachObject");

    factory.registerSimpleCondition("CheckBattery", std::bind(reachedHome));

    GripperInterface gripper;
    factory.registerSimpleAction("OpenGripper", 
                                 std::bind(&GripperInterface::open, &gripper));
    factory.registerSimpleAction("CloseGripper", 
                                 std::bind(&GripperInterface::close, &gripper));

    auto tree = factory.createTreeFromFile(BT_XML_PATH);
    tree.tickRoot();

    return 0;
}