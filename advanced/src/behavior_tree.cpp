#include <fstream>
#include <cassert>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <behaviortree_cpp_v3/bt_factory.h>

BT::NodeStatus CheckBattery()
{
    std::cout << "[ Battery: OK ]" << std::endl;
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

std::vector<geometry_msgs::Pose> loadPoses(std::string filepath)
{
    std::ifstream infile(filepath);
    std::vector<geometry_msgs::Pose> poses(8);

    geometry_msgs::Pose pose;

    while (infile >> pose.position.x
                  >> pose.position.y
                  >> pose.position.z
                  >> pose.orientation.x
                  >> pose.orientation.y
                  >> pose.orientation.z
                  >> pose.orientation.w)
    {
        poses.push_back(pose);
    }

    return poses;
}

int main()
{
    auto poses = loadPoses(POSE_PATH);

    BT::BehaviorTreeFactory factory;

    factory.registerNodeType<ApproachObject>("ApproachObject");

    factory.registerSimpleCondition("CheckBattery", std::bind(CheckBattery));

    GripperInterface gripper;
    factory.registerSimpleAction("OpenGripper", 
                                 std::bind(&GripperInterface::open, &gripper));
    factory.registerSimpleAction("CloseGripper", 
                                 std::bind(&GripperInterface::close, &gripper));

    auto tree = factory.createTreeFromFile(BT_XML_PATH);
    tree.tickRoot();

    return 0;
}