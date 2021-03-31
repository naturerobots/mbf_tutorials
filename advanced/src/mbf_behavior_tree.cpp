#include <fstream>
#include <ros/ros.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <mbf_advanced/mbf_cpp_client.h>


BT::NodeStatus DriveHome(std::shared_ptr<mbf_advanced::MBFClient>& mbfclient)
{
    return mbfclient->driveHome() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

class AttemptNext : public BT::SyncActionNode
{
public:
    AttemptNext(const std::string& name)
      : SyncActionNode(name, {})
      , mbfclient_{}
    { }

    void attachMBFClient(std::shared_ptr<mbf_advanced::MBFClient> mbfclient)
    {
        mbfclient_ = mbfclient;
    }

    BT::NodeStatus tick() override
    {
        if (mbfclient_)
        {
            std::cout << this->name() << "\n";
            return mbfclient_->next_move() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
        }

        return BT::NodeStatus::FAILURE;
    }

private:
    std::shared_ptr<mbf_advanced::MBFClient> mbfclient_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "behavior_tree");
    ros::NodeHandle n;

    auto mbfclient = std::make_shared<mbf_advanced::MBFClient>(std::move(mbf_advanced::loadPoseGoals(POSE_PATH)));

    BT::BehaviorTreeFactory factory;
    factory.registerSimpleCondition("DriveHome", std::bind(DriveHome, std::ref(mbfclient)));
    factory.registerNodeType<AttemptNext>("AttemptNext");

    auto tree = factory.createTreeFromFile(BT_XML_PATH);

    for( auto& node: tree.nodes)
    {
        if( auto attempt_next = dynamic_cast<AttemptNext*>( node.get() ))
        {
            attempt_next->attachMBFClient(mbfclient);
        }
    }

    tree.tickRoot();

    return 0;
}