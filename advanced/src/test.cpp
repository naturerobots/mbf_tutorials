#include <behaviortree_cpp_v3/bt_factory.h>

BT::NodeStatus start()
{
    std::cout << "start" << std::endl;
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus stop()
{
    std::cout << "stop" << std::endl;
    return BT::NodeStatus::FAILURE;
}

BT::NodeStatus final_stop()
{
    std::cout << "final_stop" << std::endl;
    return BT::NodeStatus::SUCCESS;
}

//class Repeat: BT::SimpleActionNode
//{
//    Repeat(const std::string& name, const BT::NodeConfiguration &config)
//    : BT::RepeatNode(name, config)
//    {}
//
//    static BT::PortsList providedPorts()
//    {
//        return { BT::InputPort<int>("n_tries")};
//    }
//
//    BT::NodeStatus tick() override
//    {
//        BT::Optional<int> msg = getInput<int>("n_tries");
//        if (!msg)
//        {
//            throw BT::RuntimeError("missing required input [n_tries]: ",
//                                   msg.error() );
//        }
//
//        // use the method value() to extract the valid message.
//        std::cout << "Robot says: " << msg.value() << std::endl;
//        return BT::NodeStatus::SUCCESS;
//    }
//};

int main(int argc, char** argv)
{
    BT::BehaviorTreeFactory factory;
    factory.registerSimpleCondition("start", std::bind(start));
    factory.registerSimpleCondition("stop", std::bind(stop));
//    factory.registerNodeType<Repeat>("Repeat");
    factory.registerSimpleCondition("final_stop", std::bind(final_stop));

    auto tree = factory.createTreeFromFile(BT_XML_PATH);
    tree.tickRoot();

    return 0;
}