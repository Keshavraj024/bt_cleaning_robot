#include <iostream>
#include <chrono>
#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include <vector>

using namespace std::chrono_literals;

/* Find room subtree */
BT::NodeStatus isRoomFound()
{
    std::cout << "Room not found " << std::endl;
    return BT::NodeStatus::FAILURE;
}

class FindRoom : public BT::SyncActionNode
{
public:
    explicit FindRoom(const std::string &name) : BT::SyncActionNode{name, {}}
    {
    }

    BT::NodeStatus tick() override
    {
        std::cout << "Finding room to clean " << std::endl;
        std::this_thread::sleep_for(3s);
        return BT::NodeStatus::SUCCESS;
    }
};

/* Find dirt subtree*/
BT::NodeStatus isDirtFound()
{
    std::cout << "Dirt not found " << std::endl;
    return BT::NodeStatus::FAILURE;
}

class FindDirt : public BT::SyncActionNode
{
public:
    explicit FindDirt(const std::string &name, const BT::NodeConfiguration &config) : BT::SyncActionNode{name, config}
    {
    }

    static BT::PortsList providedPorts()
    {
        return { BT::OutputPort <std::vector<double>>("dirt_position") };
    }

    BT::NodeStatus tick() override
    {
        std::cout << "Finding dirt to clean " << std::endl;
        std::vector<double> dirtPosition{1.0, 2.0, 3.0};
        BT::TreeNode::setOutput("dirt_position", dirtPosition);
        std::this_thread::sleep_for(3s);
        return BT::NodeStatus::SUCCESS;
    }
};

/*Move to dirt subtree*/
BT::NodeStatus isTheDirtClose(BT::TreeNode &tree)
{
    BT::Optional<std::vector<double>> msg = tree.getInput<std::vector<double>>("dirt_position");

    if (!msg)
        throw BT::RuntimeError("Missing required input[message] : ", msg.error());

    if (msg.value().at(0) > 0.5)
    {
        std::cout << "Dirt is not close to the robot " << std::endl;
        return BT::NodeStatus::FAILURE;
    }
    else
        return BT::NodeStatus::SUCCESS;
}

class MoveToDirt : public BT::SyncActionNode
{
public:
    explicit MoveToDirt(const std::string &name, const BT::NodeConfiguration &config) : BT::SyncActionNode{name, config}
    {
    }

    static BT::PortsList providedPorts()
    {
        return {BT::InputPort<std::vector<double>>("dirt_position")};
    }

    BT::NodeStatus tick() override
    {
        BT::Optional<std::vector<double>> msg = getInput<std::vector<double>>("dirt_position");
        if (!msg)
            throw BT::RuntimeError("missing input[message] : ", msg.error());

        while (msg.value().at(0) > 0.5)
        {
            msg.value().at(0) -= 0.1;
            std::cout << "Moving to dirt to clean " << msg.value().at(0) << std::endl;
        }
        return BT::NodeStatus::SUCCESS;
    }
};

/*Grasp the dirt subtree*/
BT::NodeStatus isDirtGrasped()
{
    std::cout << "Dirt is not grasped " << std::endl;
    return BT::NodeStatus::FAILURE;
}

class GraspDirt : public BT::SyncActionNode
{
public:
    explicit GraspDirt(const std::string &name) : BT::SyncActionNode{name, {}}
    {
    }

    BT::NodeStatus tick() override
    {
        std::cout << "Grasping dirt " << std::endl;
        std::this_thread::sleep_for(3s);
        return BT::NodeStatus::SUCCESS;
    }
};

/*Move to bin*/
BT::NodeStatus isBinClose()
{
    std::cout << "Bin is not close to the robot " << std::endl;
    return BT::NodeStatus::FAILURE;
}

class MoveToBin : public BT::SyncActionNode
{
public:
    explicit MoveToBin(const std::string &name) : BT::SyncActionNode{name, {}}
    {
    }

    BT::NodeStatus tick() override
    {
        std::cout << "Moving to bin" << std::endl;
        std::this_thread::sleep_for(3s);
        return BT::NodeStatus::SUCCESS;
    }
};

/*Place the trash subtree*/
BT::NodeStatus isDirtPlaced()
{
    std::cout << "Dirt is not placed in the trash " << std::endl;
    return BT::NodeStatus::FAILURE;
}

class PlaceDirt : public BT::SyncActionNode
{
public:
    explicit PlaceDirt(const std::string &name) : BT::SyncActionNode{name, {}}
    {
    }

    BT::NodeStatus tick() override
    {
        std::cout << "Placing trash in bin" << std::endl;
        std::this_thread::sleep_for(3s);
        return BT::NodeStatus::FAILURE;
    }
};

/*ask help*/
class AskForHelp : public BT::SyncActionNode
{
public:
    explicit AskForHelp(const std::string &name) : BT::SyncActionNode{name, {}}
    {
    }

    BT::NodeStatus tick() override
    {
        std::cout << "Asking for human intervention. Waiting here for 10sec" << std::endl;
        std::this_thread::sleep_for(10s);
        return BT::NodeStatus::SUCCESS;
    }
};

int main()
{
    BT::BehaviorTreeFactory factory;

    factory.registerSimpleCondition("IsRoomFound", std::bind(isRoomFound));
    factory.registerNodeType<FindRoom>("FindRoom");

    factory.registerSimpleCondition("IsDirtFound", std::bind(isDirtFound));
    factory.registerNodeType<FindDirt>("FindDirt");

    BT::PortsList dirtPosition = {BT::InputPort<std::vector<double>>("dirt_position")};
    factory.registerSimpleCondition("IsTheDirtClose", isTheDirtClose, dirtPosition);
    factory.registerNodeType<MoveToDirt>("MoveToDirt");

    factory.registerSimpleCondition("IsDirtGrasped", std::bind(isDirtGrasped));
    factory.registerNodeType<GraspDirt>("GraspDirt");

    factory.registerSimpleCondition("IsBinClose", std::bind(isBinClose));
    factory.registerNodeType<MoveToBin>("MoveToBin");

    factory.registerSimpleCondition("IsDirtPlaced", std::bind(isDirtPlaced));
    factory.registerNodeType<PlaceDirt>("PlaceDirt");

    factory.registerNodeType<AskForHelp>("AskForHelp");

    auto tree = factory.createTreeFromFile("../bt.xml");

    tree.tickRoot();

    return 0;
}