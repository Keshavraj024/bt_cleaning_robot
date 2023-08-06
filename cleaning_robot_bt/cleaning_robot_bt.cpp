#include <iostream>
#include <chrono>
#include "behaviortree_cpp/bt_factory.h"
#include <vector>

using namespace std::chrono_literals;

/**
 * Check if the room is found.
 *
 * @return BT::NodeStatus::FAILURE if the room is not found, indicating the action failed.
 */
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

    /**
     * Find the room to clean.
     *
     * @return BT::NodeStatus::SUCCESS once the room is found, indicating the action succeeded.
     */
    BT::NodeStatus tick() override
    {
        std::cout << "Finding room to clean " << std::endl;
        std::this_thread::sleep_for(3s);
        return BT::NodeStatus::SUCCESS;
    }
};

/**
 * Check if dirt is found.
 *
 * @return BT::NodeStatus::FAILURE if dirt is not found, indicating the action failed.
 */

BT::NodeStatus isDirtFound()
{
    std::cout << "Dirt not found " << std::endl;
    return BT::NodeStatus::FAILURE;
}

class FindDirt : public BT::SyncActionNode
{
public:
    explicit FindDirt(const std::string &name, const BT::NodeConfig &config) : BT::SyncActionNode{name, config}
    {
    }

    static BT::PortsList providedPorts()
    {
        return {BT::OutputPort<std::vector<double>>("dirt_position")};
    }

    /**
     * Find the dirt to clean.
     *
     * @return BT::NodeStatus::SUCCESS once the dirt is found, indicating the action succeeded.
     */
    BT::NodeStatus tick() override
    {
        std::cout << "Finding dirt to clean " << std::endl;
        std::vector<double> dirtPosition{1.0, 2.0, 3.0};
        BT::TreeNode::setOutput("dirt_position", dirtPosition);
        std::this_thread::sleep_for(3s);
        return BT::NodeStatus::SUCCESS;
    }
};

/**
 * Check if the dirt is close to the robot.
 *
 * @param tree The behavior tree node.
 * @return BT::NodeStatus::SUCCESS if the dirt is close to the robot, indicating the action succeeded.
 * @throws BT::RuntimeError if the required input "dirt_position" is missing.
 */
BT::NodeStatus isDirtClose(BT::TreeNode &tree)
{
    auto msg = tree.getInput<std::vector<double>>("dirt_position");

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
    explicit MoveToDirt(const std::string &name, const BT::NodeConfig &config) : BT::SyncActionNode{name, config}
    {
    }

    static BT::PortsList providedPorts()
    {
        return {BT::InputPort<std::vector<double>>("dirt_position")};
    }

    /**
     * Move to the dirt to clean.
     *
     * @return BT::NodeStatus::SUCCESS once the robot has moved to the dirt, indicating the action succeeded.
     * @throws BT::RuntimeError if the required input "dirt_position" is missing.
     */
    BT::NodeStatus tick() override
    {
        auto msg = getInput<std::vector<double>>("dirt_position");
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

/**
 * Check if the dirt is grasped by the robot.
 *
 * @return BT::NodeStatus::FAILURE if the dirt is not grasped, indicating the action failed.
 */
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

    /**
     * Grasp the dirt.
     *
     * @return BT::NodeStatus::SUCCESS once the dirt is grasped, indicating the action succeeded.
     */
    BT::NodeStatus tick() override
    {
        std::cout << "Grasping dirt " << std::endl;
        std::this_thread::sleep_for(3s);
        return BT::NodeStatus::SUCCESS;
    }
};

/**
 * Check if the bin is close to the robot.
 *
 * @return BT::NodeStatus::FAILURE if the bin is not close, indicating the action failed.
 */
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

    /**
     * Move to the bin.
     *
     * @return BT::NodeStatus::SUCCESS once the robot has moved to the bin, indicating the action succeeded.
     */

    BT::NodeStatus tick() override
    {
        std::cout << "Moving to bin" << std::endl;
        std::this_thread::sleep_for(3s);
        return BT::NodeStatus::SUCCESS;
    }
};

/**
 * Check if the dirt is placed in the trash.
 *
 * @return BT::NodeStatus::FAILURE if the dirt is not placed, indicating the action failed.
 */
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

    /**
     * Place the dirt in the trash.
     *
     * @return BT::NodeStatus::FAILURE once the dirt is placed in the trash, indicating the action failed.
     */

    BT::NodeStatus tick() override
    {
        std::cout << "Placing trash in bin" << std::endl;
        std::this_thread::sleep_for(3s);
        return BT::NodeStatus::SUCCESS;
    }
};

class AskForHelp : public BT::SyncActionNode
{
public:
    explicit AskForHelp(const std::string &name) : BT::SyncActionNode{name, {}}
    {
    }

    /**
     * Ask for human intervention and wait for 10 seconds.
     *
     * @return BT::NodeStatus::SUCCESS once the robot has asked for help, indicating the action succeeded.
     */
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
    factory.registerSimpleCondition("IsDirtClose", isDirtClose, dirtPosition);
    factory.registerNodeType<MoveToDirt>("MoveToDirt");

    factory.registerSimpleCondition("IsDirtGrasped", std::bind(isDirtGrasped));
    factory.registerNodeType<GraspDirt>("GraspDirt");

    factory.registerSimpleCondition("IsBinClose", std::bind(isBinClose));
    factory.registerNodeType<MoveToBin>("MoveToBin");

    factory.registerSimpleCondition("IsDirtPlaced", std::bind(isDirtPlaced));
    factory.registerNodeType<PlaceDirt>("PlaceDirt");

    factory.registerNodeType<AskForHelp>("AskForHelp");

    auto tree = factory.createTreeFromFile("../cleaning_robot_bt.xml");

    tree.tickWhileRunning();

    return 0;
}