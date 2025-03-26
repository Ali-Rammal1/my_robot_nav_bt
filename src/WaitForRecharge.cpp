#include "behaviortree_cpp_v3/action_node.h"
#include <chrono>
#include <thread>
#include <iostream>

class WaitForRecharge : public BT::SyncActionNode
{
public:
    WaitForRecharge(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts()
    {
        return {};
    }

    BT::NodeStatus tick() override
    {
        std::cout << "Recharging for 30 seconds..." << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(30));
        std::cout << "Recharge complete. Resetting battery count." << std::endl;

        int dummy;
        if (config().blackboard->get("battery_count", dummy)) {
            config().blackboard->set("battery_count", 0);
        }

        return BT::NodeStatus::SUCCESS;
    }
};
