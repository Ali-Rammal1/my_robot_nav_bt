#include "behaviortree_cpp_v3/condition_node.h"

class BatteryMonitor : public BT::ConditionNode
{
public:
    BatteryMonitor(const std::string& name, const BT::NodeConfiguration& config)
        : BT::ConditionNode(name, config) {}

    static BT::PortsList providedPorts()
    {
        return {};
    }

    BT::NodeStatus tick() override
    {
        int count = 0;
        if (config().blackboard->get("battery_count", count)) {
            if (count >= 4) {
                return BT::NodeStatus::SUCCESS; // battery low
            }
        }
        return BT::NodeStatus::FAILURE; // battery good
    }
};
