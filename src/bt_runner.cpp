#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <thread>
#include <chrono>
#include <iostream>

#include "BatteryMonitor.cpp"
#include "NavigateToWaypoint.cpp"
#include "NavigateToCharger.cpp"
#include "WaitForRecharge.cpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("bt_runner_node");

    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<BatteryMonitor>("BatteryMonitor");
    factory.registerNodeType<NavigateToWaypoint>("NavigateToWaypoint");
    factory.registerNodeType<NavigateToCharger>("NavigateToCharger");
    factory.registerNodeType<WaitForRecharge>("WaitForRecharge");

    std::string pkg_share = ament_index_cpp::get_package_share_directory("my_robot_nav_bt");
    std::string bt_xml_file = pkg_share + "/behavior_trees/bt_nav.xml";
    std::cout << "Loading BT XML file from: " << bt_xml_file << std::endl;

    auto blackboard = BT::Blackboard::create();
    blackboard->set("node", node);
    blackboard->set("bt_loop_duration", std::chrono::milliseconds(100));
    blackboard->set("server_timeout", std::chrono::milliseconds(5000));
    blackboard->set("wait_for_service_timeout", std::chrono::milliseconds(5000));
    blackboard->set("battery_count", 0);  // Initial task count

    auto tree = factory.createTreeFromFile(bt_xml_file, blackboard);

    while (rclcpp::ok()) {
        tree.tickRoot();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    rclcpp::shutdown();
    return 0;
}
