#include "nav2_behavior_tree/bt_action_node.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <vector>
#include <utility>
#include <cstdlib>

class NavigateToWaypoint : public nav2_behavior_tree::BtActionNode<nav2_msgs::action::NavigateToPose>
{
public:
  NavigateToWaypoint(const std::string &name, const BT::NodeConfiguration &config)
    : BtActionNode(name, "navigate_to_pose", config)
  {
    waypoints_ = {
      {-2.0, -1.5}, {-0.58, -0.94}, {1.38, 0.065},
      {-0.33, 0.085}, {-1.92, -1.2}, {-0.87, 0.25},
      {-1.6, 1.9}, {-1.37, -1.6}, {-1.8, 0.72}
    };
  }

  static BT::PortsList providedPorts()
  {
    return nav2_behavior_tree::BtActionNode<nav2_msgs::action::NavigateToPose>::providedPorts();
  }

  void on_tick() override
  {
    auto chosen = waypoints_[rand() % waypoints_.size()];
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.pose.position.x = chosen.first;
    pose.pose.position.y = chosen.second;
    pose.pose.orientation.w = 1.0;
    goal_.pose = pose;
  }

  BT::NodeStatus on_success() override
  {
    int count = 0;
    if (config().blackboard->get("battery_count", count)) {
        count++;
        config().blackboard->set("battery_count", count);
        std::cout << "Completed goal. Battery count now: " << count << std::endl;
    }
    return BT::NodeStatus::SUCCESS;
  }

private:
  std::vector<std::pair<float, float>> waypoints_;
};
