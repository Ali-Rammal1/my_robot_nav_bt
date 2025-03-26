#include "nav2_behavior_tree/bt_action_node.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

class NavigateToCharger : public nav2_behavior_tree::BtActionNode<nav2_msgs::action::NavigateToPose>
{
public:
  NavigateToCharger(const std::string &name, const BT::NodeConfiguration &config)
    : BtActionNode(name, "navigate_to_pose", config)
  {}

  static BT::PortsList providedPorts()
  {
    return nav2_behavior_tree::BtActionNode<nav2_msgs::action::NavigateToPose>::providedPorts();
  }

  void on_tick() override
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.pose.position.x = -0.31;
    pose.pose.position.y = 2.29;
    pose.pose.orientation.w = 1.0;
    goal_.pose = pose;
  }
};
