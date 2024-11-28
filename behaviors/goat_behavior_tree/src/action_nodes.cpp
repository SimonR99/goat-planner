#include "goat_behavior_tree/action_nodes.hpp"

namespace goat_bt
{

NavigateToAction::NavigateToAction(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config)
{
    node_ = rclcpp::Node::make_shared("navigate_to_action");
    client_ = node_->create_client<goat_behavior_tree::srv::NavigateTo>("navigate_to");
}

BT::NodeStatus NavigateToAction::tick()
{
    auto request = std::make_shared<goat_behavior_tree::srv::NavigateTo::Request>();
    request->target = getInput<std::string>("target").value();

    auto future = client_->async_send_request(request);
    
    if (rclcpp::spin_until_future_complete(node_, future) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
        auto result = future.get();
        if (result->success) {
            return BT::NodeStatus::SUCCESS;
        }
    }
    return BT::NodeStatus::FAILURE;
}

BT::PortsList NavigateToAction::providedPorts()
{
    return { BT::InputPort<std::string>("target") };
}

// Implement other action nodes similarly...

}  // namespace goat_bt 