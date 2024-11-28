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
    
    // Get input ports
    request->target = getInput<std::string>("target").value();
    request->x = getInput<double>("x").value();
    request->y = getInput<double>("y").value();

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

PickAction::PickAction(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config)
{
    node_ = rclcpp::Node::make_shared("pick_action");
    client_ = node_->create_client<goat_behavior_tree::srv::Pick>("pick");
}

BT::NodeStatus PickAction::tick()
{
    auto request = std::make_shared<goat_behavior_tree::srv::Pick::Request>();
    request->object = getInput<std::string>("object").value();

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

BT::PortsList PickAction::providedPorts()
{
    return { BT::InputPort<std::string>("object") };
}

PlaceAction::PlaceAction(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config)
{
    node_ = rclcpp::Node::make_shared("place_action");
    client_ = node_->create_client<goat_behavior_tree::srv::Place>("place");
}

BT::NodeStatus PlaceAction::tick()
{
    auto request = std::make_shared<goat_behavior_tree::srv::Place::Request>();
    request->object = getInput<std::string>("object").value();
    request->location = getInput<std::string>("location").value();

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

BT::PortsList PlaceAction::providedPorts()
{
    return { 
        BT::InputPort<std::string>("object"),
        BT::InputPort<std::string>("location")
    };
}

LocateObjectAction::LocateObjectAction(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config)
{
    node_ = rclcpp::Node::make_shared("locate_object_action");
    client_ = node_->create_client<goat_behavior_tree::srv::LocateObject>("locate_object");
}

BT::NodeStatus LocateObjectAction::tick()
{
    auto request = std::make_shared<goat_behavior_tree::srv::LocateObject::Request>();
    request->object = getInput<std::string>("object").value();

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

BT::PortsList LocateObjectAction::providedPorts()
{
    return { BT::InputPort<std::string>("object") };
}

RequestAssistanceAction::RequestAssistanceAction(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config)
{
    node_ = rclcpp::Node::make_shared("request_assistance_action");
    client_ = node_->create_client<goat_behavior_tree::srv::RequestAssistance>("request_assistance");
}

BT::NodeStatus RequestAssistanceAction::tick()
{
    auto request = std::make_shared<goat_behavior_tree::srv::RequestAssistance::Request>();
    request->task = getInput<std::string>("task").value();

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

BT::PortsList RequestAssistanceAction::providedPorts()
{
    return { BT::InputPort<std::string>("task") };
}

WaitAction::WaitAction(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config)
{
    node_ = rclcpp::Node::make_shared("wait_action");
    client_ = node_->create_client<goat_behavior_tree::srv::Wait>("wait");
}

BT::NodeStatus WaitAction::tick()
{
    auto request = std::make_shared<goat_behavior_tree::srv::Wait::Request>();
    request->duration = getInput<float>("duration").value();

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

BT::PortsList WaitAction::providedPorts()
{
    return { BT::InputPort<float>("duration") };
}

ExecuteCommandAction::ExecuteCommandAction(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config)
{
    node_ = rclcpp::Node::make_shared("execute_command_action");
    client_ = node_->create_client<goat_behavior_tree::srv::ExecuteCommand>("execute_command");
}

BT::NodeStatus ExecuteCommandAction::tick()
{
    auto request = std::make_shared<goat_behavior_tree::srv::ExecuteCommand::Request>();
    request->command = getInput<std::string>("command").value();

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

BT::PortsList ExecuteCommandAction::providedPorts()
{
    return { BT::InputPort<std::string>("command") };
}

}  // namespace goat_bt 