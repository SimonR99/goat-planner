#include "goat_behavior_tree/action_nodes.hpp"

namespace goat_bt
{

NavigateToAction::NavigateToAction(const std::string& name, const BT::NodeConfiguration& config)
    : BT::AsyncActionNode(name, config), aborted_(false)
{
    node_ = rclcpp::Node::make_shared("navigate_to_action");
    client_ = node_->create_client<goat_behavior_tree::srv::NavigateTo>("navigate_to");
    executor_.add_node(node_);
}

BT::NodeStatus NavigateToAction::tick()
{
    // If aborted, reset and return failure
    if (aborted_) {
        aborted_ = false;
        return BT::NodeStatus::FAILURE;
    }

    // Wait for service to be available
    if (!client_->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_ERROR(node_->get_logger(), "Navigation service not available");
        return BT::NodeStatus::FAILURE;
    }

    // If we haven't sent the request yet
    if (!future_response_.valid()) {
        auto request = std::make_shared<goat_behavior_tree::srv::NavigateTo::Request>();
        request->target = getInput<std::string>("target").value();
        request->x = getInput<double>("x").value();
        request->y = getInput<double>("y").value();

        RCLCPP_INFO(node_->get_logger(), "Sending navigation request to (%f, %f)", request->x, request->y);
        future_response_ = client_->async_send_request(request);
        return BT::NodeStatus::RUNNING;
    }

    // Check if the future is ready
    if (future_response_.wait_for(std::chrono::milliseconds(10)) == std::future_status::ready) {
        auto result = future_response_.get();
        // Reset the future so we can send new requests
        future_response_ = std::shared_future<typename goat_behavior_tree::srv::NavigateTo::Response::SharedPtr>();
        
        if (result->success) {
            RCLCPP_INFO(node_->get_logger(), "Navigation completed successfully: %s", result->message.c_str());
            return BT::NodeStatus::SUCCESS;
        } else {
            RCLCPP_ERROR(node_->get_logger(), "Navigation failed: %s", result->message.c_str());
            return BT::NodeStatus::FAILURE;
        }
    }

    // Keep spinning the executor while waiting
    executor_.spin_some();
    return BT::NodeStatus::RUNNING;
}

void NavigateToAction::halt()
{
    RCLCPP_INFO(node_->get_logger(), "Navigation halted");
    aborted_ = true;
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