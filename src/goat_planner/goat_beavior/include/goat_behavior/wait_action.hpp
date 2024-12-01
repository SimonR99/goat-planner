#ifndef GOAT_BEHAVIOR_WAIT_ACTION_HPP_
#define GOAT_BEHAVIOR_WAIT_ACTION_HPP_

#include <string>
#include <chrono>
#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"

namespace goat_behavior
{

class WaitAction : public BT::SyncActionNode
{
public:
    WaitAction(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config)
    {
    }

    static BT::PortsList providedPorts()
    {
        return { BT::InputPort<double>("duration") };
    }

    BT::NodeStatus tick() override
    {
        double duration;
        if (!getInput("duration", duration)) {
            RCLCPP_ERROR(rclcpp::get_logger("wait_action"), "Missing required input [duration]");
            return BT::NodeStatus::FAILURE;
        }

        RCLCPP_INFO(rclcpp::get_logger("wait_action"), "Waiting for %.1f seconds", duration);
        
        // Convert duration to milliseconds and sleep
        std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(duration * 1000)));
        
        return BT::NodeStatus::SUCCESS;
    }
};

}  // namespace goat_behavior

#endif  // GOAT_BEHAVIOR_WAIT_ACTION_HPP_ 