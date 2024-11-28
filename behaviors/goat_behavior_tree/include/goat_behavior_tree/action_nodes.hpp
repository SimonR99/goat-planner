#ifndef GOAT_BEHAVIOR_TREE_ACTION_NODES_HPP_
#define GOAT_BEHAVIOR_TREE_ACTION_NODES_HPP_

#include <behaviortree_cpp_v3/behavior_tree.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <rclcpp/rclcpp.hpp>
#include "goat_behavior_tree/srv/navigate_to.hpp"
#include "goat_behavior_tree/srv/pick.hpp"
#include "goat_behavior_tree/srv/place.hpp"
#include "goat_behavior_tree/srv/locate_object.hpp"
#include "goat_behavior_tree/srv/request_assistance.hpp"
#include "goat_behavior_tree/srv/wait.hpp"
#include "goat_behavior_tree/srv/execute_command.hpp"

namespace goat_bt
{

class NavigateToAction : public BT::SyncActionNode
{
public:
    NavigateToAction(const std::string& name, const BT::NodeConfiguration& config);
    BT::NodeStatus tick() override;
    static BT::PortsList providedPorts()
    {
        return { 
            BT::InputPort<std::string>("target", "Target location name"),
            BT::InputPort<double>("x", "Target X coordinate"),
            BT::InputPort<double>("y", "Target Y coordinate")
        };
    }
private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Client<goat_behavior_tree::srv::NavigateTo>::SharedPtr client_;
};

class PickAction : public BT::SyncActionNode
{
public:
    PickAction(const std::string& name, const BT::NodeConfiguration& config);
    BT::NodeStatus tick() override;
    static BT::PortsList providedPorts();
private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Client<goat_behavior_tree::srv::Pick>::SharedPtr client_;
};

class PlaceAction : public BT::SyncActionNode
{
public:
    PlaceAction(const std::string& name, const BT::NodeConfiguration& config);
    BT::NodeStatus tick() override;
    static BT::PortsList providedPorts();
private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Client<goat_behavior_tree::srv::Place>::SharedPtr client_;
};

class LocateObjectAction : public BT::SyncActionNode
{
public:
    LocateObjectAction(const std::string& name, const BT::NodeConfiguration& config);
    BT::NodeStatus tick() override;
    static BT::PortsList providedPorts();
private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Client<goat_behavior_tree::srv::LocateObject>::SharedPtr client_;
};

class RequestAssistanceAction : public BT::SyncActionNode
{
public:
    RequestAssistanceAction(const std::string& name, const BT::NodeConfiguration& config);
    BT::NodeStatus tick() override;
    static BT::PortsList providedPorts();
private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Client<goat_behavior_tree::srv::RequestAssistance>::SharedPtr client_;
};

class WaitAction : public BT::SyncActionNode
{
public:
    WaitAction(const std::string& name, const BT::NodeConfiguration& config);
    BT::NodeStatus tick() override;
    static BT::PortsList providedPorts();
private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Client<goat_behavior_tree::srv::Wait>::SharedPtr client_;
};

class ExecuteCommandAction : public BT::SyncActionNode
{
public:
    ExecuteCommandAction(const std::string& name, const BT::NodeConfiguration& config);
    BT::NodeStatus tick() override;
    static BT::PortsList providedPorts();
private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Client<goat_behavior_tree::srv::ExecuteCommand>::SharedPtr client_;
};

}  // namespace goat_bt

#endif  // GOAT_BEHAVIOR_TREE_ACTION_NODES_HPP_ 