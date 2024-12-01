#ifndef GOAT_BEHAVIOR_SERVICE_ACTIONS_HPP_
#define GOAT_BEHAVIOR_SERVICE_ACTIONS_HPP_

#include <string>
#include "goat_behavior/service_node_base.hpp"
#include "goat_behavior/srv/pick.hpp"
#include "goat_behavior/srv/place.hpp"
#include "goat_behavior/srv/navigate.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

namespace goat_behavior
{

// Pick Action
class PickAction : public ServiceNodeBase<goat_behavior::srv::Pick>
{
public:
    PickAction(const std::string& name, const BT::NodeConfiguration& config, rclcpp::Node::SharedPtr node)
        : ServiceNodeBase(name, config, node, "pick_object")
    {
    }

    static BT::PortsList providedPorts()
    {
        return { BT::InputPort<std::string>("object") };
    }

    BT::NodeStatus tick() override
    {
        std::string object;
        if (!getInput("object", object)) {
            RCLCPP_ERROR(node_->get_logger(), "Missing required input [object]");
            return BT::NodeStatus::FAILURE;
        }

        auto request = std::make_shared<goat_behavior::srv::Pick::Request>();
        request->object_name = object;

        if (call_service(request)) {
            RCLCPP_INFO(node_->get_logger(), "Pick action succeeded");
            return BT::NodeStatus::SUCCESS;
        } else {
            RCLCPP_ERROR(node_->get_logger(), "Pick action failed");
            return BT::NodeStatus::FAILURE;
        }
    }
};

// Place Action
class PlaceAction : public ServiceNodeBase<goat_behavior::srv::Place>
{
public:
    PlaceAction(const std::string& name, const BT::NodeConfiguration& config, rclcpp::Node::SharedPtr node)
        : ServiceNodeBase(name, config, node, "place_object")
    {
    }

    static BT::PortsList providedPorts()
    {
        return { 
            BT::InputPort<std::string>("object"),
            BT::InputPort<std::string>("location")
        };
    }

    BT::NodeStatus tick() override
    {
        std::string object, location;
        if (!getInput("object", object) || !getInput("location", location)) {
            RCLCPP_ERROR(node_->get_logger(), "Missing required inputs");
            return BT::NodeStatus::FAILURE;
        }

        auto request = std::make_shared<goat_behavior::srv::Place::Request>();
        request->object_name = object;
        request->location = location;

        if (call_service(request)) {
            RCLCPP_INFO(node_->get_logger(), "Place action succeeded");
            return BT::NodeStatus::SUCCESS;
        } else {
            RCLCPP_ERROR(node_->get_logger(), "Place action failed");
            return BT::NodeStatus::FAILURE;
        }
    }
};

// Navigation Action
class NavigateAction : public ServiceNodeBase<goat_behavior::srv::Navigate>
{
public:
    NavigateAction(const std::string& name, const BT::NodeConfiguration& config, rclcpp::Node::SharedPtr node)
        : ServiceNodeBase(name, config, node, "navigate_to_pose")
    {
    }

    static BT::PortsList providedPorts()
    {
        return { 
            BT::InputPort<double>("x", "X coordinate"),
            BT::InputPort<double>("y", "Y coordinate"),
            BT::InputPort<double>("qw", 1.0, "Quaternion w component"),
            BT::InputPort<double>("qx", 0.0, "Quaternion x component"),
            BT::InputPort<double>("qy", 0.0, "Quaternion y component"),
            BT::InputPort<double>("qz", 0.0, "Quaternion z component")
        };
    }

    BT::NodeStatus tick() override
    {
        double x, y;
        double qw = 1.0, qx = 0.0, qy = 0.0, qz = 0.0;

        if (!getInput("x", x) || !getInput("y", y)) {
            RCLCPP_ERROR(node_->get_logger(), "Missing required position inputs");
            return BT::NodeStatus::FAILURE;
        }

        // Get optional orientation inputs
        getInput("qw", qw);
        getInput("qx", qx);
        getInput("qy", qy);
        getInput("qz", qz);

        auto request = std::make_shared<goat_behavior::srv::Navigate::Request>();
        request->x = x;
        request->y = y;
        request->z = 0.0;
        request->qw = qw;
        request->qx = qx;
        request->qy = qy;
        request->qz = qz;

        if (call_service(request)) {
            RCLCPP_INFO(node_->get_logger(), "Navigation succeeded");
            return BT::NodeStatus::SUCCESS;
        } else {
            RCLCPP_ERROR(node_->get_logger(), "Navigation failed");
            return BT::NodeStatus::FAILURE;
        }
    }
};

// Add more service actions here...

}  // namespace goat_behavior

#endif  // GOAT_BEHAVIOR_SERVICE_ACTIONS_HPP_ 