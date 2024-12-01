#ifndef GOAT_BEHAVIOR_SERVICE_NODE_BASE_HPP_
#define GOAT_BEHAVIOR_SERVICE_NODE_BASE_HPP_

#include <string>
#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"

namespace goat_behavior
{

template<typename ServiceT>
class ServiceNodeBase : public BT::SyncActionNode
{
public:
    ServiceNodeBase(const std::string& name, 
                   const BT::NodeConfiguration& config,
                   rclcpp::Node::SharedPtr node,
                   const std::string& service_name)
        : BT::SyncActionNode(name, config), 
          node_(node),
          service_name_(service_name)
    {
        client_ = node_->create_client<ServiceT>(service_name_);
    }

protected:
    bool call_service(typename ServiceT::Request::SharedPtr request)
    {
        if (!client_->wait_for_service(std::chrono::seconds(5))) {
            RCLCPP_ERROR(node_->get_logger(), "Service %s not available", service_name_.c_str());
            return false;
        }

        auto future = client_->async_send_request(request);

        if (rclcpp::spin_until_future_complete(node_, future) !=
            rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(node_->get_logger(), "Service call to %s failed", service_name_.c_str());
            return false;
        }

        response_ = future.get();
        return response_->success;
    }

    rclcpp::Node::SharedPtr node_;
    typename rclcpp::Client<ServiceT>::SharedPtr client_;
    typename ServiceT::Response::SharedPtr response_;
    std::string service_name_;
};

}  // namespace goat_behavior

#endif  // GOAT_BEHAVIOR_SERVICE_NODE_BASE_HPP_ 