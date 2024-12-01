#include <rclcpp/rclcpp.hpp>
#include "behaviortree_cpp_v3/bt_factory.h"
#include "goat_behavior/service_actions.hpp"
#include "goat_behavior/wait_action.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include <filesystem>

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("behavior_manager");

    // Create the behavior tree factory
    BT::BehaviorTreeFactory factory;

    // Register our custom nodes
    factory.registerNodeType<goat_behavior::WaitAction>("Wait");

    // Register PickAction node with node parameter
    factory.registerBuilder<goat_behavior::PickAction>(
        "Pick",
        [node](const std::string& name, const BT::NodeConfiguration& config) {
            return std::make_unique<goat_behavior::PickAction>(name, config, node);
        });

    // Register PlaceAction node
    factory.registerBuilder<goat_behavior::PlaceAction>(
        "Place",
        [node](const std::string& name, const BT::NodeConfiguration& config) {
            return std::make_unique<goat_behavior::PlaceAction>(name, config, node);
        });

    // Register NavigateAction node
    factory.registerBuilder<goat_behavior::NavigateAction>(
        "Navigate",
        [node](const std::string& name, const BT::NodeConfiguration& config) {
            return std::make_unique<goat_behavior::NavigateAction>(name, config, node);
        });

    // Get the package share directory
    std::string pkg_share_dir = ament_index_cpp::get_package_share_directory("goat_behavior");
    std::string bt_xml_path = pkg_share_dir + "/behavior_trees/main_tree.xml";

    if (!std::filesystem::exists(bt_xml_path)) {
        RCLCPP_ERROR(node->get_logger(), "Behavior tree XML file not found: %s", bt_xml_path.c_str());
        return 1;
    }

    RCLCPP_INFO(node->get_logger(), "Loading behavior tree from: %s", bt_xml_path.c_str());

    // Create the tree from the XML file using createTreeFromFile
    auto tree = factory.createTreeFromFile(bt_xml_path);

    RCLCPP_INFO(node->get_logger(), "Behavior tree created");

    // Run the tree
    BT::NodeStatus result = BT::NodeStatus::RUNNING;

    while (rclcpp::ok() && result == BT::NodeStatus::RUNNING) {
        result = tree.tickRoot();
        rclcpp::spin_some(node);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    // Log the final result
    switch (result) {
        case BT::NodeStatus::SUCCESS:
            RCLCPP_INFO(node->get_logger(), "Behavior tree completed successfully");
            break;
        case BT::NodeStatus::FAILURE:
            RCLCPP_ERROR(node->get_logger(), "Behavior tree failed");
            break;
        default:
            RCLCPP_WARN(node->get_logger(), "Behavior tree ended with unexpected status");
    }

    rclcpp::shutdown();
    return 0;
}
