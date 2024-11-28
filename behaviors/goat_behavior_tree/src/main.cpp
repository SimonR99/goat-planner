#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp_v3/bt_factory.h>
#include "goat_behavior_tree/action_nodes.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <filesystem>

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("goat_bt_main");
    
    BT::BehaviorTreeFactory factory;
    
    // Register all custom action nodes
    factory.registerNodeType<goat_bt::NavigateToAction>("NavigateTo");
    factory.registerNodeType<goat_bt::PickAction>("Pick");
    factory.registerNodeType<goat_bt::PlaceAction>("Place");
    factory.registerNodeType<goat_bt::LocateObjectAction>("LocateObject");
    factory.registerNodeType<goat_bt::RequestAssistanceAction>("RequestAssistance");
    factory.registerNodeType<goat_bt::WaitAction>("Wait");
    factory.registerNodeType<goat_bt::ExecuteCommandAction>("ExecuteCommand");
    
    // Get package share directory
    std::string pkg_share_dir = ament_index_cpp::get_package_share_directory("goat_behavior_tree");
    std::string bt_xml_path = pkg_share_dir + "/config/sample_tree.xml";
    
    try {
        // Create the behavior tree from XML
        auto tree = factory.createTreeFromFile(bt_xml_path);
        
        rclcpp::Rate rate(10);  // 10Hz
        BT::NodeStatus status;
        
        RCLCPP_INFO(node->get_logger(), "Starting behavior tree execution...");
        
        do {
            status = tree.tickRoot();
            rate.sleep();
        } while (status == BT::NodeStatus::RUNNING && rclcpp::ok());
        
        RCLCPP_INFO(node->get_logger(), "Behavior tree finished with status: %s", 
                   status == BT::NodeStatus::SUCCESS ? "SUCCESS" : "FAILURE");
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "Failed to load or execute behavior tree: %s", e.what());
        return 1;
    }
    
    rclcpp::shutdown();
    return 0;
} 