#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp_v3/bt_factory.h>
#include "goat_behavior_tree/action_nodes.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    
    BT::BehaviorTreeFactory factory;
    
    // Register all custom action nodes
    factory.registerNodeType<goat_bt::NavigateToAction>("NavigateTo");
    // Register other action nodes...
    
    // Create the behavior tree from XML
    auto tree = factory.createTreeFromFile("path_to_your_tree.xml");
    
    // Execute the tree
    tree.tickRoot();
    
    rclcpp::shutdown();
    return 0;
} 