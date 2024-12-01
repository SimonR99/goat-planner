cmake_minimum_required(VERSION 3.8)
project(goat_behavior)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclpy REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(nav2_msgs REQUIRED)

# Generate interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/Pick.srv"
  "srv/Place.srv"
  "srv/Navigate.srv"
  DEPENDENCIES action_msgs
)

# Include directories
include_directories(
  include
  ${behaviortree_cpp_v3_INCLUDE_DIRS}
)

# Create the behavior tree nodes library
add_library(${PROJECT_NAME}_nodes SHARED
  src/behavior_manager.cpp
)

rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
target_link_libraries(${PROJECT_NAME}_nodes "${cpp_typesupport_target}")

ament_target_dependencies(${PROJECT_NAME}_nodes
  rclcpp
  rclcpp_action
  behaviortree_cpp_v3
  ament_index_cpp
  nav2_msgs
)

# Create the main executable
add_executable(behavior_manager src/behavior_manager.cpp)
target_link_libraries(behavior_manager
  ${PROJECT_NAME}_nodes
  ${behaviortree_cpp_v3_LIBRARIES}
)

ament_target_dependencies(behavior_manager
  rclcpp
  rclcpp_action
  behaviortree_cpp_v3
  ament_index_cpp
  nav2_msgs
)

# Install C++ targets
install(TARGETS
  ${PROJECT_NAME}_nodes
  behavior_manager
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install directories
install(DIRECTORY
  include/
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  behavior_trees
  DESTINATION share/${PROJECT_NAME}
)

# Python setup for scripts module
ament_python_install_package(scripts)

# Install Python scripts
install(PROGRAMS
  scripts/pick_service.py
  scripts/place_service.py
  scripts/navigate_service.py
  scripts/shepherd_connector.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
