cmake_minimum_required(VERSION 3.8)
project(goat_behavior_tree)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(ament_index_cpp REQUIRED)

# Generate interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/NavigateTo.srv"
  "srv/Pick.srv"
  "srv/Place.srv"
  "srv/LocateObject.srv"
  "srv/RequestAssistance.srv"
  "srv/Wait.srv"
  "srv/ExecuteCommand.srv"
  DEPENDENCIES geometry_msgs
)

# Include directories
include_directories(
  include
  ${behaviortree_cpp_v3_INCLUDE_DIRS}
)

# Create the behavior tree nodes library
add_library(goat_bt_nodes SHARED
  src/action_nodes.cpp
)

rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
target_link_libraries(goat_bt_nodes "${cpp_typesupport_target}")

ament_target_dependencies(goat_bt_nodes
  rclcpp
  behaviortree_cpp_v3
  std_msgs
  geometry_msgs
)

# Create the main executable
add_executable(goat_bt_main src/main.cpp)
target_link_libraries(goat_bt_main
  goat_bt_nodes
  ${behaviortree_cpp_v3_LIBRARIES}
)

ament_target_dependencies(goat_bt_main
  rclcpp
  behaviortree_cpp_v3
  ament_index_cpp
)

# Install C++ targets
install(TARGETS
  goat_bt_nodes
  goat_bt_main
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
  config
  DESTINATION share/${PROJECT_NAME}
)

# Python setup for scripts module
ament_python_install_package(scripts)

# Install Python scripts
install(PROGRAMS
  scripts/action_servers.py
  DESTINATION lib/${PROJECT_NAME}
)

# Linting and tests
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
