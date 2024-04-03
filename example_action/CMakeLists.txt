cmake_minimum_required(VERSION 3.5)
project(example_action)

# Default to C++14
if (NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif ()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(action_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(example_action_interfaces REQUIRED)

# Dependencies
set(dependencies
    rclcpp
    rclcpp_action
    action_msgs
    example_action_interfaces
)

# Build action server
add_executable(example_action_server src/example_action_server.cpp)
ament_target_dependencies(example_action_server ${dependencies})

# Build action client
add_executable(example_action_client src/example_action_client.cpp)
ament_target_dependencies(example_action_client ${dependencies})

# Install executables and launch files
install(TARGETS
    example_action_server
    example_action_client
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
)

# Export dependencies
ament_export_dependencies(rosidl_default_runtime)
ament_package()
