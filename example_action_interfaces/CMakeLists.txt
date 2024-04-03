cmake_minimum_required(VERSION 3.5)
project(example_action_interfaces)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(action_msgs REQUIRED)

# Generate action interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
    "action/Example.action"
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
