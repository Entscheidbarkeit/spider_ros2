cmake_minimum_required(VERSION 3.8)
project(transformer_controller)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Always generate compile_commands.json for editor integration
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rosidl_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(transformer_hw_actuators REQUIRED)
find_package(transformer_hw_servos REQUIRED)

set(action_files
  "action/TransformMode.action"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${action_files}
  DEPENDENCIES builtin_interfaces
)

add_executable(transformer_controller_node src/transformer_controller_node.cpp)
target_include_directories(transformer_controller_node PUBLIC include)
ament_target_dependencies(transformer_controller_node rclcpp rclcpp_action std_msgs transformer_hw_actuators transformer_hw_servos)

# Replace deprecated rosidl_target_interfaces with modern pattern
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
target_link_libraries(transformer_controller_node "${cpp_typesupport_target}")

install(TARGETS transformer_controller_node DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY action launch config DESTINATION share/${PROJECT_NAME} OPTIONAL)
install(DIRECTORY include/ DESTINATION include)

ament_export_include_directories(include)
ament_export_dependencies(rosidl_default_runtime)

ament_package()
