cmake_minimum_required(VERSION 3.8)
project(potato_core)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(potato_interface REQUIRED)   # 提供 Chat.action 和消息
find_package(nlohmann_json REQUIRED)
find_package(CURL REQUIRED)

# llmi 节点
add_executable(llmi_node src/llmi_node.cpp)
target_include_directories(llmi_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(llmi_node
  rclcpp
  rclcpp_action
  potato_interface
  nlohmann_json
  CURL
)

install(TARGETS llmi_node DESTINATION lib/${PROJECT_NAME})

ament_package()