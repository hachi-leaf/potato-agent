cmake_minimum_required(VERSION 3.8)
project(potato_output)

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(potato_interface REQUIRED)

# 编译 output_info 节点
add_executable(output_info_node src/output_info_node.cpp)
ament_target_dependencies(output_info_node
  rclcpp
  rclcpp_action
  std_msgs
  potato_interface
)

# 编译 cmd_exec 节点
add_executable(cmd_exec_node src/cmd_exec_node.cpp)
ament_target_dependencies(cmd_exec_node
  rclcpp
  rclcpp_action
  std_msgs
  potato_interface
)

# 编译 file_edit 节点
add_executable(file_edit_node src/file_edit_node.cpp)
ament_target_dependencies(file_edit_node
  rclcpp
  rclcpp_action
  std_msgs
  potato_interface
)

# 编译 file_view 节点
add_executable(file_view_node src/file_view_node.cpp)
ament_target_dependencies(file_view_node
  rclcpp
  rclcpp_action
  std_msgs
  potato_interface
)

# 安装所有节点
install(TARGETS
  output_info_node
  cmd_exec_node
  file_edit_node
  file_view_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()