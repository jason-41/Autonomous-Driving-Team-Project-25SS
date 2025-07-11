cmake_minimum_required(VERSION 3.0.2)
project(dummy_controller)

add_compile_options(-std=c++17)

# 查找依赖
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  simulation
  tf2
  tf2_ros
  geometry_msgs
  message_runtime
)

# 声明 catkin 包
catkin_package(
  CATKIN_DEPENDS roscpp std_msgs simulation tf2_ros geometry_msgs message_runtime
)

# 包含目录
include_directories(
  ${catkin_INCLUDE_DIRS}
)

# 原有节点
add_executable(dummy_controller_node src/dummy_controller_node.cpp)
target_link_libraries(dummy_controller_node ${catkin_LIBRARIES})
add_dependencies(dummy_controller_node ${catkin_EXPORTED_TARGETS})

# 安装
install(TARGETS 
  dummy_controller_node 
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

