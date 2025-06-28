cmake_minimum_required(VERSION 3.0.2)
project(dummy_controller)
add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  simulation
)




catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES dummy_controller
#  CATKIN_DEPENDS mav_msgs roscpp std_msgs
#  DEPENDS system_lib
  CATKIN_DEPENDS simulation roscpp std_msgs
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


add_executable(dummy_controller_node src/dummy_controller_node.cpp)
target_link_libraries(dummy_controller_node ${catkin_LIBRARIES})

