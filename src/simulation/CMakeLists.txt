cmake_minimum_required(VERSION 2.8.3)
project(simulation)

include(CheckCXXCompilerFlag)

CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)

if(COMPILER_SUPPORTS_CXX11)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wl,--no-as-needed")
elseif(COMPILER_SUPPORTS_CXX0X)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
endif()

find_package(catkin REQUIRED COMPONENTS
	geometry_msgs
	roscpp
	rospy
	sensor_msgs
	std_msgs
	tf
	image_transport
        cv_bridge
        message_generation
	)

add_message_files(
        FILES
        VehicleControl.msg
)
generate_messages(
        DEPENDENCIES
        std_msgs
)

catkin_package(CATKIN_DEPENDS
        message_runtime        
)

include(ExternalProject)
ExternalProject_Add(libsocket
  GIT_REPOSITORY https://github.com/dermesser/libsocket
  UPDATE_COMMAND ""
  CMAKE_ARGS -DCMAKE_INSTALL_PREFIX:PATH=${CATKIN_DEVEL_PREFIX} -DCMAKE_BUILD_TYPE:STRING=Release -DBUILD_SHARED_LIBS=ON
  BUILD_COMMAND make
  INSTALL_COMMAND make install
)

find_package(Eigen3)

link_directories(${CATKIN_DEVEL_PREFIX}/lib)
include_directories(${CATKIN_DEVEL_PREFIX}/include)
include_directories(${catkin_INCLUDE_DIRS} tcpimage)

add_subdirectory(tcpimage)

add_executable(unity_TCP_stream_receiver src/unity_TCP_stream_receiver.cpp)
add_executable(ROS_command_transmitter src/ROS_command_transmitter.cpp)
add_executable(JSON_param_reader src/JSON_param_reader.cpp)

target_link_libraries(unity_TCP_stream_receiver ${catkin_LIBRARIES} tcpstreamreader tcpimage)
target_link_libraries(ROS_command_transmitter ${catkin_LIBRARIES} socket++ -pthread)
target_link_libraries(JSON_param_reader  ${catkin_LIBRARIES})

add_dependencies(ROS_command_transmitter libsocket ${catkin_EXPORTED_TARGETS})

