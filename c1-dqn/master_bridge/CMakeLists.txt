cmake_minimum_required(VERSION 2.8.3)
project(master_bridge)

#set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
find_package(catkin REQUIRED COMPONENTS roscpp tf roslib std_msgs)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(master_bridge_libs src/shape_shifter.cpp)
target_link_libraries(master_bridge_libs ${catkin_LIBRARIES})

add_executable(master_bridge src/main.cpp src/multimaster.cpp src/relay_topic.cpp)
target_link_libraries(master_bridge master_bridge_libs ${catkin_LIBRARIES})
