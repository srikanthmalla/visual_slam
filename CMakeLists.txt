cmake_minimum_required(VERSION 2.8.3)
project(visual_slam)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  std_msgs
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

catkin_package(
   INCLUDE_DIRS include 
   CATKIN_DEPENDS roscpp sensor_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

# add_executable(synchronizer_node src/synchronizer_node.cpp src/Sync.cpp src/Filewriter.cpp)
# target_link_libraries(synchronizer_node ${catkin_LIBRARIES})