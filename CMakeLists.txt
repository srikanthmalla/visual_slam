cmake_minimum_required(VERSION 2.8.3)
project(visual_slam)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  visualization_msgs
  geometry_msgs
  image_transport
  cv_bridge
)
# OPENCV 
find_package(OpenCV REQUIRED)
include_directories(SYSTEM ${OpenCV_INCLUDE_DIRS})

# CPP Version
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

catkin_package(
   INCLUDE_DIRS include 
   CATKIN_DEPENDS roscpp sensor_msgs
   DEPENDS OpenCV
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

add_executable(visual_slam_node src/visual_slam_node.cpp src/image_node.cpp)
target_link_libraries(visual_slam_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

# add_executable(tested src/test.cpp)
# target_link_libraries(tested ${OpenCV_LIBRARIES})