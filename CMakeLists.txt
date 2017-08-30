cmake_minimum_required(VERSION 2.8.3)
project(opencv3mixing)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  cv_bridge
  image_geometry
  image_transport
  sensor_msgs
  std_msgs
)

catkin_package(CATKIN_DEPENDS
  roscpp
  cv_bridge
  image_geometry
  image_transport
  sensor_msgs
  std_msgs
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(minimum_node src/minimum.cpp)
target_link_libraries(minimum_node ${catkin_LIBRARIES})
