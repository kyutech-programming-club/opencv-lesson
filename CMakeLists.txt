cmake_minimum_required(VERSION 2.8.3)
project(opencv3mixing)

set(DEPEND_PACKS
  roscpp
  cv_bridge
  image_geometry
  image_transport
  sensor_msgs
  std_msgs
)

find_package(catkin REQUIRED COMPONENTS
  ${DEPEND_PACKS}
)

catkin_package(CATKIN_DEPENDS
  ${DEPEND_PACKS}
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(image_converter src/minimum.cpp)
target_link_libraries(image_converter ${catkin_LIBRARIES})
