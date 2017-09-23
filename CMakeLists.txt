cmake_minimum_required(VERSION 2.8.3)
project(opencv3mixing)
add_compile_options(-std=c++11)

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

add_executable(minimum_node src/minimum.cpp)
target_link_libraries(minimum_node ${catkin_LIBRARIES})

add_executable(blur_node src/blur.cpp)
target_link_libraries(blur_node ${catkin_LIBRARIES})

add_executable(mask_node src/color_detect.cpp)
target_link_libraries(mask_node ${catkin_LIBRARIES})
