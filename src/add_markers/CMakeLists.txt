cmake_minimum_required(VERSION 2.8.3)
project(add_markers)

add_compile_options(-std=c++11)	# for std::vector initializations...

find_package(catkin REQUIRED COMPONENTS
  roscpp
  visualization_msgs
  actionlib
  move_base_msgs
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(add_markers src/add_markers.cpp)
target_link_libraries(add_markers ${catkin_LIBRARIES})

