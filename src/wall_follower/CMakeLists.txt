cmake_minimum_required(VERSION 2.8.3)
project(wall_follower)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)
## Declare a catkin package
catkin_package(
)

## Build wall_follower
include_directories(include ${catkin_INCLUDE_DIRS})
add_executable(wall_follower src/wall_follower.cpp)
target_link_libraries(wall_follower ${catkin_LIBRARIES})

## Build wallFollowing
add_executable(wallFollowing src/node_wallFollowing.cpp src/wallFollowing.cpp)
target_link_libraries(wallFollowing ${catkin_LIBRARIES})

