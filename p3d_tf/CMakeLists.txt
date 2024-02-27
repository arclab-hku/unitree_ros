cmake_minimum_required(VERSION 3.5.1)
project(gazebo_ros_p3d_tf)

find_package(catkin REQUIRED COMPONENTS
  gazebo_dev
  roscpp
  nav_msgs
  tf
  tf2_ros
)

find_package(Boost REQUIRED COMPONENTS thread)

include_directories(include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

link_directories(
  ${catkin_LIBRARY_DIRS}
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
  gazebo_ros_p3d_tf
  CATKIN_DEPENDS
  gazebo_dev
  roscpp
  nav_msgs
  tf
  tf2_ros
)

add_definitions(-fPIC) # what is this for?

## Plugins
add_library(gazebo_ros_p3d_tf src/gazebo_ros_p3d_tf.cpp)
target_link_libraries(gazebo_ros_p3d_tf ${catkin_LIBRARIES} ${Boost_LIBRARIES})


