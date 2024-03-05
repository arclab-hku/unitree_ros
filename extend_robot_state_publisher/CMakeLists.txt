cmake_minimum_required(VERSION 3.0.2)
project(extend_robot_state_publisher)

include(CheckCXXCompilerFlag)
if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif ()

if(MSVC)
  # MSVC does not support the Wextra flag
  add_compile_options(/Wall)
else()
  add_compile_options(-Wall -Wextra)
endif()

find_package(orocos_kdl REQUIRED)
find_package(catkin REQUIRED
  COMPONENTS kdl_parser roscpp rosconsole rostime sensor_msgs tf2_ros tf2_kdl
)
find_package(Eigen3 REQUIRED)

find_package(urdfdom_headers REQUIRED)

catkin_package(
  LIBRARIES ${PROJECT_NAME}_solver joint_state_listener
  INCLUDE_DIRS include
  DEPENDS kdl_parser orocos_kdl roscpp rosconsole rostime sensor_msgs tf2_ros tf2_kdl urdfdom_headers
)

include_directories(SYSTEM ${EIGEN3_INCLUDE_DIRS})
include_directories(include ${catkin_INCLUDE_DIRS} ${orocos_kdl_INCLUDE_DIRS} ${urdfdom_headers_INCLUDE_DIRS})
link_directories(${orocos_kdl_LIBRARY_DIRS})

add_library(${PROJECT_NAME}_solver
  src/robot_state_publisher.cpp
)
target_compile_features(${PROJECT_NAME}_solver PUBLIC cxx_std_14)
target_link_libraries(${PROJECT_NAME}_solver ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES})

add_library(joint_state_listener src/joint_state_listener.cpp)
target_link_libraries(joint_state_listener ${PROJECT_NAME}_solver ${orocos_kdl_LIBRARIES})

add_executable(${PROJECT_NAME} src/robot_state_publisher_node.cpp)
target_link_libraries(${PROJECT_NAME} ${PROJECT_NAME}_solver joint_state_listener ${orocos_kdl_LIBRARIES})

# Tests
