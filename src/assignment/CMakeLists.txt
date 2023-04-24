cmake_minimum_required(VERSION 2.8.3)
project(assignment)

add_definitions(-std=c++11)
find_package(cmake_modules REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread serialization)
find_package( catkin REQUIRED COMPONENTS
  moveit_core
  moveit_ros_planning
  roscpp
  rosconsole
  pluginlib
  tf
  dynamic_reconfigure
  eigen_conversions)

catkin_package(LIBRARIES assignment_context assignment_plugin)

include_directories(SYSTEM ${Boost_INCLUDE_DIRS})
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR})
include_directories( ${catkin_INCLUDE_DIRS} )
include_directories(${MOVEIT_LIB_NAME}_INCLUDE_DIRS)

link_directories(${Boost_LIBRARY_DIRS})

add_library( assignment_context src/assignment_context.cpp )
target_link_libraries( assignment_context ${MOVEIT_LIB_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_library( assignment_plugin src/assignment_plugin.cpp)
target_link_libraries( assignment_plugin assignment_context ${MOVEIT_LIB_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

