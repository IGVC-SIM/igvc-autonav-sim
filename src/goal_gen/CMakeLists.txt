cmake_minimum_required(VERSION 3.0.2)
project(goal_gen)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf2
  tf2_ros
  tf2_geometry_msgs
  nav_msgs
  geometry_msgs
  visualization_msgs
)

## Declare a catkin package
catkin_package(
  CATKIN_DEPENDS roscpp tf2 tf2_ros tf2_geometry_msgs nav_msgs geometry_msgs visualization_msgs
)

## Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(${PROJECT_NAME}_node src/goal_gen_node.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)

## Add cmake target dependencies of the executable
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})