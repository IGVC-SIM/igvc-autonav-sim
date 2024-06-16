cmake_minimum_required(VERSION 3.0.2)
project(filter_pc)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  pcl_ros
  pcl_conversions
)

catkin_package()

include_directories(  
  include
	${catkin_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
	${PCL_LIBRARY_DIRS}
)

link_directories(
	include
)

add_executable(filter src/fullnode.cpp)
target_link_libraries(filter ${catkin_LIBRARIES} ${PCL_LIBRARIES})