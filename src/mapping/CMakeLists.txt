cmake_minimum_required(VERSION 3.0.2)
project(mapping)

find_package(catkin REQUIRED
tf
roscpp
rospy
cv_bridge
image_transport
std_msgs
sensor_msgs
geometry_msgs
nav_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES mapping
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)


include_directories(
	include
	${catkin_INCLUDE_DIRS}
)

link_directories(
	include
)


add_executable(RedirectScans src/redirect_scans.cpp)
target_link_libraries(RedirectScans ${catkin_LIBRARIES} )
