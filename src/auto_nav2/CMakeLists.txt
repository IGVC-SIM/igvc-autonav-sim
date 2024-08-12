cmake_minimum_required(VERSION 3.0.2)
project(auto_nav2)


find_package(catkin REQUIRED COMPONENTS
roscpp
rospy
std_msgs
sensor_msgs
nav_msgs
tf
tf2
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES auto_nav2
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)


include_directories(
include
${catkin_INCLUDE_DIRS}
)




add_executable(pub_goal src/pub_goal.cpp)
target_link_libraries(pub_goal ${catkin_LIBRARIES})



# Add this line to your target_link_libraries command
