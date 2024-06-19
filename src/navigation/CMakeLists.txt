cmake_minimum_required(VERSION 3.0.2)
project(navigation)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
roscpp
rospy
std_msgs
sensor_msgs
geometry_msgs
)

catkin_package()

include_directories(  
    include
    ${catkin_INCLUDE_DIRS}
)


link_directories(
	include
)


add_executable(gps src/gps.cpp)
target_link_libraries(gps ${catkin_LIBRARIES})
