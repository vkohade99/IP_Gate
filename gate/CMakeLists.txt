cmake_minimum_required(VERSION 2.8)

project(gate)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg image_transport cv_bridge)
find_package(OpenCV REQUIRED)

## Declare a catkin package
catkin_package(CATKIN_DEPENDS std_msgs sensor_msgs)
## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(test2 src/test.cpp)
target_link_libraries(test2 ${catkin_LIBRARIES})
target_link_libraries(test2 ${OpenCV_LIBRARIES})
add_dependencies(test2 gate_generate_messages_cpp)


