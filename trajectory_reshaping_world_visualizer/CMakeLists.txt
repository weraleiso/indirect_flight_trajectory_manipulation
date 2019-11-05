cmake_minimum_required(VERSION 2.8.3)
project(trajectory_reshaping_world_visualizer)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs genmsg rospy tf tf_conversions)
find_package(OpenCV REQUIRED core highgui objdetect)

catkin_package(CATKIN_DEPENDS roscpp rospy tf tf_conversions)

include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
message(${OpenCV_INCLUDE_DIRS})

link_directories(/usr/lib/x86_64-linux-gnu)

add_executable(trajectory_reshaping_world_visualizer_node src/trajectory_reshaping_world_visualizer_node.cpp)
target_link_libraries(trajectory_reshaping_world_visualizer_node ${catkin_LIBRARIES} ${OpenCV_LIBS})
