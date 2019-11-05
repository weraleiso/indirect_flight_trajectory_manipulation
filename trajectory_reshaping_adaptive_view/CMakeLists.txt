cmake_minimum_required(VERSION 2.8.3)
project(trajectory_reshaping_adaptive_view)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs genmsg rospy tf tf_conversions dynamic_reconfigure octomap_ros)
find_package(OpenCV REQUIRED core highgui objdetect)

#add dynamic reconfigure api
generate_dynamic_reconfigure_options(
  cfg/AdaptiveView.cfg
)

catkin_package(CATKIN_DEPENDS dynamic_reconfigure roscpp rospy tf tf_conversions octomap_ros)

include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
message(${OpenCV_INCLUDE_DIRS})

link_directories(/usr/lib/x86_64-linux-gnu)

add_executable(trajectory_reshaping_adaptive_view_node src/trajectory_reshaping_adaptive_view_node.cpp)
target_link_libraries(trajectory_reshaping_adaptive_view_node ${catkin_LIBRARIES} ${OpenCV_LIBS})

# make sure configure headers are built before any node using them
add_dependencies(trajectory_reshaping_adaptive_view_node trajectory_reshaping_adaptive_view_gencfg)
