cmake_minimum_required(VERSION 2.8.3)
project(trajectory_reshaping_world_logic)

add_definitions(-std=c++11)

find_package(gazebo REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  gazebo
  gazebo_msgs
  roscpp
)

catkin_package(
  CATKIN_DEPENDS
    gazebo_msgs
    roscpp
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp
  DEPENDS
)

include_directories(
  include/
  include ${catkin_INCLUDE_DIRS}
)

add_executable(trajectory_reshaping_world_logic_node
    src/trajectory_reshaping_world_logic.cpp
    src/trajectory_reshaping_world_logic_node.cpp
)
#add_dependencies(trajectory_reshaping_world_logic_node ${catkin_EXPORTED_TARGETS} )
target_link_libraries(trajectory_reshaping_world_logic_node ${catkin_LIBRARIES})

