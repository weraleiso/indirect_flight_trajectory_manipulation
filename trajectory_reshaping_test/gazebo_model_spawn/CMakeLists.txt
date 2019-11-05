cmake_minimum_required(VERSION 2.8.3)
project(gazebo_model_spawn)

add_definitions(-std=c++11)

find_package(gazebo REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
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

add_executable(gazebo_model_spawn_node
    src/gazebo_model_spawn.cpp

    src/gazebo_model_spawn_node.cpp
)
#add_dependencies(gazebo_model_spawn_node ${catkin_EXPORTED_TARGETS} )
target_link_libraries(gazebo_model_spawn_node ${catkin_LIBRARIES})

