
#pragma once

#include <ros/ros.h>

#include <gazebo_msgs/SpawnModel.h>
#include <geometry_msgs/Pose.h>

/*#include <gazebo/SpawnModel.h>
#include <gazebo/urdf2gazebo.h>*/

class GazeboModelSpawn {
  ros::NodeHandle nodeHandle;

  ros::ServiceClient gazebo_spawn_client;

 public:
  GazeboModelSpawn(ros::NodeHandle &nh);
};
