
#pragma once

#include <ros/ros.h>

#include <gazebo_msgs/SpawnModel.h>
#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/SetModelState.h>
#include <geometry_msgs/Pose.h>

/*#include <gazebo/SpawnModel.h>
#include <gazebo/urdf2gazebo.h>*/

class WorldLogic {
  ros::NodeHandle nodeHandle;

  ros::ServiceClient gazebo_set_model_state_client;

  ros::Subscriber sub_pose_bebop2camera;
  geometry_msgs::Pose msg_pose_bebop2camera;

  bool collapse_successfull;
  bool tree_collapsed;
  ros::Time time_tree_collapsed;

 public:
  WorldLogic(ros::NodeHandle &nh);

  void run();
  void callback_sub_pose_bebop2camera(const geometry_msgs::Pose::ConstPtr& msg);
  int CollapseTree();
};
