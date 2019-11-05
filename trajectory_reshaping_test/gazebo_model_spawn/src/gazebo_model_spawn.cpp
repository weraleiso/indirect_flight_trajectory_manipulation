
#include "gazebo_model_spawn/gazebo_model_spawn.h"

GazeboModelSpawn::GazeboModelSpawn(ros::NodeHandle &nh) : nodeHandle(nh) {
  gazebo_spawn_client = nodeHandle.serviceClient<gazebo_msgs::SpawnModel>(
      "/gazebo/spawn_urdf_model");

  // ROS_INFO("Gazebo model spawn started.");

  gazebo_msgs::SpawnModel model;
  geometry_msgs::Pose pose;
  geometry_msgs::Point point;
  point.x = 1;
  point.y = 0.5f;
  point.z = 0.3f;
  pose.position = point;
  model.request.initial_pose = pose;

  model.request.model_name = "Awesomium";

  std::ostringstream xml;
  xml << "<?xml version=\"1.0\"?><robot name=\"simple_box\"><link "
         "name=\"my_box\"><inertial><origin xyz=\"2 0 0\" /><mass "
         "value=\"1.0\" /><inertia  ixx=\"1.0\" ixy=\"0.0\"  ixz=\"0.0\"  "
         "iyy=\"100.0\"  iyz=\"0.0\"  izz=\"1.0\" /></inertial><visual><origin "
         "xyz=\"2 0 1\"/><geometry><box size=\"1 1 2\" "
         "/></geometry></visual><collision><origin xyz=\"2 0 "
         "1\"/><geometry><box size=\"1 1 2\" "
         "/></geometry></collision></link><gazebo "
         "reference=\"my_box\"><material>Gazebo/Blue</material></gazebo></"
         "robot>";
  model.request.model_xml = xml.str();

  gazebo_spawn_client.call(model);
}
