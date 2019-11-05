
#include "gazebo_model_spawn/gazebo_model_spawn.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "talker");

  ros::NodeHandle nodeHandle;
  GazeboModelSpawn modelSpawn(nodeHandle);
  ros::Rate loop_rate(50);

  while (ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
