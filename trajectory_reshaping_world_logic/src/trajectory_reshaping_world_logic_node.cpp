
#include "trajectory_reshaping_world_logic/trajectory_reshaping_world_logic.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "talker");
  ros::NodeHandle nodeHandle;
  WorldLogic collapse_tree(nodeHandle);
  collapse_tree.run();
  return 0;
}
