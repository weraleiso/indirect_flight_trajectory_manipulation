
#include "trajectory_reshaping_test_node.h"

Test::Test() {
  // LOAD PARAMETERS
  // clang-format off
  nh.getParam("trajectory_reshaping_test_node/par_system/par_buffered_publish_rate", PAR_BUFFERED_PUBLISH_RATE);
  nh.getParam("trajectory_reshaping_test_node/par_geometry/par_takeoff_setpoint_x", PAR_TAKEOFF_SETPOINT_X);
  nh.getParam("trajectory_reshaping_test_node/par_geometry/par_takeoff_setpoint_y", PAR_TAKEOFF_SETPOINT_Y);
  nh.getParam("trajectory_reshaping_test_node/par_geometry/par_takeoff_setpoint_z", PAR_TAKEOFF_SETPOINT_Z);
  nh.getParam("trajectory_reshaping_test_node/par_geometry/par_buffered_setpoint_sat_x", PAR_BUFFERED_SETPOINT_SAT_X);
  nh.getParam("trajectory_reshaping_test_node/par_geometry/par_buffered_setpoint_sat_y", PAR_BUFFERED_SETPOINT_SAT_Y);
  nh.getParam("trajectory_reshaping_test_node/par_geometry/par_buffered_setpoint_sat_z", PAR_BUFFERED_SETPOINT_SAT_Z);
  nh.getParam("trajectory_reshaping_test_node/par_geometry/par_buffered_setpoint_sat_yaw", PAR_BUFFERED_SETPOINT_SAT_YAW);

  f = boost::bind(&Test::dyn_reconf_callback, this, _1, _2);
  server.setCallback(f);

  pub_posestmpd_bebop2camera_command = nh.advertise<geometry_msgs::PoseStamped>
          ("/reshaping/bebop2camera/command/pose", 1);
  sub_posestmpd_realistichandright_location = nh.subscribe(
      "realistichandright/htc_vive_controller_right/pose_stamped", 1, &Test::sub_posestmpd_realistichandright_location_callback, this);
  sub_pose_bebop2actor_location = nh.subscribe
          ("/reshaping/bebop2actor/ground_truth/pose", 1, &Test::sub_pose_bebop2actor_location_callback, this);
  sub_posearray_planner_global_path_points = nh.subscribe(
      "/planner/global_path_points", 1, &Test::sub_posearray_planner_global_path_points_callback, this);
  // clang-format on
}

void Test::sub_posestmpd_realistichandright_location_callback(
    const geometry_msgs::PoseStamped::ConstPtr& msg) {
  geometry_msgs::PoseStamped msg_current_pose = (*msg);
  msg_posestmpd_realistichandright_location = msg_current_pose;
}
void Test::sub_pose_bebop2actor_location_callback(
    const geometry_msgs::Pose::ConstPtr& msg) {
  geometry_msgs::Pose msg_current_pose = (*msg);
  msg_pose_bebop2actor_location = msg_current_pose;
}
void Test::sub_posearray_planner_global_path_points_callback(
    const geometry_msgs::PoseArray::ConstPtr& msg) {
  geometry_msgs::PoseArray msg_current_path_points = (*msg);
  msg_posearray_planner_global_path_points = msg_current_path_points;
}
/*TODO: void Test::sub_octomap_world_callback(...)
{
}*/

void Test::adaptViewpoint() {
  msg_posestmpd_bebop2camera_command.header.stamp = ros::Time::now();
  msg_posestmpd_bebop2camera_command.header.frame_id = "world";

  /*msg_posestmpd_bebop2camera_command.pose.position.x =msg_pose_bebop2actor_location.position.x - 2.0;
  msg_posestmpd_bebop2camera_command.pose.position.y =msg_pose_bebop2actor_location.position.y;
  msg_posestmpd_bebop2camera_command.pose.position.z =msg_pose_bebop2actor_location.position.z + 1.0;*/

  tf::Transform tf_bebop2actor;
  tf_bebop2actor.setOrigin(tf::Vector3(msg_pose_bebop2actor_location.position.x,msg_pose_bebop2actor_location.position.y,msg_pose_bebop2actor_location.position.z));
  tf_bebop2actor.setRotation(tf::Quaternion(msg_pose_bebop2actor_location.orientation.x,msg_pose_bebop2actor_location.orientation.y,msg_pose_bebop2actor_location.orientation.z,msg_pose_bebop2actor_location.orientation.w));

  tf::Transform tf_relative_rot;
  tf_relative_rot.setOrigin(tf::Vector3(0.0,0.0,0.0));
  tf::Quaternion q_rot;
  q_rot.setRPY(0.0,0.5,-0.3);
  tf_relative_rot.setRotation(q_rot);

  tf::Transform tf_relative_trans;
  tf_relative_trans.setOrigin(tf::Vector3(-2.5,0.0,0.0));
  tf::Quaternion q_rot_t;
  q_rot_t.setRPY(0.0,0.0,0.0);
  tf_relative_trans.setRotation(q_rot_t);

  tf_bebop2actor=tf_bebop2actor*tf_relative_rot*tf_relative_trans;

  msg_posestmpd_bebop2camera_command.pose.position.x = tf_bebop2actor.getOrigin().getX();
  msg_posestmpd_bebop2camera_command.pose.position.y = tf_bebop2actor.getOrigin().getY();
  msg_posestmpd_bebop2camera_command.pose.position.z = tf_bebop2actor.getOrigin().getZ();
  msg_posestmpd_bebop2camera_command.pose.orientation.w = tf_bebop2actor.getRotation().getW();
  msg_posestmpd_bebop2camera_command.pose.orientation.x = tf_bebop2actor.getRotation().getX();
  msg_posestmpd_bebop2camera_command.pose.orientation.y = tf_bebop2actor.getRotation().getY();
  msg_posestmpd_bebop2camera_command.pose.orientation.z = tf_bebop2actor.getRotation().getZ();

  // TODO: Add more powerful method here!

  pub_posestmpd_bebop2camera_command.publish(
      msg_posestmpd_bebop2camera_command);
}

void Test::dyn_reconf_callback(
    trajectory_reshaping_test::TestConfig& config,
    uint32_t level) {
  /*ROS_INFO("Reconfigure Request: %f %f %f",
        config.double_sat_x, config.double_sat_y,
        config.double_sat_z);*/
  PAR_BUFFERED_SETPOINT_SAT_X = config.double_sat_x;
  PAR_BUFFERED_SETPOINT_SAT_Y = config.double_sat_y;
  PAR_BUFFERED_SETPOINT_SAT_Z = config.double_sat_z;
  PAR_BUFFERED_SETPOINT_SAT_YAW = config.double_sat_yaw;
}

void Test::run() {
  ros::Rate loop_rate(PAR_BUFFERED_PUBLISH_RATE);

  while (ros::ok()) {
    adaptViewpoint();

    ros::spinOnce();
    loop_rate.sleep();
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "trajectory_reshaping_test_node");
  Test test;
  ROS_WARN("Initialized. About to start spinning...");
  test.run();
  ROS_WARN("Exiting...");
  return 0;
}
