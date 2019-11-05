
#include "trajectory_reshaping_adaptive_view_node.h"

AdaptiveView::AdaptiveView()//:octree_world(0.5)
{
  // LOAD PARAMETERS
  // clang-format off
  nh.getParam("trajectory_reshaping_adaptive_view_node/par_system/par_buffered_publish_rate", PAR_BUFFERED_PUBLISH_RATE);
  nh.getParam("trajectory_reshaping_adaptive_view_node/par_geometry/par_takeoff_setpoint_x", PAR_TAKEOFF_SETPOINT_X);
  nh.getParam("trajectory_reshaping_adaptive_view_node/par_geometry/par_takeoff_setpoint_y", PAR_TAKEOFF_SETPOINT_Y);
  nh.getParam("trajectory_reshaping_adaptive_view_node/par_geometry/par_takeoff_setpoint_z", PAR_TAKEOFF_SETPOINT_Z);
  nh.getParam("trajectory_reshaping_adaptive_view_node/par_geometry/par_buffered_setpoint_sat_x", PAR_BUFFERED_SETPOINT_SAT_X);
  nh.getParam("trajectory_reshaping_adaptive_view_node/par_geometry/par_buffered_setpoint_sat_y", PAR_BUFFERED_SETPOINT_SAT_Y);
  nh.getParam("trajectory_reshaping_adaptive_view_node/par_geometry/par_buffered_setpoint_sat_z", PAR_BUFFERED_SETPOINT_SAT_Z);
  nh.getParam("trajectory_reshaping_adaptive_view_node/par_geometry/par_buffered_setpoint_sat_yaw", PAR_BUFFERED_SETPOINT_SAT_YAW);

  nh.getParam("trajectory_reshaping_adaptive_view_node/par_system/par_head_pose_pitch_min", par_head_pose_pitch_min);
  nh.getParam("trajectory_reshaping_adaptive_view_node/par_system/par_head_pose_pitch_max", par_head_pose_pitch_max);
  nh.getParam("trajectory_reshaping_adaptive_view_node/par_system/par_view_distance_scale_min", par_view_distance_scale_min);
  nh.getParam("trajectory_reshaping_adaptive_view_node/par_system/par_view_distance_scale_max", par_view_distance_scale_max);

  f = boost::bind(&AdaptiveView::dyn_reconf_callback, this, _1, _2);
  server.setCallback(f);

  pub_posestmpd_current_view_distance = nh.advertise<geometry_msgs::PoseStamped>("/reshaping/bebop2camera/current_view_distance", 1);
  pub_posestmpd_bebop2camera_command = nh.advertise<geometry_msgs::PoseStamped>("/reshaping/bebop2camera/command/pose_to_lowpass", 1);
  sub_posestmpd_realistichandright_location = nh.subscribe("/reshaping/realistichandright/htc_vive_controller_right/pose_stamped", 1, &AdaptiveView::sub_posestmpd_realistichandright_location_callback, this);
  sub_pose_realistichandright_closest_point = nh.subscribe("/reshaping/realistichandright/closest_point", 1, &AdaptiveView::sub_pose_realistichandright_closest_point_callback, this);
  sub_pose_bebop2actor_location = nh.subscribe("/reshaping/bebop2actor/ground_truth/pose", 1, &AdaptiveView::sub_pose_bebop2actor_location_callback, this);
  sub_pose_bebop2camera_location = nh.subscribe("/reshaping/bebop2camera/ground_truth/pose", 1, &AdaptiveView::sub_pose_bebop2camera_location_callback, this);
  sub_pose_bebop2camera_desired_view = nh.subscribe("/reshaping/bebop2camera/desired_view", 1, &AdaptiveView::sub_pose_bebop2camera_desired_view_callback, this);
  sub_pose_bebop2actor_odometry = nh.subscribe("/reshaping/bebop2actor/ground_truth/odometry", 1, &AdaptiveView::sub_pose_bebop2actor_odometry_callback, this);
  sub_octomap_world = nh.subscribe("/octomap_full", 1, &AdaptiveView::sub_sub_octomap_world_callback, this);
  sub_posearray_planner_global_path_points = nh.subscribe("/planner/global_path_points", 1, &AdaptiveView::sub_posearray_planner_global_path_points_callback, this);
  // clang-format SUCKS A**! Why would you poke yourself with a stick??!

  msg_posestmpd_head_htc_vive_hmd.header.stamp = ros::Time::now();
  msg_posestmpd_head_htc_vive_hmd.header.frame_id = "reshaping/bebop2camera/htc_vive_hmd_base";
  msg_posestmpd_head_htc_vive_hmd.pose.position.x = 0.0;
  msg_posestmpd_head_htc_vive_hmd.pose.position.y = 0.0;
  msg_posestmpd_head_htc_vive_hmd.pose.position.z = 0.0;
  msg_posestmpd_head_htc_vive_hmd.pose.orientation.w = 1.0;
  msg_posestmpd_head_htc_vive_hmd.pose.orientation.x = 0.0;
  msg_posestmpd_head_htc_vive_hmd.pose.orientation.y = 0.0;
  msg_posestmpd_head_htc_vive_hmd.pose.orientation.z = 0.0;

  msg_posestmpd_head_htc_vive_hmd_world.header.stamp = ros::Time::now();
  msg_posestmpd_head_htc_vive_hmd_world.header.frame_id = "world";
  msg_posestmpd_head_htc_vive_hmd_world.pose.position.x = -1.0;
  msg_posestmpd_head_htc_vive_hmd_world.pose.position.y = 0.0;
  msg_posestmpd_head_htc_vive_hmd_world.pose.position.z = 1.0;
  msg_posestmpd_head_htc_vive_hmd_world.pose.orientation.w = 1.0;
  msg_posestmpd_head_htc_vive_hmd_world.pose.orientation.x = 0.0;
  msg_posestmpd_head_htc_vive_hmd_world.pose.orientation.y = 0.0;
  msg_posestmpd_head_htc_vive_hmd_world.pose.orientation.z = 0.0;

  octree_world=new octomap::OcTree(0.5);
}

void AdaptiveView::TransformToPose(tf::Transform& transform,
                                   geometry_msgs::Pose& pose) {
  pose.position.x = transform.getOrigin().getX();
  pose.position.y = transform.getOrigin().getY();
  pose.position.z = transform.getOrigin().getZ();

  pose.orientation.x = transform.getRotation().getX();
  pose.orientation.y = transform.getRotation().getY();
  pose.orientation.z = transform.getRotation().getZ();
  pose.orientation.w = transform.getRotation().getW();
}

void AdaptiveView::sub_posestmpd_realistichandright_location_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  geometry_msgs::PoseStamped msg_current_pose = (*msg);
  msg_posestmpd_realistichandright_location = msg_current_pose;
}
void AdaptiveView::sub_pose_realistichandright_closest_point_callback(const geometry_msgs::Pose::ConstPtr& msg)
{
  geometry_msgs::Pose msg_current_pose = (*msg);
  msg_pose_realistichandright_closest_point = msg_current_pose;
}

void AdaptiveView::sub_sub_octomap_world_callback(const octomap_msgs::Octomap::ConstPtr& msg)
{
    msg_octomap_world=(*msg);
    octomap::AbstractOcTree* abstract_tree = octomap_msgs::fullMsgToMap(msg_octomap_world);
    abstract_tree->setResolution(0.5);
    octree_world=(octomap::OcTree*)abstract_tree;
}
void AdaptiveView::sub_pose_bebop2actor_odometry_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
    msg_pose_bebop2actor_odometry=(*msg);
}
void AdaptiveView::sub_pose_bebop2camera_location_callback(const geometry_msgs::Pose::ConstPtr& msg)
{
    msg_pose_bebop2camera_location=(*msg);
}
void AdaptiveView::sub_pose_bebop2camera_desired_view_callback(const geometry_msgs::Pose::ConstPtr& msg)
{
    msg_pose_bebop2camera_desired_view=(*msg);
}

void AdaptiveView::sub_pose_bebop2actor_location_callback(const geometry_msgs::Pose::ConstPtr& msg)
{
    geometry_msgs::Pose msg_current_pose = (*msg);
    msg_pose_bebop2actor_location = msg_current_pose;

    /*
    tf::StampedTransform transform,transform_world;
    try
    {
        // Get transform locally in the HMD base frame
        tf_listener.lookupTransform("reshaping/bebop2camera/htc_vive_hmd_base","reshaping/bebop2camera/htc_vive_hmd",ros::Time(0),transform);

        msg_posestmpd_head_htc_vive_hmd.header.stamp = ros::Time::now();
        TransformToPose(transform, msg_posestmpd_head_htc_vive_hmd.pose);

        // Get transform in global world coordinates
        // We avoid using TF here!
        //tf_listener.lookupTransform("world","reshaping/bebop2camera/htc_vive_hmd",ros::Time(0),transform_world);
        //TransformToPose(transform_world, msg_posestmpd_head_htc_vive_hmd_world.pose);
        //msg_posestmpd_head_htc_vive_hmd_world.header.stamp = ros::Time::now();
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("%s", ex.what());

        msg_posestmpd_head_htc_vive_hmd.header.stamp = ros::Time::now();
        msg_posestmpd_head_htc_vive_hmd.header.frame_id = "reshaping/bebop2camera/htc_vive_hmd_base";
        msg_posestmpd_head_htc_vive_hmd.pose.position.x = 0.0;
        msg_posestmpd_head_htc_vive_hmd.pose.position.y = 0.0;
        msg_posestmpd_head_htc_vive_hmd.pose.position.z = 0.0;
        msg_posestmpd_head_htc_vive_hmd.pose.orientation.w = 1.0;
        msg_posestmpd_head_htc_vive_hmd.pose.orientation.x = 0.0;
        msg_posestmpd_head_htc_vive_hmd.pose.orientation.y = 0.0;
        msg_posestmpd_head_htc_vive_hmd.pose.orientation.z = 0.0;
    }*/
}
void AdaptiveView::sub_posearray_planner_global_path_points_callback(const geometry_msgs::PoseArray::ConstPtr& msg)
{
    geometry_msgs::PoseArray msg_current_path_points = (*msg);
    msg_posearray_planner_global_path_points = msg_current_path_points;
}

void AdaptiveView::adaptViewpoint()
{
  msg_posestmpd_bebop2camera_command.header.stamp = ros::Time::now();
  msg_posestmpd_bebop2camera_command.header.frame_id = "world";

  /*msg_posestmpd_bebop2camera_command.pose.position.x
  =msg_pose_bebop2actor_location.position.x - 2.0;
  msg_posestmpd_bebop2camera_command.pose.position.y
  =msg_pose_bebop2actor_location.position.y;
  msg_posestmpd_bebop2camera_command.pose.position.z
  =msg_pose_bebop2actor_location.position.z + 1.0;*/

  //tf::Quaternion quat_head_pose(msg_posestmpd_head_htc_vive_hmd.pose.orientation.x,msg_posestmpd_head_htc_vive_hmd.pose.orientation.y,msg_posestmpd_head_htc_vive_hmd.pose.orientation.z,msg_posestmpd_head_htc_vive_hmd.pose.orientation.w);
  tf::Quaternion quat_head_pose(msg_pose_bebop2camera_desired_view.orientation.x,msg_pose_bebop2camera_desired_view.orientation.y,msg_pose_bebop2camera_desired_view.orientation.z,msg_pose_bebop2camera_desired_view.orientation.w);
  //ROS_INFO("%3.3f,%3.3f,%3.3f,%3.3f",msg_pose_bebop2camera_desired_view.orientation.x,msg_pose_bebop2camera_desired_view.orientation.y,msg_pose_bebop2camera_desired_view.orientation.z,msg_pose_bebop2camera_desired_view.orientation.w);
  double head_yaw, head_pitch, head_roll;
  tf::Matrix3x3(quat_head_pose).getRPY(head_roll, head_pitch, head_yaw);
  // if(head_pitch>par_head_pose_pitch_max)head_pitch=par_head_pose_pitch_max;
  // if(head_pitch<par_head_pose_pitch_min)head_pitch=par_head_pose_pitch_min;
  if (head_pitch > 1.5) head_pitch = 1.5;
  if (head_pitch < 0.25) head_pitch = 0.25;
  //if (head_yaw < -1.5) head_yaw = -1.5;
  //if (head_yaw > 1.5) head_yaw = 1.5;
  //head_yaw=0.523598776; // FIXED FOR TESTING

  //head_pitch=0.785398163;// FIXED FOR TESTING

  tf::Transform tf_bebop2actor_location;
  tf_bebop2actor_location.setOrigin(
      tf::Vector3(msg_pose_bebop2actor_location.position.x,
                  msg_pose_bebop2actor_location.position.y,
                  msg_pose_bebop2actor_location.position.z));
  tf_bebop2actor_location.setRotation(
      tf::Quaternion(msg_pose_bebop2actor_location.orientation.x,
                     msg_pose_bebop2actor_location.orientation.y,
                     msg_pose_bebop2actor_location.orientation.z,
                     msg_pose_bebop2actor_location.orientation.w));

  tf::Transform tf_relative_rot;
  tf_relative_rot.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
  tf::Quaternion q_rot;
  //q_rot.setRPY(0.0, head_pitch, head_yaw);
  //q_rot.setRPY(0.0,M_PI/4.0,0.0); // OVERWRITE!
  double FOV_hmd=75.0;
  double beta=(45+(1/2+1/3)*FOV_hmd)*M_PI/180.0;
  double gamma=(1/3*FOV_hmd)*M_PI/180.0;
  double beta_hat=M_PI-beta;
  q_rot.setRPY(0.0,beta,0.0);
  tf_relative_rot.setRotation(q_rot);

  // Add adaptive distance depending on actors current hand projected on closest point (and min/max threshold)
  tf::Vector3 location_hand(msg_pose_realistichandright_closest_point.position.x,msg_pose_realistichandright_closest_point.position.y,msg_pose_realistichandright_closest_point.position.z);
  tf::Vector3 location_actor(msg_pose_bebop2actor_location.position.x,msg_pose_bebop2actor_location.position.y,msg_pose_bebop2actor_location.position.z);
  double d_hand=(location_hand-location_actor).length();
  if(d_hand<3.0)d_hand=3.0;
  if(d_hand>50.0)d_hand=50.0;

  tf::Vector3 tf_bebop2actor_speed(msg_pose_bebop2actor_odometry.twist.twist.linear.x,msg_pose_bebop2actor_odometry.twist.twist.linear.y,msg_pose_bebop2actor_odometry.twist.twist.linear.z);
  double vec_speed_length=fabs(tf_bebop2actor_speed.length());
  //ROS_INFO("Current speed of actor drone is: %3.3f",vec_speed_length);
  tf::Transform tf_relative_trans;
  tf_relative_trans.setOrigin(tf::Vector3(d_hand/sin(gamma)*sin(M_PI-(gamma+beta_hat)), 0.0, 0.0)); //OVERWRITE!
  //tf_relative_trans.setOrigin(tf::Vector3(-1.0*(7.5), 0.0, 0.0));
  tf::Quaternion q_rot_t(0.0,0.0,0.0,1.0);
  tf_relative_trans.setRotation(q_rot_t);

  tf::Transform tf_bebop2actor_command_to_bebop2camera;
  tf_bebop2actor_command_to_bebop2camera = tf_bebop2actor_location * tf_relative_rot * tf_relative_trans;

  tf::Quaternion q_rot_final=tf_bebop2actor_command_to_bebop2camera.getRotation();
  q_rot_final.setX(0.0);
  q_rot_final.setY(0.0);
  q_rot_final=q_rot_final.normalize();
  tf_bebop2actor_command_to_bebop2camera.setRotation(q_rot_final);
  TransformToPose(tf_bebop2actor_command_to_bebop2camera, msg_posestmpd_bebop2camera_command.pose);
  if(     msg_posestmpd_bebop2camera_command.pose.position.x!=msg_posestmpd_bebop2camera_command.pose.position.x ||
          msg_posestmpd_bebop2camera_command.pose.position.y!=msg_posestmpd_bebop2camera_command.pose.position.y ||
          msg_posestmpd_bebop2camera_command.pose.position.z!=msg_posestmpd_bebop2camera_command.pose.position.z ||
          msg_posestmpd_bebop2camera_command.pose.orientation.w!=msg_posestmpd_bebop2camera_command.pose.orientation.w ||
          msg_posestmpd_bebop2camera_command.pose.orientation.x!=msg_posestmpd_bebop2camera_command.pose.orientation.x ||
          msg_posestmpd_bebop2camera_command.pose.orientation.y!=msg_posestmpd_bebop2camera_command.pose.orientation.y ||
          msg_posestmpd_bebop2camera_command.pose.orientation.z!=msg_posestmpd_bebop2camera_command.pose.orientation.z)
  {
      return;
  }

  // Set constant for Proof of concept Lab experiment only
  /*
  msg_posestmpd_bebop2camera_command.pose.position.x=-1.23;
  msg_posestmpd_bebop2camera_command.pose.position.y=0.0;
  msg_posestmpd_bebop2camera_command.pose.position.z=2.8;
  msg_posestmpd_bebop2camera_command.pose.orientation.w=1.0;
  msg_posestmpd_bebop2camera_command.pose.orientation.x=0.0;
  msg_posestmpd_bebop2camera_command.pose.orientation.y=0.0;
  msg_posestmpd_bebop2camera_command.pose.orientation.z=0.0;
  */

  pub_posestmpd_bebop2camera_command.publish(msg_posestmpd_bebop2camera_command);

  //tf_relative_trans.setOrigin(tf::Vector3(-(par_view_distance_scale_min+par_view_distance_scale_max*fabs(tf_bebop2actor_speed.length()), 0.0, 0.0));
  /*if((ros::Time::now()-time_last_update).toSec()>=3.0) // Dont periodically update distance to prevent oscillations
  {
    tf_relative_trans.setOrigin(tf::Vector3(-1.0*(2.5+7.5*vec_speed_length), 0.0, 0.0));
    time_last_update=ros::Time::now();
  }
  else
  {
      tf_relative_trans.setOrigin(tf::Vector3(-1.0*(2.5+7.5*vec_speed_length), 0.0, 0.0));
  }*/
  //tf_relative_trans.setOrigin(tf::Vector3(-1.0*(2.5+7.5*0.8), 0.0, 0.0)); // Fixed for testing!



  // CALCULATE MAX VIEW DISTANCE BASED ON RAYCASTING AND PUBLISH TO FORCE PLUGIN TO SCALE HAND VECTOR!
  octomap::point3d p3d_view_origin;
  p3d_view_origin.x()=msg_pose_bebop2camera_location.position.x;//msg_posestmpd_head_htc_vive_hmd_world.pose.position.x;//tf_bebop2actor_command_to_bebop2camera.getOrigin().getX();
  p3d_view_origin.y()=msg_pose_bebop2camera_location.position.y;//msg_posestmpd_head_htc_vive_hmd_world.pose.position.y;//tf_bebop2actor_command_to_bebop2camera.getOrigin().getY();
  p3d_view_origin.z()=msg_pose_bebop2camera_location.position.z;//msg_posestmpd_head_htc_vive_hmd_world.pose.position.z;//tf_bebop2actor_command_to_bebop2camera.getOrigin().getZ();
  //tf::Quaternion quat_view_direction(msg_posestmpd_head_htc_vive_hmd_world.pose.orientation.x, msg_posestmpd_head_htc_vive_hmd_world.pose.orientation.y, msg_posestmpd_head_htc_vive_hmd_world.pose.orientation.z, msg_posestmpd_head_htc_vive_hmd_world.pose.orientation.w);
  tf::Quaternion quat_view_direction(msg_pose_bebop2camera_location.orientation.x, msg_pose_bebop2camera_location.orientation.y,msg_pose_bebop2camera_location.orientation.z,msg_pose_bebop2camera_location.orientation.w);
  tf::Vector3 vector_default(1.0, 0.0, 0.0);
  tf::Vector3 vec_view_direction = tf::quatRotate(quat_view_direction, vector_default);
  vec_view_direction=vec_view_direction.normalize(); // Normalize to be on the safe side although octomap API doesnt require it!

  octomap::point3d p3d_view_direction;
  p3d_view_direction.x()=vec_view_direction.getX();
  p3d_view_direction.y()=vec_view_direction.getY();
  p3d_view_direction.z()=vec_view_direction.getZ();

  octomap::point3d p3d_view_direction_max;
  bool result=octree_world->castRay(p3d_view_origin,p3d_view_direction,p3d_view_direction_max,true,150);

  tf::Vector3 vec_length_from_view_origin_to_max;
  vec_length_from_view_origin_to_max.setX(p3d_view_direction_max.x()-p3d_view_origin.x());
  vec_length_from_view_origin_to_max.setY(p3d_view_direction_max.y()-p3d_view_origin.y());
  vec_length_from_view_origin_to_max.setZ(p3d_view_direction_max.z()-p3d_view_origin.z());

  // Encode origin as usual, encode length into real part and view vector into imaginary parts!
  geometry_msgs::PoseStamped posestmpd_current_view_distance;
  posestmpd_current_view_distance.pose.position.x=p3d_view_origin.x();
  posestmpd_current_view_distance.pose.position.y=p3d_view_origin.y();
  posestmpd_current_view_distance.pose.position.z=p3d_view_origin.z();

  //posestmpd_current_view_distance.pose.orientation.w=fabs(vec_length_from_view_origin_to_max.length())*1.0; //For better "UX"
  posestmpd_current_view_distance.pose.orientation.w=100.0; //Overwrite with constant for better handling of hand pointer!

  posestmpd_current_view_distance.pose.orientation.x=p3d_view_direction.x();
  posestmpd_current_view_distance.pose.orientation.y=p3d_view_direction.y();
  posestmpd_current_view_distance.pose.orientation.z=p3d_view_direction.z();
  pub_posestmpd_current_view_distance.publish(posestmpd_current_view_distance);
}

void AdaptiveView::dyn_reconf_callback(trajectory_reshaping_adaptive_view::AdaptiveViewConfig& config,uint32_t level)
{
  /*ROS_INFO("Reconfigure Request: %f %f %f",
        config.double_sat_x, config.double_sat_y,
        config.double_sat_z);*/
  PAR_BUFFERED_SETPOINT_SAT_X = config.double_sat_x;
  PAR_BUFFERED_SETPOINT_SAT_Y = config.double_sat_y;
  PAR_BUFFERED_SETPOINT_SAT_Z = config.double_sat_z;
  PAR_BUFFERED_SETPOINT_SAT_YAW = config.double_sat_yaw;
}

void AdaptiveView::run()
{
    ros::Rate loop_rate(50.0);

    while (ros::ok())
    {
        adaptViewpoint();

        ros::spinOnce();
        loop_rate.sleep();
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "trajectory_reshaping_adaptive_view_node");
    AdaptiveView adaptview;
    ROS_WARN("Initialized. About to start spinning...");
    adaptview.run();
    ROS_WARN("Exiting...");
    return 0;
}
