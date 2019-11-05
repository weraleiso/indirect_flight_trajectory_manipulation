/*
 * Copyright 2013 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

/*
   Desc: Force Field based Indirect manipulation of UAV flight trajectories
   Author: W. A. Isop
   Date: 2018/2019
 */

#include <algorithm>
#include <assert.h>

#include <gazebo_plugins/gazebo_ros_force_gravity.h>

namespace gazebo
{
GZ_REGISTER_MODEL_PLUGIN(GazeboRosForceGravity);

////////////////////////////////////////////////////////////////////////////////
// Constructor
GazeboRosForceGravity::GazeboRosForceGravity()
{
  this->wrench_msg_.force.x = 0;
  this->wrench_msg_.force.y = 0;
  this->wrench_msg_.force.z = 0;
  this->wrench_msg_.torque.x = 0;
  this->wrench_msg_.torque.y = 0;
  this->wrench_msg_.torque.z = 0;
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboRosForceGravity::~GazeboRosForceGravity()
{
  event::Events::DisconnectWorldUpdateBegin(this->update_connection_);

  // Custom Callback Queue
  this->queue_.clear();
  this->queue_.disable();
  this->rosnode_->shutdown();
  this->callback_queue_thread_.join();

  delete this->rosnode_;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboRosForceGravity::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  // Get the world name.
  this->world_ = _model->GetWorld();
  
  // load parameters
  this->robot_namespace_ = "";
  if (_sdf->HasElement("robotNamespace"))
    this->robot_namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>() + "/";

  if (!_sdf->HasElement("bodyName"))
  {
    ROS_FATAL("force plugin missing <bodyName>, cannot proceed");
    return;
  }
  else
    this->link_name_ = _sdf->GetElement("bodyName")->Get<std::string>();

  this->link_ = _model->GetLink(this->link_name_);
  if (!this->link_)
  {
    ROS_FATAL("gazebo_ros_force plugin error: link named: %s does not exist\n",this->link_name_.c_str());
    return;
  }

  if (!_sdf->HasElement("topicName"))
  {
    ROS_FATAL("force plugin missing <topicName>, cannot proceed");
    return;
  }
  else
    this->topic_name_ = _sdf->GetElement("topicName")->Get<std::string>();

  this->model_trajectory_point_prefix_ = "mdl_trj_pt";
  if (_sdf->HasElement("modelTrajectoryPointPrefix"))
    this->model_trajectory_point_prefix_ = _sdf->GetElement("modelTrajectoryPointPrefix")->Get<std::string>() + "_";

  this->interaction_mode_ = 1;
  if (_sdf->HasElement("interactionMode"))
    this->interaction_mode_ = _sdf->GetElement("interactionMode")->Get<int>();

  this->force_mode_ = 2;
  if (_sdf->HasElement("forceMode"))
    this->force_mode_ = _sdf->GetElement("forceMode")->Get<int>();

  this->pushpull_mode_ = 1;
  if (_sdf->HasElement("pushpullMode"))
    this->pushpull_mode_ = _sdf->GetElement("pushpullMode")->Get<int>();

  this->use_hands_option_ = 1;
  if (_sdf->HasElement("useHandsOption"))
    this->use_hands_option_ = _sdf->GetElement("useHandsOption")->Get<int>();

  this->mirror_around_closest_point_ = 1;
  if (_sdf->HasElement("mirrorAroundClosestPoint"))
    this->mirror_around_closest_point_ = _sdf->GetElement("mirrorAroundClosestPoint")->Get<int>();

  this->hand_scale_translational_max_ = 75.0;
  if (_sdf->HasElement("handScaleTranslational"))
    this->hand_scale_translational_max_ = _sdf->GetElement("handScaleTranslationalMax")->Get<double>();

  this->force_min_distance_threshold_ = 0.5;
  if (_sdf->HasElement("forceMinDistanceThreshold"))
    this->force_min_distance_threshold_ = _sdf->GetElement("forceMinDistanceThreshold")->Get<double>();

  this->force_max_distance_threshold_ = 25.0;
  if (_sdf->HasElement("forceMaxDistanceThreshold"))
    this->force_max_distance_threshold_ = _sdf->GetElement("forceMaxDistanceThreshold")->Get<double>();

  this->force_velocity_scale_translational_max_ = 5.0;
  if (_sdf->HasElement("forceVelocityScaleTranslationalMax"))
    this->force_velocity_scale_translational_max_ = _sdf->GetElement("forceVelocityScaleTranslationalMax")->Get<double>();

  this->mockup_mode_ = 0;
  if (_sdf->HasElement("mockupMode"))
      this->mockup_mode_ = _sdf->GetElement("mockupMode")->Get<int>();

  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
      << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  this->rosnode_ = new ros::NodeHandle(this->robot_namespace_);

  // Custom Callback Queue
  ros::SubscribeOptions so = ros::SubscribeOptions::create<geometry_msgs::Wrench>(
    this->topic_name_,1,
    boost::bind( &GazeboRosForceGravity::UpdateObjectForce,this,_1),
    ros::VoidPtr(), &this->queue_);
  this->sub_ = this->rosnode_->subscribe(so);

  // Custom services and callbacks
  gazebo_spawn_model_client = this->rosnode_->serviceClient<gazebo_msgs::SpawnModel>("/reshaping/gazebo/spawn_urdf_model");
  gazebo_delete_model_client = this->rosnode_->serviceClient<gazebo_msgs::DeleteModel>("/reshaping/gazebo/delete_model");
  sub_pose_bebop2_camera = this->rosnode_->subscribe("/reshaping/bebop2camera/ground_truth/pose", 1, &GazeboRosForceGravity::sub_pose_bebop2_camera_callback, this);
  sub_posestmpd_bebop2_camera_current_view_distance = this->rosnode_->subscribe("/reshaping/bebop2camera/current_view_distance", 1, &GazeboRosForceGravity::sub_posestmpd_bebop2_camera_current_view_distance_callback, this);
  sub_pose_bebop2_actor = this->rosnode_->subscribe("/reshaping/bebop2actor/ground_truth/pose", 1, &GazeboRosForceGravity::sub_pose_bebop2_actor_callback, this);
  sub_trajectory_points_planner = this->rosnode_->subscribe("/planner/global_path_points", 1, &GazeboRosForceGravity::sub_trajectory_points_planner_callback, this);
  sub_trajectory_points_planner_interpolated_original = this->rosnode_->subscribe("/planner/global_path_points/interpolated_original", 1, &GazeboRosForceGravity::sub_trajectory_points_planner_interpolated_original_callback, this);
  pub_spawn_points_success = this->rosnode_->advertise<std_msgs::Bool>("/reshaping/gazebo/spawn_points_success", 0);
  pub_trajectory_points_planner_forces = this->rosnode_->advertise<geometry_msgs::PoseArray>("/planner/global_path_points/forces", 0);
  pub_poses_trajectory_reshaped = this->rosnode_->advertise<geometry_msgs::PoseArray>("/planner/global_path_points", 0);
  pub_pose_bebop2_camera_desired_view = this->rosnode_->advertise<geometry_msgs::Pose>("/reshaping/bebop2camera/desired_view", 0);
  pub_pose_trajectory_point_closest_to_hand = this->rosnode_->advertise<geometry_msgs::Pose>("/reshaping/realistichandright/closest_point", 0);
  pub_clicked_point_valid = this->rosnode_->advertise<geometry_msgs::PointStamped>("/clicked_point_valid", 0);
  //pub_posestmpd_actor_drone_location = this->rosnode_->advertise<geometry_msgs::PoseStamped>("/dronespace/mocapsystem/drone/location/pose", 0);
  //sub_pose_actor_drone_location =  this->rosnode_->subscribe("/reshaping/bebop2actor/ground_truth/pose", 1, &GazeboRosForceGravity::sub_pose_actor_drone_location_callback, this);
  //sub_posestmpd_hand_left = this->rosnode_->subscribe("/vrpn_client_node/DTrack/0/pose", 1, &GazeboRosForceGravity::sub_posestmpd_hand_left_callback, this);
  //sub_posestmpd_hand_right = this->rosnode_->subscribe("/vrpn_client_node/DTrack/1/pose", 1, &GazeboRosForceGravity::sub_posestmpd_hand_right_callback, this);
  sub_posestmpd_hand_left = this->rosnode_->subscribe("htc_vive_controller_left/pose_stamped", 1, &GazeboRosForceGravity::sub_posestmpd_hand_left_callback, this);
  sub_posestmpd_hand_right = this->rosnode_->subscribe("htc_vive_controller_right/pose_stamped", 1, &GazeboRosForceGravity::sub_posestmpd_hand_right_callback, this);
  sub_joy_hand_left = this->rosnode_->subscribe("htc_vive_controller_left/joy", 1, &GazeboRosForceGravity::sub_joy_hand_left_callback, this);
  sub_joy_hand_right = this->rosnode_->subscribe("htc_vive_controller_right/joy", 1, &GazeboRosForceGravity::sub_joy_hand_right_callback, this);
  trajectory_points_counter=0;
  lock_apply_force=true;
  //use_hands_option_=2; // Use both hands
  //force_max_distance_threshold_=5.0;

  msg_posestmpd_bebop2_camera_current_view_distance.header.stamp=ros::Time::now();
  msg_posestmpd_bebop2_camera_current_view_distance.header.frame_id="map";
  msg_posestmpd_bebop2_camera_current_view_distance.pose.position.x=-1.0;
  msg_posestmpd_bebop2_camera_current_view_distance.pose.position.y=0.0;
  msg_posestmpd_bebop2_camera_current_view_distance.pose.position.z=1.0;
  msg_posestmpd_bebop2_camera_current_view_distance.pose.orientation.w=1.0; // View vector length
  msg_posestmpd_bebop2_camera_current_view_distance.pose.orientation.x=1.0; // View vector direction
  msg_posestmpd_bebop2_camera_current_view_distance.pose.orientation.y=0.0;
  msg_posestmpd_bebop2_camera_current_view_distance.pose.orientation.z=0.0;

  yaw_desired_view=0.0;
  pitch_desired_view=0.25;
  msg_pose_desired_view.position.x=0.0;
  msg_pose_desired_view.position.y=0.0;
  msg_pose_desired_view.position.z=0.0;
  msg_pose_desired_view.orientation.w=1.0;
  msg_pose_desired_view.orientation.x=0.0;
  msg_pose_desired_view.orientation.y=0.0;
  msg_pose_desired_view.orientation.z=0.0;

  // Custom Callback Queue
  this->callback_queue_thread_ = boost::thread( boost::bind( &GazeboRosForceGravity::QueueThread,this ) );

  // New Mechanism for Updating every World Cycle
  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->update_connection_ = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&GazeboRosForceGravity::UpdateChild, this));
}

void GazeboRosForceGravity::sub_pose_bebop2_camera_callback(const geometry_msgs::Pose::ConstPtr& msg)
{
    msg_pose_bebop2_camera=*msg;
}
void GazeboRosForceGravity::sub_pose_bebop2_actor_callback(const geometry_msgs::Pose::ConstPtr& msg)
{
    msg_pose_bebop2_actor=*msg;
}

// Interpolated path points usef for MIRRORING force!
void GazeboRosForceGravity::sub_trajectory_points_planner_interpolated_original_callback(const geometry_msgs::PoseArray::ConstPtr& msg)
{
    msg_planner_path_points_interpolated_original=*msg;
    ROS_WARN("Received interpol. POINTS FOR MIRROR!");
}

// Receive global path points from planner and spawn points as models
void GazeboRosForceGravity::sub_trajectory_points_planner_callback(const geometry_msgs::PoseArray::ConstPtr& msg)
{
    ROS_INFO("Received new trajectory! About to spawn points as models...");
    if(msg->poses[0].orientation.w==-1.0 &&
        msg->poses[0].orientation.w==-1.0 &&
        msg->poses[0].orientation.w==-1.0 &&
        msg->poses[0].orientation.w==-1.0)
    {
        ROS_INFO("Planner reported non-feasable path! Aborting spawning...");
        return;
    }


    // Sync. with Gazebo related thread (Update child)
    lock_apply_force=true;

    msg_poses_trajectory_points = (*msg);

    int trajectory_points_counter_new=msg_poses_trajectory_points.poses.size();

    if(trajectory_points_counter_new==trajectory_points_counter)
    {
        for(int i=0;i<trajectory_points_counter;i++)
        {
            std::stringstream str_model_name;
            str_model_name << model_trajectory_point_prefix_ << std::setw(5) << std::setfill('0') << i;
            gazebo::physics::WorldPtr world = this->world_;
            gazebo::physics::ModelPtr mdl_traj_point = world->GetModel(str_model_name.str());
            gazebo::physics::LinkPtr link_trj_pt = mdl_traj_point->GetLink("base_link");
            math::Pose pose_trajectory_point_current;
            pose_trajectory_point_current.pos.x=msg_poses_trajectory_points.poses[i].position.x;
            pose_trajectory_point_current.pos.y=msg_poses_trajectory_points.poses[i].position.y;
            pose_trajectory_point_current.pos.z=msg_poses_trajectory_points.poses[i].position.z;
            pose_trajectory_point_current.rot.w=msg_poses_trajectory_points.poses[i].orientation.w;
            pose_trajectory_point_current.rot.x=msg_poses_trajectory_points.poses[i].orientation.x;
            pose_trajectory_point_current.rot.y=msg_poses_trajectory_points.poses[i].orientation.y;
            pose_trajectory_point_current.rot.z=msg_poses_trajectory_points.poses[i].orientation.z;
            link_trj_pt->SetWorldPose(pose_trajectory_point_current);
        }
    }
    else if(trajectory_points_counter_new>trajectory_points_counter)
    {
        // Spawn missing points
        spawn_trajectory_point(msg_poses_trajectory_points,model_trajectory_point_prefix_,trajectory_points_counter-1,trajectory_points_counter_new);

        for(int i=0;i<trajectory_points_counter_new;i++)
        {
            std::stringstream str_model_name;
            str_model_name << model_trajectory_point_prefix_ << std::setw(5) << std::setfill('0') << i;
            gazebo::physics::WorldPtr world = this->world_;
            gazebo::physics::ModelPtr mdl_traj_point = world->GetModel(str_model_name.str());
            gazebo::physics::LinkPtr link_trj_pt = mdl_traj_point->GetLink("base_link");
            math::Pose pose_trajectory_point_current;
            pose_trajectory_point_current.pos.x=msg_poses_trajectory_points.poses[i].position.x;
            pose_trajectory_point_current.pos.y=msg_poses_trajectory_points.poses[i].position.y;
            pose_trajectory_point_current.pos.z=msg_poses_trajectory_points.poses[i].position.z;
            pose_trajectory_point_current.rot.w=msg_poses_trajectory_points.poses[i].orientation.w;
            pose_trajectory_point_current.rot.x=msg_poses_trajectory_points.poses[i].orientation.x;
            pose_trajectory_point_current.rot.y=msg_poses_trajectory_points.poses[i].orientation.y;
            pose_trajectory_point_current.rot.z=msg_poses_trajectory_points.poses[i].orientation.z;
            link_trj_pt->SetWorldPose(pose_trajectory_point_current);
        }

        trajectory_points_counter=trajectory_points_counter_new;//msg_poses_trajectory_points.poses.size(); // Only update if larger set of points was received!
    }
    else if(trajectory_points_counter_new<trajectory_points_counter)
    {
        cleanup_trajectory_points(model_trajectory_point_prefix_,trajectory_points_counter_new-1,trajectory_points_counter);

        for(int i=0;i<trajectory_points_counter_new;i++)
        {
            std::stringstream str_model_name;
            str_model_name << model_trajectory_point_prefix_ << std::setw(5) << std::setfill('0') << i;
            gazebo::physics::WorldPtr world = this->world_;
            gazebo::physics::ModelPtr mdl_traj_point = world->GetModel(str_model_name.str());
            gazebo::physics::LinkPtr link_trj_pt = mdl_traj_point->GetLink("base_link");
            math::Pose pose_trajectory_point_current;
            pose_trajectory_point_current.pos.x=msg_poses_trajectory_points.poses[i].position.x;
            pose_trajectory_point_current.pos.y=msg_poses_trajectory_points.poses[i].position.y;
            pose_trajectory_point_current.pos.z=msg_poses_trajectory_points.poses[i].position.z;
            pose_trajectory_point_current.rot.w=msg_poses_trajectory_points.poses[i].orientation.w;
            pose_trajectory_point_current.rot.x=msg_poses_trajectory_points.poses[i].orientation.x;
            pose_trajectory_point_current.rot.y=msg_poses_trajectory_points.poses[i].orientation.y;
            pose_trajectory_point_current.rot.z=msg_poses_trajectory_points.poses[i].orientation.z;
            link_trj_pt->SetWorldPose(pose_trajectory_point_current);
        }

        trajectory_points_counter=trajectory_points_counter_new;//msg_poses_trajectory_points.poses.size(); // Only update if larger set of points was received!
    }

    //Remove old points
    /*for(int i=0;i<trajectory_points_counter;i++)
    {
        cleanup_trajectory_points(model_trajectory_point_prefix_,i);
    }*/

    // Update counter to current size of trajctory!
    /*if(msg_poses_trajectory_points.poses.size()>trajectory_points_counter)
    {
        trajectory_points_counter=msg_poses_trajectory_points.poses.size();
    }*/

    //ROS_ERROR("REceived n %d points.",trajectory_points_counter);
    //for(int i=0;i<trajectory_points_counter;i++)
    //{//}
    //spawn_trajectory_point(msg_poses_trajectory_points,model_trajectory_point_prefix_,trajectory_points_counter);
    ROS_INFO("Success!");
    std_msgs::Bool msg_spawn_points_success;
    msg_spawn_points_success.data=true;
    pub_spawn_points_success.publish(msg_spawn_points_success);

    lock_apply_force=false;
}
void GazeboRosForceGravity::cleanup_trajectory_points(std::string trajectory_point_model_prefix,
                            int trajectory_point_model_id_start,int trajectory_point_model_ids_max)
{
    for(int i=trajectory_point_model_id_start;i<trajectory_point_model_ids_max;i++)
    {
        /*
        std::stringstream str_model_name;
        str_model_name << trajectory_point_model_prefix << std::setw(5) << std::setfill('0') << i;
        gazebo_msgs::DeleteModel model;
        model.request.model_name = str_model_name.str().c_str();
        gazebo_delete_model_client.call(model);
        */
        std::stringstream str_model_name;
        str_model_name << model_trajectory_point_prefix_ << std::setw(5) << std::setfill('0') << i;
        gazebo::physics::WorldPtr world = this->world_;
        gazebo::physics::ModelPtr mdl_traj_point = world->GetModel(str_model_name.str());
        gazebo::physics::LinkPtr link_trj_pt = mdl_traj_point->GetLink("base_link");
        math::Pose pose_trajectory_point_current;
        pose_trajectory_point_current.pos.x=0.0;
        pose_trajectory_point_current.pos.y=0.0;
        pose_trajectory_point_current.pos.z=-10000.0;
        pose_trajectory_point_current.rot.w=1.0;
        pose_trajectory_point_current.rot.x=0.0;
        pose_trajectory_point_current.rot.y=0.0;
        pose_trajectory_point_current.rot.z=0.0;
        link_trj_pt->SetWorldPose(pose_trajectory_point_current);
    }
}
void GazeboRosForceGravity::spawn_trajectory_point(geometry_msgs::PoseArray pose_trajectory_points,
                            std::string trajectory_point_model_prefix,int trajectory_point_model_id_start,
                            int trajectory_point_model_ids_max)
{
    gazebo_msgs::SpawnModel model;
    std::ifstream ifs;
    ifs.open("/home/groundstation/catkin_ws/src/trajectory_reshaping/trajectory_reshaping_world_visualizer/resources/trajectory/trajectory_point.urdf");
    std::string xml((std::istreambuf_iterator<char>(ifs)), std::istreambuf_iterator<char>());
    model.request.model_xml = xml;

    std::stringstream str_model_name;
    if(trajectory_point_model_id_start<0)trajectory_point_model_id_start=0; // Prevent negative index during first spawn iteration
    for(int i=trajectory_point_model_id_start;i<trajectory_point_model_ids_max;i++)
    {
        str_model_name << trajectory_point_model_prefix << std::setw(5) << std::setfill('0') << i;
        model.request.model_name = str_model_name.str().c_str();
        model.request.initial_pose = pose_trajectory_points.poses[i];
        gazebo_spawn_model_client.call(model);
        str_model_name.str("");
    }

    // Make it more efficient?
    //gazebo::physics::WorldPtr world_ptr = this->world_;
    //world_ptr->InsertModelFile("/home/groundstation/catkin_ws/src/trajectory_reshaping/trajectory_reshaping_world_visualizer/resources/trajectory/trajectory_point.urdf");
    //gazebo::physics::ModelPtr model_ptr=world_ptr->ge
    //gazebo::physics::World::InsertModelFile("/home/groundstation/catkin_ws/src/trajectory_reshaping/trajectory_reshaping_world_visualizer/resources/trajectory/trajectory_point.urdf");
    //gazebo::physics::State
    //gazebo::physics::World::LoadModel()
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboRosForceGravity::UpdateObjectForce(const geometry_msgs::Wrench::ConstPtr& _msg)
{
  this->wrench_msg_.force.x = _msg->force.x;
  this->wrench_msg_.force.y = _msg->force.y;
  this->wrench_msg_.force.z = _msg->force.z;
  this->wrench_msg_.torque.x = _msg->torque.x;
  this->wrench_msg_.torque.y = _msg->torque.y;
  this->wrench_msg_.torque.z = _msg->torque.z;
}
/*void GazeboRosForceGravity::sub_pose_actor_drone_location_callback(const geometry_msgs::Pose::ConstPtr& msg)
{
//
    geometry_msgs::Pose msg_forward_pose=(*msg);
    geometry_msgs::PoseStamped msg_forward_posestmpd;
    msg_forward_posestmpd.header.frame_id="world";
    msg_forward_posestmpd.header.stamp=ros::Time::now();
    msg_forward_posestmpd.pose=msg_forward_pose;
    pub_posestmpd_actor_drone_location.publish(msg_forward_posestmpd);
}*/
void GazeboRosForceGravity::sub_posestmpd_bebop2_camera_current_view_distance_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    msg_posestmpd_bebop2_camera_current_view_distance=(*msg);
}
void GazeboRosForceGravity::sub_posestmpd_hand_left_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    msg_posestmpd_hand_left.header.stamp=ros::Time::now();
    msg_posestmpd_hand_left=(*msg);

    double hand_scale_translation_current=1.0;
    /*if(msg_joy_hand_left.axes.size()>0)
    {
        hand_scale_translation_current=(1.0+hand_scale_translational_max_*msg_joy_hand_left.axes[0]); // DONT SCALE LEFT HAND FOR NOW!
    }*/
    msg_posestmpd_hand_left.pose.position.x=msg_posestmpd_hand_left.pose.position.x*hand_scale_translation_current;
    msg_posestmpd_hand_left.pose.position.y=msg_posestmpd_hand_left.pose.position.y*hand_scale_translation_current;
    msg_posestmpd_hand_left.pose.position.z=msg_posestmpd_hand_left.pose.position.z*hand_scale_translation_current;

    /*try{
        //listener.lookupTransform("reshaping/bebop2camera/htc_vive_shoulder_left", "reshaping/bebop2camera/htc_vive_hmd_left_controller",
        //                         ros::Time::now(), tf_shoulder_hand_left);
        listener.lookupTransform("world", "reshaping/bebop2camera/htc_vive_hmd_left_controller",
                                 ros::Time(0), tf_world_hand_left);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(0.1).sleep();
    }*/

    tf::Transform tf_world_wrt_bebop2camera, tf_bebop2camera_wrt_shoulder_left,tf_shoulder_left_wrt_hand_left;
    tf_world_wrt_bebop2camera.setOrigin(tf::Vector3(msg_pose_bebop2_camera.position.x,msg_pose_bebop2_camera.position.y,msg_pose_bebop2_camera.position.z));
    tf::Quaternion quat_bebop2camera_yaw(0.0,0.0,msg_pose_bebop2_camera.orientation.z,msg_pose_bebop2_camera.orientation.w);
    tf_world_wrt_bebop2camera.setRotation(quat_bebop2camera_yaw.normalize());
    tf_bebop2camera_wrt_shoulder_left.setOrigin(tf::Vector3(0.15,0.15,-0.25));
    tf_bebop2camera_wrt_shoulder_left.setRotation(tf::Quaternion(0.0,0.0,0.0,1.0));
    tf_shoulder_left_wrt_hand_left.setOrigin(tf::Vector3(msg_posestmpd_hand_left.pose.position.x,msg_posestmpd_hand_left.pose.position.y,msg_posestmpd_hand_left.pose.position.z));
    tf_shoulder_left_wrt_hand_left.setRotation(tf::Quaternion(msg_posestmpd_hand_left.pose.orientation.x,msg_posestmpd_hand_left.pose.orientation.y,msg_posestmpd_hand_left.pose.orientation.z,msg_posestmpd_hand_left.pose.orientation.w));

    tf_world_hand_left=tf_world_wrt_bebop2camera*tf_bebop2camera_wrt_shoulder_left*tf_shoulder_left_wrt_hand_left;

    msg_posestmpd_hand_left.pose.position.x=tf_world_hand_left.getOrigin().getX();
    msg_posestmpd_hand_left.pose.position.y=tf_world_hand_left.getOrigin().getY();
    msg_posestmpd_hand_left.pose.position.z=tf_world_hand_left.getOrigin().getZ();
    msg_posestmpd_hand_left.pose.orientation.w=tf_world_hand_left.getRotation().getW();
    msg_posestmpd_hand_left.pose.orientation.x=tf_world_hand_left.getRotation().getX();
    msg_posestmpd_hand_left.pose.orientation.y=tf_world_hand_left.getRotation().getY();
    msg_posestmpd_hand_left.pose.orientation.z=tf_world_hand_left.getRotation().getZ();
}
void GazeboRosForceGravity::sub_posestmpd_hand_right_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    msg_posestmpd_hand_right.header.stamp=ros::Time::now();
    msg_posestmpd_hand_right=(*msg);

    if(interaction_mode_==2)return; // Disable scaling here for mode 3!

    double hand_scale_translation_current=1.0;

    if(msg_joy_hand_right.axes.size()>0)
    {
        hand_scale_translational_max_=msg_posestmpd_bebop2_camera_current_view_distance.pose.orientation.w; // Works fine, but interfers with path manipulation, set static now in adaptive view management!
        hand_scale_translation_current=(1.0+hand_scale_translational_max_*msg_joy_hand_right.axes[0]); // Never scale to zero!
    }
    msg_posestmpd_hand_right.pose.position.x=msg_posestmpd_hand_right.pose.position.x*hand_scale_translation_current;
    msg_posestmpd_hand_right.pose.position.y=msg_posestmpd_hand_right.pose.position.y*hand_scale_translation_current;
    msg_posestmpd_hand_right.pose.position.z=msg_posestmpd_hand_right.pose.position.z*hand_scale_translation_current;

    tf::StampedTransform tf_compenstate_hmd_rot;
    try{
      //listener.lookupTransform("reshaping/bebop2camera/htc_vive_shoulder_right", "reshaping/bebop2camera/htc_vive_hmd_right_controller",
      //                         ros::Time::now(), tf_shoulder_hand_right);
      listener.lookupTransform("world", "hmd", ros::Time(0), tf_compenstate_hmd_rot);
      tf_compenstate_hmd_rot.setOrigin(tf::Vector3(0.0,0.0,0.0));
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(0.1).sleep();
    }

    tf::Transform tf_world_wrt_bebop2camera, tf_bebop2camera_wrt_shoulder_right,tf_shoulder_right_wrt_hand_right;
    tf_world_wrt_bebop2camera.setOrigin(tf::Vector3(msg_pose_bebop2_camera.position.x,msg_pose_bebop2_camera.position.y,msg_pose_bebop2_camera.position.z));
    tf::Quaternion quat_bebop2camera_yaw(0.0,0.0,msg_pose_bebop2_camera.orientation.z,msg_pose_bebop2_camera.orientation.w);
    tf_world_wrt_bebop2camera.setRotation(quat_bebop2camera_yaw.normalize());
    tf_bebop2camera_wrt_shoulder_right.setOrigin(tf::Vector3(0.15,-0.15,-0.25));
    tf_bebop2camera_wrt_shoulder_right.setRotation(tf::Quaternion(0.0,0.0,0.0,1.0));
    tf_shoulder_right_wrt_hand_right.setOrigin(tf::Vector3(msg_posestmpd_hand_right.pose.position.x,msg_posestmpd_hand_right.pose.position.y,msg_posestmpd_hand_right.pose.position.z));
    tf_shoulder_right_wrt_hand_right.setRotation(tf::Quaternion(msg_posestmpd_hand_right.pose.orientation.x,msg_posestmpd_hand_right.pose.orientation.y,msg_posestmpd_hand_right.pose.orientation.z,msg_posestmpd_hand_right.pose.orientation.w));

    tf_world_hand_right=tf_world_wrt_bebop2camera*tf_bebop2camera_wrt_shoulder_right*tf_shoulder_right_wrt_hand_right;

    msg_posestmpd_hand_right.pose.position.x=tf_world_hand_right.getOrigin().getX();
    msg_posestmpd_hand_right.pose.position.y=tf_world_hand_right.getOrigin().getY();
    msg_posestmpd_hand_right.pose.position.z=tf_world_hand_right.getOrigin().getZ();
    msg_posestmpd_hand_right.pose.orientation.w=tf_world_hand_right.getRotation().getW();
    msg_posestmpd_hand_right.pose.orientation.x=tf_world_hand_right.getRotation().getX();
    msg_posestmpd_hand_right.pose.orientation.y=tf_world_hand_right.getRotation().getY();
    msg_posestmpd_hand_right.pose.orientation.z=tf_world_hand_right.getRotation().getZ();
}
void GazeboRosForceGravity::sub_joy_hand_left_callback(const sensor_msgs::Joy::ConstPtr& msg)
{
    msg_joy_hand_left.header.stamp=ros::Time::now();
    msg_joy_hand_left=(*msg);
}
void GazeboRosForceGravity::sub_joy_hand_right_callback(const sensor_msgs::Joy::ConstPtr& msg)
{
    msg_joy_hand_right.header.stamp=ros::Time::now();
    msg_joy_hand_right=(*msg);
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboRosForceGravity::UpdateChild()
{
    this->lock_.lock();

    double force_velocity_scale_translation_current=0.0;


    // UPDATE LEFT HAND POSE
    gazebo::physics::WorldPtr world_ptr_hand_left = this->world_;
    gazebo::physics::ModelPtr mdl_hand_left = world_ptr_hand_left->GetModel("realistichandleft");
    gazebo::physics::LinkPtr link_hand_left = mdl_hand_left->GetLink("base_link");
    math::Pose pose_hand_left_current;
    pose_hand_left_current.pos.x=msg_posestmpd_hand_left.pose.position.x;
    pose_hand_left_current.pos.y=msg_posestmpd_hand_left.pose.position.y;
    pose_hand_left_current.pos.z=msg_posestmpd_hand_left.pose.position.z;
    pose_hand_left_current.rot.w=msg_posestmpd_hand_left.pose.orientation.w;
    pose_hand_left_current.rot.x=msg_posestmpd_hand_left.pose.orientation.x;
    pose_hand_left_current.rot.y=msg_posestmpd_hand_left.pose.orientation.y;
    pose_hand_left_current.rot.z=msg_posestmpd_hand_left.pose.orientation.z;
    link_hand_left->SetWorldPose(pose_hand_left_current);


    // UPDATE RIGHT HAND POSE
    math::Pose pose_hand_right_current;
    if(interaction_mode_!=2)
    {
        gazebo::physics::WorldPtr world_ptr_hand_right = this->world_;
        gazebo::physics::ModelPtr mdl_hand_right = world_ptr_hand_right->GetModel("realistichandright");
        gazebo::physics::LinkPtr link_hand_right = mdl_hand_right->GetLink("base_link");
        pose_hand_right_current.pos.x=msg_posestmpd_hand_right.pose.position.x;
        pose_hand_right_current.pos.y=msg_posestmpd_hand_right.pose.position.y;
        pose_hand_right_current.pos.z=msg_posestmpd_hand_right.pose.position.z;
        pose_hand_right_current.rot.w=msg_posestmpd_hand_right.pose.orientation.w;
        pose_hand_right_current.rot.x=msg_posestmpd_hand_right.pose.orientation.x;
        pose_hand_right_current.rot.y=msg_posestmpd_hand_right.pose.orientation.y;
        pose_hand_right_current.rot.z=msg_posestmpd_hand_right.pose.orientation.z;
        link_hand_right->SetWorldPose(pose_hand_right_current);
    }
    /*else
    {
        // If in mode 3 then set default setpoint for tree experiment
        pose_hand_right_current.pos.x=65.0;
        pose_hand_right_current.pos.y=-15.0;
        pose_hand_right_current.pos.z=2.5;
        pose_hand_right_current.rot.w=1.0;
        pose_hand_right_current.rot.x=0.0;
        pose_hand_right_current.rot.y=0.0;
        pose_hand_right_current.rot.z=0.0;
    }*/


    // TRIGGER SETPOINT FOR EXPLORATION
    //int mockup_mode_=1;
    if(msg_joy_hand_right.axes.size()>0 && msg_joy_hand_right.buttons[4]==1 && (ros::Time::now()-time_last_button_press).toSec()>=3.0) // Check for right hand gripper button
    {
        msg_clicked_point_valid.header.stamp=ros::Time::now();
        if(mockup_mode_==0)
        {
            msg_clicked_point_valid.header.frame_id="raw"; // Generate Raw trajectory based on virtual hands setpoint
            msg_clicked_point_valid.point.x=45.0;//pose_hand_right_current.pos.x;
            msg_clicked_point_valid.point.y=-14.0;//pose_hand_right_current.pos.y;
            msg_clicked_point_valid.point.z=2.5;//pose_hand_right_current.pos.z;
        }
        if(mockup_mode_==1)
        {
            msg_clicked_point_valid.header.frame_id="raw_mockup_synthetic"; // Generate trajectory for synthetic mockup scene
            msg_clicked_point_valid.point.x=1.0; // not relevant actually because overwritten in PRM callback!
            msg_clicked_point_valid.point.y=0.0;
            msg_clicked_point_valid.point.z=1.0;
        }
        if(mockup_mode_==2)
        {
            msg_clicked_point_valid.header.frame_id="raw_mockup_lab"; // Generate trajectory for real-world test in lab
            msg_clicked_point_valid.point.x=2.0; // not relevant actually because overwritten in PRM callback!
            msg_clicked_point_valid.point.y=0.0;
            msg_clicked_point_valid.point.z=1.0;
            //system("rostopic pub /joy sensor_msgs/Joy '{ header: {seq: 0, stamp: {secs: 0, nsecs: 0}, frame_id: 'world'}, axes: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], buttons: [0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0]}'");
        }

        pub_clicked_point_valid.publish(msg_clicked_point_valid);
        time_last_button_press=ros::Time::now();
        ROS_WARN("Published valid setpoint for global planner at xyz %3.3f %3.3f %3.3f!",msg_clicked_point_valid.point.x,msg_clicked_point_valid.point.y,msg_clicked_point_valid.point.z);
    }

    // TRIGGER VIEW CHANGE
    if(msg_joy_hand_right.axes.size()>0 && msg_joy_hand_right.buttons[1]==1 && (ros::Time::now()-time_last_button_press_desired_view).toSec()>=1.0) // Check for right hand gripper button, 0.5 before
    {
        if(msg_joy_hand_right.axes[1]>0.5 && fabs(msg_joy_hand_right.axes[2])<0.5)
        {
            // Increment YAW
            yaw_desired_view+=7.0*M_PI/180.0;
        }
        else if(msg_joy_hand_right.axes[1]<-0.5 && fabs(msg_joy_hand_right.axes[2])<0.5)
        {
            // Decrement YAW
            yaw_desired_view-=7.0*M_PI/180.0;
        }
        else if(fabs(msg_joy_hand_right.axes[1])<0.5 && msg_joy_hand_right.axes[2]>0.5)
        {
            // Increment PITCH
            pitch_desired_view+=7.0*M_PI/180.0;
        }
        else if(fabs(msg_joy_hand_right.axes[1])<0.5 && msg_joy_hand_right.axes[2]<-0.5)
        {
            // Decrement PITCH
            pitch_desired_view-=7.0*M_PI/180.0;
        }

        if(pitch_desired_view<0.25)pitch_desired_view=0.25; // Should be same as in adaptive view node!
        if(pitch_desired_view>(1.5-0.25))pitch_desired_view=(1.5-0.25);

        tf::Quaternion quat_desired_view;
        quat_desired_view.setRPY(0.0,pitch_desired_view,yaw_desired_view);
        msg_pose_desired_view.position.x=0.0;
        msg_pose_desired_view.position.y=0.0;
        msg_pose_desired_view.position.z=0.0;
        msg_pose_desired_view.orientation.w=quat_desired_view.getW();
        msg_pose_desired_view.orientation.x=quat_desired_view.getX();
        msg_pose_desired_view.orientation.y=quat_desired_view.getY();
        msg_pose_desired_view.orientation.z=quat_desired_view.getZ();

        time_last_button_press_desired_view=ros::Time::now();
        ROS_WARN("Desired view changed to angle RPY (%3.3f,%3.3f,%3.3f)",0.0,pitch_desired_view,yaw_desired_view);
    }

    pub_pose_bebop2_camera_desired_view.publish(msg_pose_desired_view);


    // APPLY FORCES
    if(!lock_apply_force)
    {
        if(interaction_mode_==2)
        {
            int trajectory_point_index_closest=0;

            for(int i=0;i<trajectory_points_counter;i++)
            {
                std::stringstream str_model_name;
                str_model_name << model_trajectory_point_prefix_ << std::setw(5) << std::setfill('0') << i;
                gazebo::physics::WorldPtr world = this->world_;
                gazebo::physics::ModelPtr mdl_traj_point = world->GetModel(str_model_name.str());
                gazebo::physics::LinkPtr link_trj_pt = mdl_traj_point->GetLink("base_link");

                // Calculate vector from bebop2actor to current traj_point
                geometry_msgs::Pose pose_temp;
                pose_temp.position.x=link_trj_pt->GetWorldPose().pos.x;
                pose_temp.position.y=link_trj_pt->GetWorldPose().pos.y;
                pose_temp.position.z=link_trj_pt->GetWorldPose().pos.z;
                pose_temp.orientation.w=1.0;
                pose_temp.orientation.x=0.0;
                pose_temp.orientation.y=0.0;
                pose_temp.orientation.z=0.0;

                tf::Vector3 vec_bebop2actor_to_traj_point;
                vec_bebop2actor_to_traj_point.setX(pose_temp.position.x-msg_pose_bebop2_actor.position.x);
                vec_bebop2actor_to_traj_point.setY(pose_temp.position.y-msg_pose_bebop2_actor.position.y);
                vec_bebop2actor_to_traj_point.setY(pose_temp.position.z-msg_pose_bebop2_actor.position.z);

                tf::Vector3 vec_bebop2actor_flight_direction(1.0,0.0,0.0);
                tf::Quaternion quat_bebop2actor(msg_pose_bebop2_actor.orientation.x,msg_pose_bebop2_actor.orientation.y,msg_pose_bebop2_actor.orientation.z,msg_pose_bebop2_actor.orientation.w);
                vec_bebop2actor_flight_direction = tf::quatRotate(quat_bebop2actor, vec_bebop2actor_flight_direction);
                if(fabs(vec_bebop2actor_to_traj_point.angle(vec_bebop2actor_flight_direction))<=1.570796327)
                {
                    trajectory_point_index_closest=i;
                    break;
                }
            }
            //ROS_INFO("traj point closest index %d",trajectory_point_index_closest);

            int trajectory_point_index_manipulate=0;
            if(msg_joy_hand_right.axes.size()>0)
            {
                trajectory_point_index_manipulate=trajectory_point_index_closest+
                        int(double((trajectory_points_counter-1-trajectory_point_index_closest))*msg_joy_hand_right.axes[0]);
                //ROS_INFO("slected traj point index is %d",trajectory_point_index_manipulate);

                // Select trajectory point which is closest according to scale
                std::stringstream str_model_name;
                str_model_name << model_trajectory_point_prefix_ << std::setw(5) << std::setfill('0') << trajectory_point_index_manipulate;
                gazebo::physics::WorldPtr world = this->world_;
                gazebo::physics::ModelPtr mdl_traj_point = world->GetModel(str_model_name.str());
                gazebo::physics::LinkPtr link_trj_pt = mdl_traj_point->GetLink("base_link");
                geometry_msgs::Pose pose_trajectory_point_selected;
                pose_trajectory_point_selected.position.x=link_trj_pt->GetWorldPose().pos.x;
                pose_trajectory_point_selected.position.y=link_trj_pt->GetWorldPose().pos.y;
                pose_trajectory_point_selected.position.z=link_trj_pt->GetWorldPose().pos.z;
                pose_trajectory_point_selected.orientation.w=msg_posestmpd_hand_right.pose.orientation.w; // Orient according to right hand pose!
                pose_trajectory_point_selected.orientation.x=msg_posestmpd_hand_right.pose.orientation.x;
                pose_trajectory_point_selected.orientation.y=msg_posestmpd_hand_right.pose.orientation.y;
                pose_trajectory_point_selected.orientation.z=msg_posestmpd_hand_right.pose.orientation.z;

                tf::Transform tf_world_to_point_selected;
                tf_world_to_point_selected.setOrigin(tf::Vector3(link_trj_pt->GetWorldPose().pos.x,link_trj_pt->GetWorldPose().pos.y,link_trj_pt->GetWorldPose().pos.z));
                tf_world_to_point_selected.setRotation(tf::Quaternion(msg_posestmpd_hand_right.pose.orientation.x,msg_posestmpd_hand_right.pose.orientation.y,msg_posestmpd_hand_right.pose.orientation.z,msg_posestmpd_hand_right.pose.orientation.w));
                tf::Transform tf_point_selected_to_force;
                tf_point_selected_to_force.setOrigin(tf::Vector3(0.0,0.0,0.5)); // Fixed distance for now
                tf_point_selected_to_force.setRotation(tf::Quaternion(0.0,0.0,0.0,1.0));
                tf::Transform tf_world_to_force;
                tf_world_to_force=tf_world_to_point_selected*tf_point_selected_to_force;

                // Apply force by sliding hand along the path
                gazebo::physics::WorldPtr world_ptr_hand_right = this->world_;
                gazebo::physics::ModelPtr mdl_hand_right = world_ptr_hand_right->GetModel("realistichandright");
                gazebo::physics::LinkPtr link_hand_right = mdl_hand_right->GetLink("base_link");

                pose_hand_right_current_along_path.pos.x=tf_world_to_force.getOrigin().getX();
                pose_hand_right_current_along_path.pos.y=tf_world_to_force.getOrigin().getY();
                pose_hand_right_current_along_path.pos.z=tf_world_to_force.getOrigin().getZ();
                pose_hand_right_current_along_path.rot.w=tf_world_to_force.getRotation().getW();
                pose_hand_right_current_along_path.rot.x=tf_world_to_force.getRotation().getX();
                pose_hand_right_current_along_path.rot.y=tf_world_to_force.getRotation().getY();
                pose_hand_right_current_along_path.rot.z=tf_world_to_force.getRotation().getZ();
                link_hand_right->SetWorldPose(pose_hand_right_current_along_path);
            }

                /*
                // Reset all forces or velocity inputs after confirmation was triggered!
                math::Vector3 vector_zero(0.0,0.0,0.0);
                link_trj_pt->AddForce(vector_zero);
                link_trj_pt->AddTorque(vector_zero);
                link_trj_pt->SetLinearVel(vector_zero);
                link_trj_pt->SetAngularVel(vector_zero);*/

            //this->lock_.unlock(); // Dont forget to unlock before leaving update!
            //return; // In case of mode 3 dont trigger methods 1/2!
        } // FOR NOW JUST VISUALIZING RIGHT HAND



        // HERE START MODE 1 and 2
        geometry_msgs::PoseArray msg_poses_trajectory_points_planner_forces;

        // In case of mirroring calculate trajectory point which is closest to hand!
        int trajectory_point_index_closest_to_hand=0;
        if(mirror_around_closest_point_==1)// && msg_planner_path_points_interpolated_original.poses.size()>0)
        {
            double distance_min=100.0;
            /*for(int i=0;i<trajectory_points_counter;i++)
            {
                std::stringstream str_model_name;
                str_model_name << model_trajectory_point_prefix_ << std::setw(5) << std::setfill('0') << i;
                gazebo::physics::WorldPtr world = this->world_;
                gazebo::physics::ModelPtr mdl_traj_point = world->GetModel(str_model_name.str());
                gazebo::physics::LinkPtr link_trj_pt = mdl_traj_point->GetLink("base_link");

                // Calculate vector from bebop2actor to current traj_point
                geometry_msgs::Pose pose_temp;
                pose_temp.position.x=link_trj_pt->GetWorldPose().pos.x;
                pose_temp.position.y=link_trj_pt->GetWorldPose().pos.y;
                pose_temp.position.z=link_trj_pt->GetWorldPose().pos.z;
                pose_temp.orientation.w=1.0;
                pose_temp.orientation.x=0.0;
                pose_temp.orientation.y=0.0;
                pose_temp.orientation.z=0.0;

                tf::Vector3 vec_hand_right_to_traj_point;
                vec_hand_right_to_traj_point.setX(pose_temp.position.x-pose_hand_right_current.pos.x);
                vec_hand_right_to_traj_point.setY(pose_temp.position.y-pose_hand_right_current.pos.y);
                vec_hand_right_to_traj_point.setY(pose_temp.position.z-pose_hand_right_current.pos.z);

                if(fabs(vec_hand_right_to_traj_point.length())<distance_min)
                {
                    distance_min=fabs(vec_hand_right_to_traj_point.length());
                    trajectory_point_index_closest_to_hand=i;
                }
            }*/
            for(int i=0;i<msg_planner_path_points_interpolated_original.poses.size();i++)
            {
                /*std::stringstream str_model_name;
                str_model_name << model_trajectory_point_prefix_ << std::setw(5) << std::setfill('0') << i;
                gazebo::physics::WorldPtr world = this->world_;
                gazebo::physics::ModelPtr mdl_traj_point = world->GetModel(str_model_name.str());
                gazebo::physics::LinkPtr link_trj_pt = mdl_traj_point->GetLink("base_link");*/

                // Calculate vector from bebop2actor to current traj_point
                geometry_msgs::Pose pose_temp;
                pose_temp.position.x=msg_planner_path_points_interpolated_original.poses[i].position.x;
                pose_temp.position.y=msg_planner_path_points_interpolated_original.poses[i].position.y;
                pose_temp.position.z=msg_planner_path_points_interpolated_original.poses[i].position.z;
                pose_temp.orientation.w=1.0;
                pose_temp.orientation.x=0.0;
                pose_temp.orientation.y=0.0;
                pose_temp.orientation.z=0.0;

                tf::Vector3 vec_hand_right_to_traj_point;
                vec_hand_right_to_traj_point.setX(pose_temp.position.x-pose_hand_right_current.pos.x);
                vec_hand_right_to_traj_point.setY(pose_temp.position.y-pose_hand_right_current.pos.y);
                vec_hand_right_to_traj_point.setY(pose_temp.position.z-pose_hand_right_current.pos.z);

                if(fabs(vec_hand_right_to_traj_point.length())<distance_min)
                {
                    distance_min=fabs(vec_hand_right_to_traj_point.length());
                    trajectory_point_index_closest_to_hand=i;
                }
            }

            // Mirror hand pose vector here for generating force!
            /*std::stringstream str_model_name;
            str_model_name << model_trajectory_point_prefix_ << std::setw(5) << std::setfill('0') << trajectory_point_index_closest_to_hand;
            gazebo::physics::WorldPtr world = this->world_;
            gazebo::physics::ModelPtr mdl_traj_point = world->GetModel(str_model_name.str());
            gazebo::physics::LinkPtr link_trj_pt = mdl_traj_point->GetLink("base_link");
            // Calculate distance from hand position to trajectory point
            math::Vector3 trajectory_point_vector(link_trj_pt->GetWorldPose().pos.x,link_trj_pt->GetWorldPose().pos.y,link_trj_pt->GetWorldPose().pos.z);
            math::Vector3 hand_right_vector(pose_hand_right_current.pos.x,pose_hand_right_current.pos.y,pose_hand_right_current.pos.z);
            math::Vector3 vec_hand_right_to_trajectory_point=trajectory_point_vector-hand_right_vector;
            hand_right_vector=hand_right_vector+2*vec_hand_right_to_trajectory_point;
            pose_hand_right_current.pos.x=hand_right_vector.x;
            pose_hand_right_current.pos.y=hand_right_vector.y;
            pose_hand_right_current.pos.z=hand_right_vector.z;*/

            // Calculate distance from hand position to closest INTERPOLATED trajectory point from original trajectory
            math::Vector3 trajectory_point_vector(msg_planner_path_points_interpolated_original.poses[trajectory_point_index_closest_to_hand].position.x,
                                                  msg_planner_path_points_interpolated_original.poses[trajectory_point_index_closest_to_hand].position.y,
                                                  msg_planner_path_points_interpolated_original.poses[trajectory_point_index_closest_to_hand].position.z);
            math::Vector3 hand_right_vector(pose_hand_right_current.pos.x,pose_hand_right_current.pos.y,pose_hand_right_current.pos.z);
            math::Vector3 vec_hand_right_to_trajectory_point=trajectory_point_vector-hand_right_vector;
            hand_right_vector=hand_right_vector+2*vec_hand_right_to_trajectory_point;
            pose_hand_right_current.pos.x=hand_right_vector.x;
            pose_hand_right_current.pos.y=hand_right_vector.y;
            pose_hand_right_current.pos.z=hand_right_vector.z;

            msg_pose_trajectory_point_closest_to_hand.position.x=trajectory_point_vector.x;
            msg_pose_trajectory_point_closest_to_hand.position.y=trajectory_point_vector.y;
            msg_pose_trajectory_point_closest_to_hand.position.z=trajectory_point_vector.z;
            pub_pose_trajectory_point_closest_to_hand.publish(msg_pose_trajectory_point_closest_to_hand);
        }

        for(int i=0;i<trajectory_points_counter;i++)
        {
            // Scale force vector of right hand, do that for BOTH hand interaction cases (right handed or both handed)
            if(msg_joy_hand_left.axes.size()>0)
            {
                // Scale FORCE VECTOR originated at right hand with linear axis from left hand
                force_velocity_scale_translation_current=force_velocity_scale_translational_max_*msg_joy_hand_left.axes[0];
            }
            else
            {
                force_velocity_scale_translation_current=0.0;
            }

          if(use_hands_option_==1) // Use right hand only
          {   
              /* Scale force vector of right hand
              if(msg_joy_hand_left.axes.size()>0)
              {
                  // Scale FORCE VECTOR originated at right hand with linear axis from left hand
                  force_velocity_scale_translation_current=force_velocity_scale_translational_max_*msg_joy_hand_left.axes[0];
              }
              else
              {
                  force_velocity_scale_translation_current=0.0;
              }*/

              std::stringstream str_model_name;
              str_model_name << model_trajectory_point_prefix_ << std::setw(5) << std::setfill('0') << i;
              gazebo::physics::WorldPtr world = this->world_;
              gazebo::physics::ModelPtr mdl_traj_point = world->GetModel(str_model_name.str());
              gazebo::physics::LinkPtr link_trj_pt = mdl_traj_point->GetLink("base_link");

              // Calculate distance from hand position to trajectory point
              math::Vector3 trajectory_point_vector(link_trj_pt->GetWorldPose().pos.x,link_trj_pt->GetWorldPose().pos.y,link_trj_pt->GetWorldPose().pos.z);

              math::Vector3 hand_pose_vector;
              if(interaction_mode_==2)
              {
                  hand_pose_vector.x=pose_hand_right_current_along_path.pos.x;
                  hand_pose_vector.y=pose_hand_right_current_along_path.pos.y;
                  hand_pose_vector.z=pose_hand_right_current_along_path.pos.z;
              }
              else
              {
                  hand_pose_vector.x=pose_hand_right_current.pos.x;
                  hand_pose_vector.y=pose_hand_right_current.pos.y;
                  hand_pose_vector.z=pose_hand_right_current.pos.z;
              }

              math::Vector3 hand_to_trajectory_vector=trajectory_point_vector-hand_pose_vector;
              //math::Vector3 hand_to_trajectory_vector_to_visualizer=hand_to_trajectory_vector;
              float dist_hand_point=hand_to_trajectory_vector.GetLength();
              math::Vector3 hand_to_trajectory_vector_norm=hand_to_trajectory_vector.Normalize();
              math::Vector3 hand_to_trajectory_point_force;

              if(dist_hand_point<=force_max_distance_threshold_ && dist_hand_point>=force_min_distance_threshold_)
              {
                  //hand_to_trajectory_point_force=hand_to_trajectory_vector_norm*par_gravitational_constant_default*(par_mass_hand_default*par_mass_trajectory_point_default)/pow(dist_hand_point,2.0);
                  hand_to_trajectory_point_force=hand_to_trajectory_vector_norm*force_velocity_scale_translation_current*(1.0*1.0)/pow(dist_hand_point,2.0);
                  if(pushpull_mode_==2)
                  {
                      hand_to_trajectory_point_force=-1.0*hand_to_trajectory_point_force;
                  }
                  math::Vector3 force(hand_to_trajectory_point_force.x,hand_to_trajectory_point_force.y,hand_to_trajectory_point_force.z);
                  math::Vector3 torque(0.0,0.0,0.0);
                  math::Vector3 angvel(0.0,0.0,0.0);
                  if(force_mode_==1)
                  {
                    link_trj_pt->AddForce(force);
                    link_trj_pt->AddTorque(torque);
                  }
                  if(force_mode_==2)
                  {
                    link_trj_pt->SetLinearVel(force);
                    link_trj_pt->SetAngularVel(angvel);
                  }
              }
              else
              {
                  math::Vector3 vector_zero(0.0,0.0,0.0);
                  link_trj_pt->AddForce(vector_zero);
                  link_trj_pt->AddTorque(vector_zero);
                  link_trj_pt->SetLinearVel(vector_zero);
                  link_trj_pt->SetAngularVel(vector_zero);
              }

              // Push back forces to Pose Array and publish to RViz World Visualizer
              geometry_msgs::Pose msg_pose_trajectory_points_planner_force;
              msg_pose_trajectory_points_planner_force.position.x=hand_to_trajectory_point_force.x;
              msg_pose_trajectory_points_planner_force.position.y=hand_to_trajectory_point_force.y;
              msg_pose_trajectory_points_planner_force.position.z=hand_to_trajectory_point_force.z;
              msg_pose_trajectory_points_planner_force.orientation.w=dist_hand_point;
              msg_pose_trajectory_points_planner_force.orientation.x=hand_pose_vector.x;
              msg_pose_trajectory_points_planner_force.orientation.y=hand_pose_vector.y;
              msg_pose_trajectory_points_planner_force.orientation.z=hand_pose_vector.z;
              msg_poses_trajectory_points_planner_forces.poses.push_back(msg_pose_trajectory_points_planner_force);
          }
          else if(use_hands_option_==2) // Use both hands
          {
              std::stringstream str_model_name;
              str_model_name << model_trajectory_point_prefix_ << std::setw(5) << std::setfill('0') << i;
              gazebo::physics::WorldPtr world = this->world_;
              gazebo::physics::ModelPtr mdl_traj_point = world->GetModel(str_model_name.str());
              gazebo::physics::LinkPtr link_trj_pt = mdl_traj_point->GetLink("base_link");

              // Calculate distance from hand position to trajectory point
              math::Vector3 trajectory_point_vector(link_trj_pt->GetWorldPose().pos.x,link_trj_pt->GetWorldPose().pos.y,link_trj_pt->GetWorldPose().pos.z);
              math::Vector3 hand_left_pose_vector(pose_hand_left_current.pos.x,pose_hand_left_current.pos.y,pose_hand_left_current.pos.z);
              math::Vector3 hand_right_pose_vector(pose_hand_right_current.pos.x,pose_hand_right_current.pos.y,pose_hand_right_current.pos.z);
              math::Vector3 hand_left_to_trajectory_vector=trajectory_point_vector-hand_left_pose_vector;
              math::Vector3 hand_right_to_trajectory_vector=trajectory_point_vector-hand_right_pose_vector;
              float dist_hand_left_point=hand_left_to_trajectory_vector.GetLength();
              float dist_hand_right_point=hand_right_to_trajectory_vector.GetLength();
              math::Vector3 hand_left_to_trajectory_vector_norm=hand_left_to_trajectory_vector.Normalize();
              math::Vector3 hand_right_to_trajectory_vector_norm=hand_right_to_trajectory_vector.Normalize();
              math::Vector3 hand_left_to_trajectory_point_force;
              math::Vector3 hand_right_to_trajectory_point_force;

              if(dist_hand_left_point<=force_max_distance_threshold_ && dist_hand_right_point<=force_max_distance_threshold_ &&
                      dist_hand_left_point>=force_min_distance_threshold_ && dist_hand_right_point>=force_min_distance_threshold_)
              {
                  //hand_to_trajectory_point_force=hand_to_trajectory_vector_norm*par_gravitational_constant_default*(par_mass_hand_default*par_mass_trajectory_point_default)/pow(dist_hand_point,2.0);
                  hand_left_to_trajectory_point_force=hand_left_to_trajectory_vector_norm*force_velocity_scale_translation_current*(1.0*1.0)/pow(dist_hand_left_point,2.0);
                  hand_right_to_trajectory_point_force=hand_right_to_trajectory_vector_norm*force_velocity_scale_translation_current*(1.0*1.0)/pow(dist_hand_right_point,2.0);
                  if(pushpull_mode_==2)
                  {
                      hand_left_to_trajectory_point_force=-1.0*hand_left_to_trajectory_point_force;
                      hand_right_to_trajectory_point_force=-1.0*hand_right_to_trajectory_point_force;
                  }
                  math::Vector3 force(hand_left_to_trajectory_point_force.x+hand_right_to_trajectory_point_force.x,
                                      hand_left_to_trajectory_point_force.y+hand_right_to_trajectory_point_force.y,
                                      hand_left_to_trajectory_point_force.z+hand_right_to_trajectory_point_force.z);
                  //math::Vector3 torque(0.0,0.0,0.0);
                  //link_trj_pt->AddForce(force);
                  //link_trj_pt->AddTorque(torque);
                  math::Vector3 torque(0.0,0.0,0.0);
                  math::Vector3 angvel(0.0,0.0,0.0);
                  if(force_mode_==1)
                  {
                    link_trj_pt->AddForce(force);
                    link_trj_pt->AddTorque(torque);
                  }
                  if(force_mode_==2)
                  {
                    link_trj_pt->SetLinearVel(force);
                    link_trj_pt->SetAngularVel(angvel);
                  }
              }
              else
              {
                  math::Vector3 vector_zero(0.0,0.0,0.0);
                  link_trj_pt->AddForce(vector_zero);
                  link_trj_pt->AddTorque(vector_zero);
                  link_trj_pt->SetLinearVel(vector_zero);
                  link_trj_pt->SetAngularVel(vector_zero);
              }
          }
        }
        pub_trajectory_points_planner_forces.publish(msg_poses_trajectory_points_planner_forces);
        msg_poses_trajectory_points_planner_forces.poses.clear();

        // TRIGGER UPDATING NEW TRAJECTORY AFTER RESHAPING
        if(!lock_apply_force && msg_joy_hand_left.axes[0]==0.0 && msg_joy_hand_left.axes.size()>0 && msg_joy_hand_left.buttons[4]==1 && (ros::Time::now()-time_last_button_press_left).toSec()>=3.0) // Check for right hand gripper button
        {
            // First clear all visualized forces
            for(int i=0;i<trajectory_points_counter;i++)
            {
                geometry_msgs::Pose msg_pose_trajectory_points_planner_force;
                msg_pose_trajectory_points_planner_force.position.x=0.0;
                msg_pose_trajectory_points_planner_force.position.y=0.0;
                msg_pose_trajectory_points_planner_force.position.z=0.0;
                msg_pose_trajectory_points_planner_force.orientation.w=0.0; // Currently not used on the Visualizer side!
                msg_pose_trajectory_points_planner_force.orientation.x=0.0;
                msg_pose_trajectory_points_planner_force.orientation.y=0.0;
                msg_pose_trajectory_points_planner_force.orientation.z=0.0;
                msg_poses_trajectory_points_planner_forces.poses.push_back(msg_pose_trajectory_points_planner_force);
            }
            pub_trajectory_points_planner_forces.publish(msg_poses_trajectory_points_planner_forces);
            msg_poses_trajectory_points_planner_forces.poses.clear();


            geometry_msgs::PoseArray msg_poses_trajectory_reshaped;

            // Push back current actor uav pose first
            geometry_msgs::Pose pose_uav_current_tmp=msg_pose_bebop2_actor;
            msg_poses_trajectory_reshaped.poses.push_back(pose_uav_current_tmp);
            bool first_point_in_front=true;

            for(int i=0;i<trajectory_points_counter;i++)
            {
                std::stringstream str_model_name;
                str_model_name << model_trajectory_point_prefix_ << std::setw(5) << std::setfill('0') << i;
                gazebo::physics::WorldPtr world = this->world_;
                gazebo::physics::ModelPtr mdl_traj_point = world->GetModel(str_model_name.str());
                gazebo::physics::LinkPtr link_trj_pt = mdl_traj_point->GetLink("base_link");

                // Calculate vector from bebop2actor to current traj_point
                geometry_msgs::Pose pose_temp;
                pose_temp.position.x=link_trj_pt->GetWorldPose().pos.x;
                pose_temp.position.y=link_trj_pt->GetWorldPose().pos.y;
                pose_temp.position.z=link_trj_pt->GetWorldPose().pos.z;
                pose_temp.orientation.w=1.0;
                pose_temp.orientation.x=0.0;
                pose_temp.orientation.y=0.0;
                pose_temp.orientation.z=0.0;

                tf::Vector3 vec_bebop2actor_to_traj_point;
                vec_bebop2actor_to_traj_point.setX(pose_temp.position.x-msg_pose_bebop2_actor.position.x);
                vec_bebop2actor_to_traj_point.setY(pose_temp.position.y-msg_pose_bebop2_actor.position.y);
                vec_bebop2actor_to_traj_point.setY(pose_temp.position.z-msg_pose_bebop2_actor.position.z);

                tf::Vector3 vec_bebop2actor_flight_direction(1.0,0.0,0.0);
                tf::Quaternion quat_bebop2actor(msg_pose_bebop2_actor.orientation.x,msg_pose_bebop2_actor.orientation.y,msg_pose_bebop2_actor.orientation.z,msg_pose_bebop2_actor.orientation.w);
                vec_bebop2actor_flight_direction = tf::quatRotate(quat_bebop2actor, vec_bebop2actor_flight_direction);
                if(fabs(vec_bebop2actor_to_traj_point.angle(vec_bebop2actor_flight_direction))<=1.570796327)
                {
                    if(first_point_in_front==true)
                    {
                        // skipping first point in front to prevent wrong trajectory curvature!
                        first_point_in_front=false;
                    }
                    else
                    {
                        msg_poses_trajectory_reshaped.poses.push_back(pose_temp);
                    }
                }

                // Reset all forces or velocity inputs after confirmation was triggered!
                math::Vector3 vector_zero(0.0,0.0,0.0);
                link_trj_pt->AddForce(vector_zero);
                link_trj_pt->AddTorque(vector_zero);
                link_trj_pt->SetLinearVel(vector_zero);
                link_trj_pt->SetAngularVel(vector_zero);
            }
            msg_poses_trajectory_reshaped.header.stamp=ros::Time::now();
            msg_poses_trajectory_reshaped.header.frame_id="world";
            pub_poses_trajectory_reshaped.publish(msg_poses_trajectory_reshaped);
            ROS_WARN("Publishing updated trajectory with now %d poses!",int(msg_poses_trajectory_reshaped.poses.size()));
            time_last_button_press_left=ros::Time::now();
        }
    }

    this->lock_.unlock();
}

// Custom Callback Queue
////////////////////////////////////////////////////////////////////////////////
// custom callback queue thread
void GazeboRosForceGravity::QueueThread()
{
  static const double timeout = 0.01;

  while (this->rosnode_->ok())
  {
    this->queue_.callAvailable(ros::WallDuration(timeout));
  }
}

}
