/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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
 * Desc: A dynamic controller plugin that performs generic force interface.
 * Author: John Hsu
 * Date: 24 Sept 2008
 */

#ifndef GAZEBO_ROS_FORCE_HH
#define GAZEBO_ROS_FORCE_HH

#include <string>

// Custom Callback Queue
#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/Joy.h>
#include <gazebo_msgs/SpawnModel.h>
#include <gazebo_msgs/DeleteModel.h>
#include <gazebo_msgs/ApplyBodyWrench.h>
#include <tf/transform_listener.h>


#include <ros/ros.h>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

#include <gazebo/physics/physics.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>


namespace gazebo
{
/// @addtogroup gazebo_dynamic_plugins Gazebo ROS Dynamic Plugins
/// @{
/** \defgroup GazeboRosForceGravity Plugin XML Reference and Example

  \brief Ros Force Plugin.
  
  This is a Plugin that collects data from a ROS topic and applies wrench to a body accordingly.

  Example Usage:
  \verbatim
      <gazebo>
        <plugin filename="libgazebo_ros_force.so" name="gazebo_ros_force">
          <bodyName>box_body</bodyName>
          <topicName>box_force</topicName>
        </plugin>
      </gazebo>
  \endverbatim
 
\{
*/

/**
           .
 
*/

class GazeboRosForceGravity : public ModelPlugin
{
  /// \brief Constructor
  public: GazeboRosForceGravity();

  /// \brief Destructor
  public: virtual ~GazeboRosForceGravity();

  // Documentation inherited
  protected: void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

  // Documentation inherited
  protected: virtual void UpdateChild();

  /// \brief call back when a Wrench message is published
  /// \param[in] _msg The Incoming ROS message representing the new force to exert.
  private: void UpdateObjectForce(const geometry_msgs::Wrench::ConstPtr& _msg);

  /// \brief The custom callback queue thread function.
  private: void QueueThread();

  /// \brief A pointer to the gazebo world.
  private: physics::WorldPtr world_;

  /// \brief A pointer to the Link, where force is applied
  private: physics::LinkPtr link_;

  /// \brief A pointer to the ROS node.  A node will be instantiated if it does not exist.
  private: ros::NodeHandle* rosnode_;
  private: ros::Subscriber sub_;

  /// \brief A mutex to lock access to fields that are used in ROS message callbacks
  private: boost::mutex lock_;

  /// \brief ROS Wrench topic name inputs
  private: std::string topic_name_;
  /// \brief The Link this plugin is attached to, and will exert forces on.
  private: std::string link_name_;

  /// \brief for setting ROS name space
  private: std::string robot_namespace_;

  // Custom parameters
  private: std::string model_trajectory_point_prefix_;
  private: double hand_scale_translational_max_;
  private: double force_velocity_scale_translational_max_;
  private: double force_min_distance_threshold_;
  private: double force_max_distance_threshold_;
  private: int use_hands_option_;
  private: bool mirror_around_closest_point_;
  private: int interaction_mode_;
  private: int mockup_mode_;
  private: int force_mode_;
  private: int pushpull_mode_;

  // Custom Callback Queue
  private: ros::CallbackQueue queue_;
  /// \brief Thead object for the running callback Thread.
  private: boost::thread callback_queue_thread_;
  /// \brief Container for the wrench force that this plugin exerts on the body.
  private: geometry_msgs::Wrench wrench_msg_;

  // Pointer to the update event connection
  private: event::ConnectionPtr update_connection_;

    // Custom interfaces
public:
    void sub_trajectory_points_planner_callback(const geometry_msgs::PoseArray::ConstPtr&);
    void sub_trajectory_points_planner_interpolated_original_callback(const geometry_msgs::PoseArray::ConstPtr&);
    void sub_posestmpd_hand_left_callback(const geometry_msgs::PoseStamped::ConstPtr&);
    void sub_posestmpd_hand_right_callback(const geometry_msgs::PoseStamped::ConstPtr&);
    void sub_joy_hand_left_callback(const sensor_msgs::Joy::ConstPtr&);
    void sub_joy_hand_right_callback(const sensor_msgs::Joy::ConstPtr&);
    void sub_pose_bebop2_camera_callback(const geometry_msgs::Pose::ConstPtr&);
    void sub_posestmpd_bebop2_camera_current_view_distance_callback(const geometry_msgs::PoseStamped::ConstPtr&);
    void sub_pose_bebop2_actor_callback(const geometry_msgs::Pose::ConstPtr&);

    void cleanup_trajectory_points(std::string,int,int);
    void spawn_trajectory_point(geometry_msgs::PoseArray,std::string,int,int);

    tf::TransformListener listener;
    //tf::StampedTransform tf_shoulder_hand_left;//,tf_world_hand_left;
    //tf::StampedTransform tf_shoulder_hand_right;//,tf_world_hand_right;
    tf::Transform tf_world_hand_left;
    tf::Transform tf_world_hand_right;

    ros::Publisher pub_spawn_points_success;
    ros::Publisher pub_trajectory_points_planner_forces;
    ros::Publisher pub_clicked_point_valid;
    ros::Publisher pub_poses_trajectory_reshaped;
    ros::Publisher pub_pose_trajectory_point_closest_to_hand;
    ros::Publisher pub_pose_bebop2_camera_desired_view;
    ros::Subscriber sub_trajectory_points_planner_interpolated_original;
    ros::Subscriber sub_trajectory_points_planner;
    ros::Subscriber sub_pose_bebop2_camera;
    ros::Subscriber sub_posestmpd_bebop2_camera_current_view_distance;
    ros::Subscriber sub_pose_bebop2_actor;
    ros::Subscriber sub_posestmpd_hand_left,sub_joy_hand_left;
    ros::Subscriber sub_posestmpd_hand_right,sub_joy_hand_right;
    ros::ServiceClient gazebo_spawn_model_client;
    ros::ServiceClient gazebo_delete_model_client;

    math::Pose pose_hand_right_current_along_path;

    geometry_msgs::Pose msg_pose_bebop2_camera;
    geometry_msgs::Pose msg_pose_bebop2_actor;
    geometry_msgs::Pose msg_pose_trajectory_point_closest_to_hand;
    geometry_msgs::PoseArray msg_poses_trajectory_points;
    geometry_msgs::PoseArray msg_planner_path_points_interpolated_original;
    geometry_msgs::PoseStamped msg_posestmpd_hand_left;
    geometry_msgs::PoseStamped msg_posestmpd_hand_right;
    geometry_msgs::PoseStamped msg_posestmpd_bebop2_camera_current_view_distance;
    sensor_msgs::Joy msg_joy_hand_left;
    sensor_msgs::Joy msg_joy_hand_right;
    geometry_msgs::PointStamped msg_clicked_point_valid;
    int trajectory_points_counter;
    bool lock_apply_force;

    ros::Time time_since_last_button_press;
    ros::Time time_last_button_press;
    ros::Time time_last_button_press_left;

    // Interacting with view
    geometry_msgs::Pose msg_pose_desired_view;
    ros::Time time_last_button_press_desired_view;
    double yaw_desired_view;
    double pitch_desired_view;
};
/** \} */
/// @}
}
#endif
