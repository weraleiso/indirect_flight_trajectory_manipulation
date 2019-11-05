/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

//==============================
//   TRAJECTORY RESHAPING
//   "World Visualization"
//   W.A. Isop & M. Hrlec
//   2018 @ ICG (TU-Graz)
//==============================

// References:
// ---



#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <gazebo_msgs/LinkStates.h>
#include <gazebo_msgs/LinkState.h>
#include <geometry_msgs/Vector3.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <gazebo_msgs/SpawnModel.h>
#include <gazebo_msgs/ApplyBodyWrench.h>
//#include <ds_setpoint_buffer/WorldVisualizerConfig.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <iostream>
#include <fstream>
#include <math.h>
#include <vector>

//#include <Eigen/Core>
//#include <Eigen/Dense>
//#include <Eigen/Geometry>

#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>



class WorldVisualizer {

    ros::NodeHandle nh;

    tf::TransformBroadcaster broadcaster_;

    ros::Publisher pub_buffered_setpoint;
    ros::Subscriber sub_interface_setpoint;
    ros::Subscriber sub_drone_position;
    //ros::Subscriber sub_virtual_hand_mapped;

    ros::Subscriber sub_trajectory_points_planner;
    ros::Subscriber sub_trajectory_points_planner_forces;
    ros::Subscriber sub_lnkstats_position_actual;
    //ros::Publisher pub_msg_lnkstat_pose_hand_right;

    geometry_msgs::PoseStamped msg_posestmpd_drone_setpoint_buffered;
    geometry_msgs::PoseStamped msg_posestmpd_drone_setpoint_out;
    geometry_msgs::PoseStamped msg_posestmpd_drone_pose;

    geometry_msgs::PoseStamped msg_virtual_hand_pose;
    gazebo_msgs::LinkStates msg_lnkstats_gazebo;
    gazebo_msgs::LinkState msg_lnkstat_pose_hand_right;

    int PAR_BUFFERED_PUBLISH_RATE;
    double PAR_TAKEOFF_SETPOINT_X;
    double PAR_TAKEOFF_SETPOINT_Y;
    double PAR_TAKEOFF_SETPOINT_Z;
    double PAR_BUFFERED_SETPOINT_SAT_X;
    double PAR_BUFFERED_SETPOINT_SAT_Y;
    double PAR_BUFFERED_SETPOINT_SAT_Z;
    double PAR_BUFFERED_SETPOINT_SAT_YAW;

    //ros::Publisher pub_marker_world;
    //ros::Publisher pub_marker_hand;

    //ros::ServiceClient gazebo_spawn_client;
    //ros::ServiceClient gazebo_force_client;

    geometry_msgs::PoseArray msg_poses_trajectory_points_planner;
    geometry_msgs::PoseArray msg_poses_trajectory_points_planner_forces;
    int mdl_traj_point_index;
    std::string trajectory_point_mdl_prfx;
    int i_trajectory_points_counter,i_trajectory_points_forces_counter;

    float par_gravitational_constant_default;
    float par_mass_hand_default;
    float par_mass_trajectory_point_default;

    bool apply_force;

    float par_scale_translation;

    ros::Publisher pub_marker_drone_boundary;
    ros::Publisher pub_marker_hand;
    ros::Publisher pub_marker_path;
    ros::Publisher pub_marker_array_path;
    ros::Publisher pub_marker_obstacle_spheres;
    ros::Publisher pub_marker_obstacle_spheres_transparent;
    //visualization_msgs::Marker marker_hand_sphere;
    //visualization_msgs::Marker marker_hand_terrain;
    //visualization_msgs::Marker marker_path_points;
    //visualization_msgs::Marker marker_path_lines;
    //visualization_msgs::Marker marker_path_terrain;

    visualization_msgs::MarkerArray marker_array_forces;

public:
    WorldVisualizer()
    {
        // LOAD PARAMETERS
        nh.getParam("trajectory_reshaping_world_visualizer_node/par_system/par_buffered_publish_rate", PAR_BUFFERED_PUBLISH_RATE);
        nh.getParam("trajectory_reshaping_world_visualizer_node/par_geometry/par_takeoff_setpoint_x", PAR_TAKEOFF_SETPOINT_X);
        nh.getParam("trajectory_reshaping_world_visualizer_node/par_geometry/par_takeoff_setpoint_y", PAR_TAKEOFF_SETPOINT_Y);
        nh.getParam("trajectory_reshaping_world_visualizer_node/par_geometry/par_takeoff_setpoint_z", PAR_TAKEOFF_SETPOINT_Z);
        nh.getParam("trajectory_reshaping_world_visualizer_node/par_geometry/par_buffered_setpoint_sat_x", PAR_BUFFERED_SETPOINT_SAT_X);
        nh.getParam("trajectory_reshaping_world_visualizer_node/par_geometry/par_buffered_setpoint_sat_y", PAR_BUFFERED_SETPOINT_SAT_Y);
        nh.getParam("trajectory_reshaping_world_visualizer_node/par_geometry/par_buffered_setpoint_sat_z", PAR_BUFFERED_SETPOINT_SAT_Z);
        nh.getParam("trajectory_reshaping_world_visualizer_node/par_geometry/par_buffered_setpoint_sat_yaw", PAR_BUFFERED_SETPOINT_SAT_YAW);

        nh.getParam("trajectory_reshaping_world_visualizer_node/par_gravitational_force/par_gravitational_constant_default", par_gravitational_constant_default);
        nh.getParam("trajectory_reshaping_world_visualizer_node/par_gravitational_force/par_mass_hand_default", par_mass_hand_default);
        nh.getParam("trajectory_reshaping_world_visualizer_node/par_hand_mapping/par_scale_translation", par_scale_translation);

        nh.getParam("trajectory_reshaping_world_visualizer_node/par_gravitational_force/par_mass_trajectory_point_default", par_mass_trajectory_point_default);


        // Visualization marker publishers
        pub_marker_drone_boundary=nh.advertise<visualization_msgs::Marker>("marker_drone_boundary", 0);
        pub_marker_hand=nh.advertise<visualization_msgs::Marker>("marker_hand_sphere", 0);
        pub_marker_path=nh.advertise<visualization_msgs::Marker>("marker_path", 0);
        pub_marker_array_path=nh.advertise<visualization_msgs::MarkerArray>("marker_array_path", 0);
        pub_marker_obstacle_spheres=nh.advertise<visualization_msgs::Marker>("marker_obstacle_spheres", 0);
        pub_marker_obstacle_spheres_transparent=nh.advertise<visualization_msgs::Marker>("marker_obstacle_spheres_transparent", 0);

        pub_buffered_setpoint = nh.advertise<geometry_msgs::PoseStamped>("drone/setpoint/pose", 1);
        sub_interface_setpoint = nh.subscribe("drone/setpoint/pose_buffered", 1, &WorldVisualizer::sub_buffered_pose_callback, this);
        sub_drone_position = nh.subscribe("/reshaping/bebop2actor/ground_truth/pose", 1, &WorldVisualizer::sub_drone_position_callback, this);
        //sub_pose_bebop2actor_location = nh.subscribe("/reshaping/bebop2actor/ground_truth/pose", 1, &AdaptiveView::sub_pose_bebop2actor_location_callback, this);

        //sub_virtual_hand_mapped = nh.subscribe("/vrpn_client_node/DTrack/pose", 1, &WorldVisualizer::sub_virtual_hand_mapped_callback, this); // /vrpn_client_node/DTrack/pose

        //pub_marker_world = nh.advertise<visualization_msgs::Marker>("rviz_marker_world", 0 );
        //pub_marker_hand = nh.advertise<visualization_msgs::Marker>("rviz_marker_hand", 0 );

        //gazebo_spawn_client = nh.serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_urdf_model");
        //gazebo_force_client = nh.serviceClient<gazebo_msgs::ApplyBodyWrench>("/gazebo/apply_body_wrench");
        sub_trajectory_points_planner = nh.subscribe("/planner/global_path_points", 1, &WorldVisualizer::sub_trajectory_points_planner_callback, this);
        sub_trajectory_points_planner_forces = nh.subscribe("/planner/global_path_points/forces", 1, &WorldVisualizer::sub_trajectory_points_planner_forces_callback, this);

        //pub_msg_lnkstat_pose_hand_right = nh.advertise<gazebo_msgs::LinkState>("/gazebo/set_link_state", 1);
        sub_lnkstats_position_actual = nh.subscribe("gazebo/link_states", 1, &WorldVisualizer::sub_gazebo_linkstats, this);

        msg_posestmpd_drone_setpoint_out.header.stamp=ros::Time::now();
        msg_posestmpd_drone_setpoint_out.header.frame_id="map";
        msg_posestmpd_drone_setpoint_out.pose.position.x=PAR_TAKEOFF_SETPOINT_X;
        msg_posestmpd_drone_setpoint_out.pose.position.y=PAR_TAKEOFF_SETPOINT_Y;
        msg_posestmpd_drone_setpoint_out.pose.position.z=PAR_TAKEOFF_SETPOINT_Z;
        msg_posestmpd_drone_setpoint_out.pose.orientation.x=0.0;
        msg_posestmpd_drone_setpoint_out.pose.orientation.y=0.0;
        msg_posestmpd_drone_setpoint_out.pose.orientation.z=0.0;
        msg_posestmpd_drone_setpoint_out.pose.orientation.w=1.0;

        msg_posestmpd_drone_setpoint_buffered.header.stamp=ros::Time::now();
        msg_posestmpd_drone_setpoint_buffered.header.frame_id="map";
        msg_posestmpd_drone_setpoint_buffered.pose=msg_posestmpd_drone_setpoint_out.pose;
        /*position.x=1.5;
        msg_posestmpd_drone_setpoint_buffered.pose.position.y=2.0;
        msg_posestmpd_drone_setpoint_buffered.pose.position.z=1.0;
        msg_posestmpd_drone_setpoint_buffered.pose.orientation.x=0.0;
        msg_posestmpd_drone_setpoint_buffered.pose.orientation.y=0.0;
        msg_posestmpd_drone_setpoint_buffered.pose.orientation.z=0.0;
        msg_posestmpd_drone_setpoint_buffered.pose.orientation.w=1.0;*/

        msg_posestmpd_drone_pose.header.stamp=ros::Time::now();
        msg_posestmpd_drone_pose.header.frame_id="map";
        msg_posestmpd_drone_pose.pose.position.x=0.0;
        msg_posestmpd_drone_pose.pose.position.y=0.0;
        msg_posestmpd_drone_pose.pose.position.z=0.0;
        msg_posestmpd_drone_pose.pose.orientation.x=0.0;
        msg_posestmpd_drone_pose.pose.orientation.y=0.0;
        msg_posestmpd_drone_pose.pose.orientation.z=0.0;
        msg_posestmpd_drone_pose.pose.orientation.w=1.0;

        msg_virtual_hand_pose.pose.position.x=0.0;
        msg_virtual_hand_pose.pose.position.y=0.0;
        msg_virtual_hand_pose.pose.position.z=0.0;
        msg_virtual_hand_pose.pose.orientation.w=1.0;
        msg_virtual_hand_pose.pose.orientation.x=0.0;
        msg_virtual_hand_pose.pose.orientation.y=0.0;
        msg_virtual_hand_pose.pose.orientation.z=0.0;

        i_trajectory_points_counter=0;
        i_trajectory_points_forces_counter=0;
        trajectory_point_mdl_prfx="mdl_traj_point_";
        apply_force=false;
        mdl_traj_point_index=0;

        msg_lnkstat_pose_hand_right.link_name="hand_right::base_link";//link_hand_right";
        msg_lnkstat_pose_hand_right.reference_frame="ground_plane::link";
        msg_lnkstat_pose_hand_right.twist.angular.x=0.0;
        msg_lnkstat_pose_hand_right.twist.angular.y=0.0;
        msg_lnkstat_pose_hand_right.twist.angular.z=0.0;
        msg_lnkstat_pose_hand_right.twist.linear.x=0.0;
        msg_lnkstat_pose_hand_right.twist.linear.y=0.0;
        msg_lnkstat_pose_hand_right.twist.linear.z=0.0;
    }

    void sub_gazebo_linkstats(const gazebo_msgs::LinkStates::ConstPtr msg)
    {
        msg_lnkstats_gazebo = (*msg);

        // Prepare for visualization of trajectory
        visualization_msgs::Marker marker_hand_sphere;
        visualization_msgs::Marker marker_hand_terrain;
        visualization_msgs::Marker marker_hand_right_sphere;
        visualization_msgs::Marker marker_hand_right_terrain;
        visualization_msgs::Marker marker_path_points;
        visualization_msgs::Marker marker_path_spheres; // Not instantiated as member variable, lets see if it works
        visualization_msgs::Marker marker_path_lines;
        visualization_msgs::Marker marker_path_terrain;
        visualization_msgs::Marker marker_path_terrain_spheres;
        visualization_msgs::Marker marker_path_planar_lines;
        visualization_msgs::Marker marker_drone_boundary;
        visualization_msgs::Marker marker_obstacle_spheres;
        visualization_msgs::Marker marker_obstacle_spheres_transparent;

        // ADDING OBSTACLE OBJECTS
        marker_obstacle_spheres.header.frame_id = "world";
        marker_obstacle_spheres.header.stamp = ros::Time::now();
        marker_obstacle_spheres.ns = "obstacle_spheres";
        marker_obstacle_spheres.id = 93;
        marker_obstacle_spheres.type = visualization_msgs::Marker::SPHERE_LIST;
        marker_obstacle_spheres.action = visualization_msgs::Marker::ADD;
        marker_obstacle_spheres.pose.position.x = 0.0;
        marker_obstacle_spheres.pose.position.y = 0.0;
        marker_obstacle_spheres.pose.position.z = 0.0;
        marker_obstacle_spheres.pose.orientation.x = 0.0;
        marker_obstacle_spheres.pose.orientation.y = 0.0;
        marker_obstacle_spheres.pose.orientation.z = 0.0;
        marker_obstacle_spheres.pose.orientation.w = 1.0;
        marker_obstacle_spheres.scale.x = 2.0;
        marker_obstacle_spheres.scale.y = 2.0;
        marker_obstacle_spheres.scale.z = 2.0;
        marker_obstacle_spheres.color.a=1.0;
        marker_obstacle_spheres.color.r=1.0;
        marker_obstacle_spheres.color.g=0.0;
        marker_obstacle_spheres.color.b=0.0;

        float x_appear=20.0;
        if(msg_posestmpd_drone_pose.pose.position.x>=10.0)x_appear=20.0;
        //if(msg_posestmpd_drone_pose.pose.position.x>=20.0)x_appear=30.0;
        if(msg_posestmpd_drone_pose.pose.position.x>=30.0)x_appear=40.0;
        //if(msg_posestmpd_drone_pose.pose.position.x>=40.0)x_appear=50.0;
        if(msg_posestmpd_drone_pose.pose.position.x>=50.0)x_appear=60.0;
        //if(msg_posestmpd_drone_pose.pose.position.x>=60.0)x_appear=70.0;
        if(msg_posestmpd_drone_pose.pose.position.x>=70.0)x_appear=80.0;
        //if(msg_posestmpd_drone_pose.pose.position.x>=80.0)x_appear=90.0;
        if(msg_posestmpd_drone_pose.pose.position.x>=90.0)x_appear=100.0;

        for(float x_obstacle=20.0;x_obstacle<=x_appear;x_obstacle=x_obstacle+20.0)
        {
            geometry_msgs::Point p_obstacle_spheres;
            p_obstacle_spheres.x=x_obstacle;
            p_obstacle_spheres.y=0.0;
            p_obstacle_spheres.z=1.0;
            marker_obstacle_spheres.points.push_back(p_obstacle_spheres);
        }
        pub_marker_obstacle_spheres.publish(marker_obstacle_spheres);

        /*
        marker_obstacle_spheres_transparent.header.frame_id = "world";
        marker_obstacle_spheres_transparent.header.stamp = ros::Time::now();
        marker_obstacle_spheres_transparent.ns = "obstacle_spheres_transparent";
        marker_obstacle_spheres_transparent.id = 94;
        marker_obstacle_spheres_transparent.type = visualization_msgs::Marker::SPHERE_LIST;
        marker_obstacle_spheres_transparent.action = visualization_msgs::Marker::ADD;
        marker_obstacle_spheres_transparent.pose.position.x = 0.0;
        marker_obstacle_spheres_transparent.pose.position.y = 0.0;
        marker_obstacle_spheres_transparent.pose.position.z = 0.0;
        marker_obstacle_spheres_transparent.pose.orientation.x = 0.0;
        marker_obstacle_spheres_transparent.pose.orientation.y = 0.0;
        marker_obstacle_spheres_transparent.pose.orientation.z = 0.0;
        marker_obstacle_spheres_transparent.pose.orientation.w = 1.0;
        marker_obstacle_spheres_transparent.scale.x = 2.0;
        marker_obstacle_spheres_transparent.scale.y = 2.0;
        marker_obstacle_spheres_transparent.scale.z = 2.0;
        marker_obstacle_spheres_transparent.color.a=0.5;
        marker_obstacle_spheres_transparent.color.r=1.0;
        marker_obstacle_spheres_transparent.color.g=0.0;
        marker_obstacle_spheres_transparent.color.b=0.0;
        for(float x_obstacle_transparent=60.0;x_obstacle_transparent<=100.0;x_obstacle_transparent=x_obstacle_transparent+10.0)
        {
            geometry_msgs::Point p_obstacle_spheres_transparent;
            p_obstacle_spheres_transparent.x=x_obstacle_transparent;
            p_obstacle_spheres_transparent.y=0.0;
            p_obstacle_spheres_transparent.z=1.0;
            marker_obstacle_spheres_transparent.points.push_back(p_obstacle_spheres_transparent);
        }
        pub_marker_obstacle_spheres_transparent.publish(marker_obstacle_spheres_transparent);*/


        marker_drone_boundary.header.frame_id = "world"; //htc_vive was original!
        marker_drone_boundary.header.stamp = ros::Time::now();
        marker_drone_boundary.ns = "drone_sphere";
        marker_drone_boundary.id = 95;
        marker_drone_boundary.type = visualization_msgs::Marker::SPHERE;
        marker_drone_boundary.action = visualization_msgs::Marker::ADD;
        marker_drone_boundary.pose.position.x = msg_posestmpd_drone_pose.pose.position.x;
        marker_drone_boundary.pose.position.y = msg_posestmpd_drone_pose.pose.position.y;
        marker_drone_boundary.pose.position.z = msg_posestmpd_drone_pose.pose.position.z;
        marker_drone_boundary.pose.orientation.x = 0.0;
        marker_drone_boundary.pose.orientation.y = 0.0;
        marker_drone_boundary.pose.orientation.z = 0.0;
        marker_drone_boundary.pose.orientation.w = 1.0;
        marker_drone_boundary.scale.x = 0.5;
        marker_drone_boundary.scale.y = 0.5;
        marker_drone_boundary.scale.z = 0.5;
        marker_drone_boundary.color.a = 0.667; // For textured mesh set everything to 0! Otherwise don't forget to set the alpha!
        marker_drone_boundary.color.r = 0.0;
        marker_drone_boundary.color.g = 0.0;
        marker_drone_boundary.color.b = 1.0;
        pub_marker_drone_boundary.publish(marker_drone_boundary);

        // Publish path lines
        marker_path_lines.header.frame_id = "world";
        marker_path_lines.header.stamp = ros::Time::now();
        marker_path_lines.ns = "path";
        marker_path_lines.id = 102;
        marker_path_lines.type = visualization_msgs::Marker::LINE_STRIP;
        marker_path_lines.action = visualization_msgs::Marker::ADD;
        marker_path_lines.scale.x = 0.05;

        // Publish path points
        marker_path_points.header.frame_id = "world";
        marker_path_points.header.stamp = ros::Time::now();
        marker_path_points.ns = "path";
        marker_path_points.id = 101;
        marker_path_points.type = visualization_msgs::Marker::POINTS;
        marker_path_points.action = visualization_msgs::Marker::ADD;
        marker_path_points.pose.position.x = 0.0;
        marker_path_points.pose.position.y = 0.0;
        marker_path_points.pose.position.z = 0.0;
        marker_path_points.pose.orientation.x = 0.0;
        marker_path_points.pose.orientation.y = 0.0;
        marker_path_points.pose.orientation.z = 0.0;
        marker_path_points.pose.orientation.w = 1.0;
        marker_path_points.scale.x = 0.075;
        marker_path_points.scale.y = 0.075;
        marker_path_points.scale.y = 0.075;

        // Publish path spheres
        marker_path_spheres.header.frame_id = "world";
        marker_path_spheres.header.stamp = ros::Time::now();
        marker_path_spheres.ns = "path";
        marker_path_spheres.id = 98;
        marker_path_spheres.type = visualization_msgs::Marker::SPHERE_LIST;
        marker_path_spheres.action = visualization_msgs::Marker::ADD;
        marker_path_spheres.pose.position.x = 0.0;
        marker_path_spheres.pose.position.y = 0.0;
        marker_path_spheres.pose.position.z = 0.0;
        marker_path_spheres.pose.orientation.x = 0.0;
        marker_path_spheres.pose.orientation.y = 0.0;
        marker_path_spheres.pose.orientation.z = 0.0;
        marker_path_spheres.pose.orientation.w = 1.0;
        marker_path_spheres.scale.x = 0.5; //0.5//0.1//0.2
        marker_path_spheres.scale.y = 0.5;
        marker_path_spheres.scale.z = 0.5;
        marker_path_spheres.color.a=0.2; //0.333
        marker_path_spheres.color.r=0.0;
        marker_path_spheres.color.g=1.0;
        marker_path_spheres.color.b=0.0;

        // Add supportive lines for terrain
        marker_path_terrain.header.frame_id = "world";
        marker_path_terrain.header.stamp = ros::Time::now();
        marker_path_terrain.ns = "path";
        marker_path_terrain.id = 103;
        marker_path_terrain.type = visualization_msgs::Marker::LINE_LIST;
        marker_path_terrain.action = visualization_msgs::Marker::ADD;
        marker_path_terrain.scale.x = 0.0125;

        // Publish path TERRAIN SPHERES
        marker_path_terrain_spheres.header.frame_id = "world";
        marker_path_terrain_spheres.header.stamp = ros::Time::now();
        marker_path_terrain_spheres.ns = "path";
        marker_path_terrain_spheres.id = 96;
        marker_path_terrain_spheres.type = visualization_msgs::Marker::SPHERE_LIST;
        marker_path_terrain_spheres.action = visualization_msgs::Marker::ADD;
        marker_path_terrain_spheres.pose.position.x = 0.0;
        marker_path_terrain_spheres.pose.position.y = 0.0;
        marker_path_terrain_spheres.pose.position.z = 0.0;
        marker_path_terrain_spheres.pose.orientation.x = 0.0;
        marker_path_terrain_spheres.pose.orientation.y = 0.0;
        marker_path_terrain_spheres.pose.orientation.z = 0.0;
        marker_path_terrain_spheres.pose.orientation.w = 1.0;
        marker_path_terrain_spheres.scale.x = 0.1;
        marker_path_terrain_spheres.scale.y = 0.1;
        marker_path_terrain_spheres.scale.z = 0.1;
        marker_path_terrain_spheres.color.a=0.5;
        marker_path_terrain_spheres.color.r=1.0;
        marker_path_terrain_spheres.color.g=1.0;
        marker_path_terrain_spheres.color.b=1.0;

        // Add supportive lines in the xy-plane
        marker_path_planar_lines.header.frame_id = "world";
        marker_path_planar_lines.header.stamp = ros::Time::now();
        marker_path_planar_lines.ns = "path";
        marker_path_planar_lines.id = 97;
        marker_path_planar_lines.type = visualization_msgs::Marker::LINE_LIST;
        marker_path_planar_lines.action = visualization_msgs::Marker::ADD;
        marker_path_planar_lines.scale.x = 0.01;
        marker_path_planar_lines.color.a=0.25;
        marker_path_planar_lines.color.r=1.0;
        marker_path_planar_lines.color.g=1.0;
        marker_path_planar_lines.color.b=1.0;

        bool trajectory_point_found=false;
        for(int i=0;i<msg_lnkstats_gazebo.pose.size();i++) // Crawl all links
        {
            if (msg_lnkstats_gazebo.name[i].find("mdl_traj_point_")!=std::string::npos)
            {
                trajectory_point_found=true;

                std::string mdl_traj_point_name=msg_lnkstats_gazebo.name[i];
                mdl_traj_point_index=atoi(mdl_traj_point_name.substr(15,5).c_str());
                //ROS_WARN("pt_name %s, idx %d, i_pts %d, i_forces %d",msg_lnkstats_gazebo.name[i].c_str(),mdl_traj_point_index,i_trajectory_points_counter,i_trajectory_points_forces_counter);

                // Path LINES and POINTS
                geometry_msgs::Point p;
                p.x=msg_lnkstats_gazebo.pose[i].position.x;
                p.y=msg_lnkstats_gazebo.pose[i].position.y;
                p.z=msg_lnkstats_gazebo.pose[i].position.z;
                geometry_msgs::Point p_terrain;
                p_terrain.x=msg_lnkstats_gazebo.pose[i].position.x;
                p_terrain.y=msg_lnkstats_gazebo.pose[i].position.y;
                p_terrain.z=0.0;
                marker_path_points.points.push_back(p);
                marker_path_spheres.points.push_back(p);
                marker_path_lines.points.push_back(p);
                marker_path_terrain.points.push_back(p);
                marker_path_terrain.points.push_back(p_terrain);
                marker_path_terrain_spheres.points.push_back(p_terrain);

                if(mdl_traj_point_index==0 || mdl_traj_point_index==(i_trajectory_points_counter-1))
                {
                    // Color code start and end points
                    std_msgs::ColorRGBA color_path_lines;
                    color_path_lines.a = 0.9;
                    color_path_lines.r = 1.0;
                    color_path_lines.g = 165.0/256.0;
                    color_path_lines.b = 0.0;
                    marker_path_lines.colors.push_back(color_path_lines);
                    marker_path_points.colors.push_back(color_path_lines);
                }
                else
                {
                    // Color code rest of trajectory
                    std_msgs::ColorRGBA color_path_lines;
                    color_path_lines.a = 0.9;
                    color_path_lines.r = 0.0;
                    color_path_lines.g = 1.0;
                    color_path_lines.b = 0.0;
                    marker_path_lines.colors.push_back(color_path_lines);
                    marker_path_points.colors.push_back(color_path_lines);
                }
                // Color path TERRAIN lines
                std_msgs::ColorRGBA color_path_terrain;
                color_path_terrain.a = 0.5; // Don't forget to set the Alpha!
                color_path_terrain.r = 0.0;
                color_path_terrain.g = 1.0;//165.0/256.0;
                color_path_terrain.b = 0.0;
                marker_path_terrain.colors.push_back(color_path_terrain); // As we added two points already
                marker_path_terrain.colors.push_back(color_path_terrain);


                // Visualize forces applied to individual interpolation points
                if(i_trajectory_points_forces_counter==i_trajectory_points_counter)
                {
                    visualization_msgs::Marker marker_forces;
                    marker_forces.header.frame_id = "world";
                    marker_forces.header.stamp = ros::Time::now();
                    marker_forces.ns = "path";
                    marker_forces.id = 104+mdl_traj_point_index;
                    marker_forces.type = visualization_msgs::Marker::ARROW;
                    marker_forces.action = visualization_msgs::Marker::ADD;
                    marker_forces.color.a=0.9;
                    marker_forces.color.r=1.0;
                    marker_forces.color.g=0.0;
                    marker_forces.color.b=0.0;
                    marker_forces.scale.x=0.0;
                    marker_forces.scale.y=0.0;
                    //marker_forces.scale.z=1.0;
                    geometry_msgs::Point p_force_origin;
                    p_force_origin.x=msg_poses_trajectory_points_planner_forces.poses[mdl_traj_point_index].orientation.x;
                    p_force_origin.y=msg_poses_trajectory_points_planner_forces.poses[mdl_traj_point_index].orientation.y;
                    p_force_origin.z=msg_poses_trajectory_points_planner_forces.poses[mdl_traj_point_index].orientation.z;
                    geometry_msgs::Point p_force_direction_at_point;
                    p_force_direction_at_point.x=p.x;//+msg_poses_trajectory_points_planner_forces.poses[mdl_traj_point_index].position.x*3.0;
                    p_force_direction_at_point.y=p.y;//+msg_poses_trajectory_points_planner_forces.poses[mdl_traj_point_index].position.y*3.0;
                    p_force_direction_at_point.z=p.z;//+msg_poses_trajectory_points_planner_forces.poses[mdl_traj_point_index].position.z*3.0; // 5.0 scale for debug
                    marker_forces.points.push_back(p_force_origin);
                    marker_forces.points.push_back(p_force_direction_at_point);

                    // Add only visual aids in xy-plane for force influenced points
                    tf::Vector3 vec_trajectory_point_planner_force( msg_poses_trajectory_points_planner_forces.poses[mdl_traj_point_index].position.x,
                                                                    msg_poses_trajectory_points_planner_forces.poses[mdl_traj_point_index].position.y,
                                                                    msg_poses_trajectory_points_planner_forces.poses[mdl_traj_point_index].position.z);
                    if(vec_trajectory_point_planner_force.length()>0.0) // Double check threshold 0.01
                    {
                        geometry_msgs::Point p_planar;
                        p_planar.x=p.x-100.0;
                        p_planar.y=p.y;
                        p_planar.z=p.z;
                        marker_path_planar_lines.points.push_back(p_planar);
                        p_planar.x=p.x+100.0;
                        p_planar.y=p.y;
                        p_planar.z=p.z;
                        marker_path_planar_lines.points.push_back(p_planar);
                        p_planar.x=p.x;
                        p_planar.y=p.y-100.0;
                        p_planar.z=p.z;
                        marker_path_planar_lines.points.push_back(p_planar);
                        p_planar.x=p.x;
                        p_planar.y=p.y+100.0;
                        p_planar.z=p.z;
                        marker_path_planar_lines.points.push_back(p_planar);

                        marker_forces.scale.x=0.025;
                        marker_forces.scale.y=0.025;
                    }

                    marker_array_forces.markers.push_back(marker_forces);
                }
            }

            if (msg_lnkstats_gazebo.name[i].find("realistichandleft::base_link") != std::string::npos)
            {
                tf::Transform tf_hand_left;
                tf::Vector3 vec_hand_left(msg_lnkstats_gazebo.pose[i].position.x,msg_lnkstats_gazebo.pose[i].position.y,msg_lnkstats_gazebo.pose[i].position.z);
                tf_hand_left.setOrigin(vec_hand_left);
                tf::Quaternion qt_hand_left(msg_lnkstats_gazebo.pose[i].orientation.x,msg_lnkstats_gazebo.pose[i].orientation.y,msg_lnkstats_gazebo.pose[i].orientation.z,msg_lnkstats_gazebo.pose[i].orientation.w);
                tf_hand_left.setRotation(qt_hand_left);

                // VISUALIZING REALISTIC HAND POSE IN GAZEBOS WORLD COORDINDATES IN RVIZ!
                tf::StampedTransform tf_hand_left_stamped (tf_hand_left, ros::Time::now(), "world", "realistichandleft/base_link");
                broadcaster_.sendTransform(tf_hand_left_stamped);

                // Hand sphere Marker
                //marker_hand_sphere.header.frame_id = "reshaping/bebop2camera/htc_vive_shoulder_left"; //htc_vive was original!
                marker_hand_sphere.header.frame_id = "world"; //htc_vive was original!
                marker_hand_sphere.header.stamp = ros::Time::now();
                marker_hand_sphere.ns = "hand_sphere";
                marker_hand_sphere.id = 100;
                marker_hand_sphere.type = visualization_msgs::Marker::SPHERE;
                marker_hand_sphere.action = visualization_msgs::Marker::ADD;
                marker_hand_sphere.pose.position.x = vec_hand_left.getX();
                marker_hand_sphere.pose.position.y = vec_hand_left.getY();
                marker_hand_sphere.pose.position.z = vec_hand_left.getZ();
                marker_hand_sphere.pose.orientation.x = 0.0;
                marker_hand_sphere.pose.orientation.y = 0.0;
                marker_hand_sphere.pose.orientation.z = 0.0;
                marker_hand_sphere.pose.orientation.w = 1.0;
                marker_hand_sphere.scale.x = 0.25;
                marker_hand_sphere.scale.y = 0.25;
                marker_hand_sphere.scale.z = 0.25;
                marker_hand_sphere.color.a = 0.25; // For textured mesh set everything to 0! Otherwise don't forget to set the alpha!
                marker_hand_sphere.color.r = 0.0;
                marker_hand_sphere.color.g = 1.0;
                marker_hand_sphere.color.b = 0.0;
                pub_marker_hand.publish(marker_hand_sphere);

                // Hand terrain marker
                //marker_hand_terrain.header.frame_id = "reshaping/bebop2camera/htc_vive_shoulder_left";
                marker_hand_terrain.header.frame_id = "world";
                marker_hand_terrain.header.stamp = ros::Time::now();
                marker_hand_terrain.ns = "hand_terrain";
                marker_hand_terrain.id = 99;
                marker_hand_terrain.type = visualization_msgs::Marker::LINE_STRIP;
                marker_hand_terrain.action = visualization_msgs::Marker::ADD;
                marker_hand_terrain.scale.x = 0.05; //0.025;
                geometry_msgs::Point p_hand;
                p_hand.x=vec_hand_left.getX();
                p_hand.y=vec_hand_left.getY();
                p_hand.z=vec_hand_left.getZ();
                geometry_msgs::Point p_hand_terrain;
                p_hand_terrain.x=vec_hand_left.getX();
                p_hand_terrain.y=vec_hand_left.getY();
                p_hand_terrain.z=-50.0; // because in local coordinate frame of hand!
                marker_hand_terrain.points.push_back(p_hand);
                marker_hand_terrain.points.push_back(p_hand_terrain);
                // Color hand TERRAIN line
                std_msgs::ColorRGBA color_path_terrain;
                color_path_terrain.a = 0.5; // Don't forget to set the Alpha!
                color_path_terrain.r = 1.0;
                color_path_terrain.g = 165.0/256.0;
                color_path_terrain.b = 0.0;
                marker_hand_terrain.colors.push_back(color_path_terrain); // As we added two points already
                marker_hand_terrain.colors.push_back(color_path_terrain);
                pub_marker_hand.publish(marker_hand_terrain);
                marker_hand_terrain.points.clear();
            }

            if (msg_lnkstats_gazebo.name[i].find("realistichandright::base_link") != std::string::npos)
            {
                tf::Transform tf_hand_right;
                tf::Vector3 vec_hand_right(msg_lnkstats_gazebo.pose[i].position.x,msg_lnkstats_gazebo.pose[i].position.y,msg_lnkstats_gazebo.pose[i].position.z);
                tf_hand_right.setOrigin(vec_hand_right);
                tf::Quaternion qt_hand_right(msg_lnkstats_gazebo.pose[i].orientation.x,msg_lnkstats_gazebo.pose[i].orientation.y,msg_lnkstats_gazebo.pose[i].orientation.z,msg_lnkstats_gazebo.pose[i].orientation.w);
                tf_hand_right.setRotation(qt_hand_right);

                tf::StampedTransform tf_hand_right_stamped (tf_hand_right, ros::Time::now(), "world", "realistichandright/base_link");
                broadcaster_.sendTransform(tf_hand_right_stamped);

                // Hand sphere Marker
                //marker_hand_right_sphere.header.frame_id = "reshaping/bebop2camera/htc_vive_shoulder_right";
                marker_hand_right_sphere.header.frame_id = "world";
                marker_hand_right_sphere.header.stamp = ros::Time::now();
                marker_hand_right_sphere.ns = "hand_sphere";
                marker_hand_right_sphere.id = 90;
                marker_hand_right_sphere.type = visualization_msgs::Marker::SPHERE;
                marker_hand_right_sphere.action = visualization_msgs::Marker::ADD;
                marker_hand_right_sphere.pose.position.x = vec_hand_right.getX();
                marker_hand_right_sphere.pose.position.y = vec_hand_right.getY();
                marker_hand_right_sphere.pose.position.z = vec_hand_right.getZ();
                marker_hand_right_sphere.pose.orientation.x = 0.0;
                marker_hand_right_sphere.pose.orientation.y = 0.0;
                marker_hand_right_sphere.pose.orientation.z = 0.0;
                marker_hand_right_sphere.pose.orientation.w = 1.0;
                marker_hand_right_sphere.scale.x = 0.25;
                marker_hand_right_sphere.scale.y = 0.25;
                marker_hand_right_sphere.scale.z = 0.25;
                marker_hand_right_sphere.color.a = 0.25; // For textured mesh set everything to 0! Otherwise don't forget to set the alpha!
                marker_hand_right_sphere.color.r = 1.0;
                marker_hand_right_sphere.color.g = 0.0;
                marker_hand_right_sphere.color.b = 0.0;
                pub_marker_hand.publish(marker_hand_right_sphere);

                // Hand terrain marker
                //marker_hand_right_terrain.header.frame_id = "reshaping/bebop2camera/htc_vive_shoulder_right";
                marker_hand_right_terrain.header.frame_id = "world";
                marker_hand_right_terrain.header.stamp = ros::Time::now();
                marker_hand_right_terrain.ns = "hand_terrain";
                marker_hand_right_terrain.id = 91;
                marker_hand_right_terrain.type = visualization_msgs::Marker::LINE_STRIP;
                marker_hand_right_terrain.action = visualization_msgs::Marker::ADD;
                marker_hand_right_terrain.scale.x = 0.025;
                geometry_msgs::Point p_hand;
                p_hand.x=vec_hand_right.getX();
                p_hand.y=vec_hand_right.getY();
                p_hand.z=vec_hand_right.getZ();
                geometry_msgs::Point p_hand_terrain;
                p_hand_terrain.x=vec_hand_right.getX();
                p_hand_terrain.y=vec_hand_right.getY();
                p_hand_terrain.z=-50.0;
                marker_hand_right_terrain.points.push_back(p_hand);
                marker_hand_right_terrain.points.push_back(p_hand_terrain);
                // Color hand TERRAIN line
                std_msgs::ColorRGBA color_path_terrain;
                color_path_terrain.a = 0.5; // Don't forget to set the Alpha!
                color_path_terrain.r = 1.0;
                color_path_terrain.g = 165.0/256.0;
                color_path_terrain.b = 0.0;
                marker_hand_right_terrain.colors.push_back(color_path_terrain); // As we added two points already
                marker_hand_right_terrain.colors.push_back(color_path_terrain);
                pub_marker_hand.publish(marker_hand_right_terrain);
                marker_hand_right_terrain.points.clear();
            }
            if (msg_lnkstats_gazebo.name[i].find("tree::base_link") != std::string::npos)
            {
                tf::Transform tf_tree;
                tf::Vector3 vec_tree(msg_lnkstats_gazebo.pose[i].position.x,msg_lnkstats_gazebo.pose[i].position.y,msg_lnkstats_gazebo.pose[i].position.z);
                tf_tree.setOrigin(vec_tree);
                tf::Quaternion qt_tree(msg_lnkstats_gazebo.pose[i].orientation.x,msg_lnkstats_gazebo.pose[i].orientation.y,msg_lnkstats_gazebo.pose[i].orientation.z,msg_lnkstats_gazebo.pose[i].orientation.w);
                tf_tree.setRotation(qt_tree);
                tf::StampedTransform tf_tree_stamped (tf_tree, ros::Time::now(), "world", "tree/base_link");
                broadcaster_.sendTransform(tf_tree_stamped);
            }
            if (msg_lnkstats_gazebo.name[i].find("obstaclesphere::base_link") != std::string::npos)
            {
                tf::Transform tf_sphere;
                tf::Vector3 vec_sphere(msg_lnkstats_gazebo.pose[i].position.x,msg_lnkstats_gazebo.pose[i].position.y,msg_lnkstats_gazebo.pose[i].position.z);
                tf_sphere.setOrigin(vec_sphere);
                tf::Quaternion qt_sphere(msg_lnkstats_gazebo.pose[i].orientation.x,msg_lnkstats_gazebo.pose[i].orientation.y,msg_lnkstats_gazebo.pose[i].orientation.z,msg_lnkstats_gazebo.pose[i].orientation.w);
                tf_sphere.setRotation(qt_sphere);
                tf::StampedTransform tf_sphere_stamped (tf_sphere, ros::Time::now(), "world", "obstaclesphere/base_link");
                broadcaster_.sendTransform(tf_sphere_stamped);
            }
        }

        // Finally publish path lines
        if(trajectory_point_found)
        {
            pub_marker_path.publish(marker_path_points);
            pub_marker_path.publish(marker_path_spheres);
            pub_marker_path.publish(marker_path_lines);
            pub_marker_path.publish(marker_path_terrain);
            pub_marker_path.publish(marker_path_terrain_spheres);
            pub_marker_path.publish(marker_path_planar_lines);
            pub_marker_array_path.publish(marker_array_forces);
            marker_path_points.points.clear();
            marker_path_lines.points.clear();
            marker_array_forces.markers.clear();
        }

        // Latest naming Conventions in Gazebo:
        // name: ['ground_plane::link', 'world_model::world_link', 'bebop2_camera::bebop2_camera/base_link', 'bebop2_camera::bebop2_camera/imu_link', 'bebop2_camera::bebop2_camera/imugt_link', 'bebop2_camera::bebop2_camera/odometry_sensor1_link', 'bebop2_camera::bebop2_camera/odometry_sensorgt_link', 'bebop2_camera::bebop2_camera/rotor_0', 'bebop2_camera::bebop2_camera/rotor_1', 'bebop2_camera::bebop2_camera/rotor_2', 'bebop2_camera::bebop2_camera/rotor_3', 'bebop2_world::__default__', 'bebop2_actor::bebop2_actor/base_link', 'bebop2_actor::bebop2_actor/imu_link', 'bebop2_actor::bebop2_actor/imugt_link', 'bebop2_actor::bebop2_actor/odometry_sensor1_link', 'bebop2_actor::bebop2_actor/odometry_sensorgt_link', 'bebop2_actor::bebop2_actor/rotor_0', 'bebop2_actor::bebop2_actor/rotor_1', 'bebop2_actor::bebop2_actor/rotor_2', 'bebop2_actor::bebop2_actor/rotor_3']
        // 'bebop2camera::/reshaping/bebop2camera/odometry_sensor1_link', 'bebop2camera::/reshaping/bebop2camera/odometry_sensorgt_link', 'bebop2camera::/reshaping/bebop2camera/rotor_0', 'bebop2camera::/reshaping/bebop2camera/rotor_1', 'bebop2camera::/reshaping/bebop2camera/rotor_2', 'bebop2camera::/reshaping/bebop2camera/rotor_3', 'realistichandleft::base_link', 'realistichandright::base_link', 'world::__default__', 'tree::base_link'
    }

    void trajectory_points_apply_force(geometry_msgs::Vector3 trajectory_point_force,
                                       std::string trajectory_point_model_prefix,
                                       int trajectory_point_model_id)
    {
        std::stringstream str_model_name;
        str_model_name << trajectory_point_model_prefix << trajectory_point_model_id << "::" << "base_link";

        gazebo_msgs::ApplyBodyWrench wrench;
        wrench.request.body_name=str_model_name.str();
        wrench.request.reference_frame="world";
        wrench.request.wrench.force=trajectory_point_force;
        wrench.request.start_time=ros::Time(0);
        wrench.request.duration=ros::Duration(-1);
        //gazebo_force_client.call(wrench);
    }

    void sub_drone_position_callback(const geometry_msgs::Pose::ConstPtr& msg)
    {
        msg_posestmpd_drone_pose.header.stamp=ros::Time::now();
        msg_posestmpd_drone_pose.pose=(*msg);
    }

    void sub_buffered_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        geometry_msgs::PoseStamped msg_current_pose = (*msg);
        msg_posestmpd_drone_setpoint_buffered=msg_current_pose;
        msg_posestmpd_drone_setpoint_buffered.header.frame_id=msg_posestmpd_drone_setpoint_out.header.frame_id;
        ROS_INFO("BUFFERED incoming setpoint!");
    }

    // Receive global path points from planner
    void sub_trajectory_points_planner_callback(const geometry_msgs::PoseArray::ConstPtr& msg)
    {
        ROS_WARN("Received new planner trajectory! Updating visualization...");
        msg_poses_trajectory_points_planner = (*msg);
        i_trajectory_points_counter=msg_poses_trajectory_points_planner.poses.size();
        ROS_WARN("updated %d",i_trajectory_points_counter);
    }
    // Receive force vectors for vis.
    void sub_trajectory_points_planner_forces_callback(const geometry_msgs::PoseArray::ConstPtr& msg)
    {
        msg_poses_trajectory_points_planner_forces = (*msg);
        i_trajectory_points_forces_counter=msg_poses_trajectory_points_planner_forces.poses.size();
    }

    void run()
    {
        //ros::Rate loop_rate(PAR_BUFFERED_PUBLISH_RATE);

        while(ros::ok())
        {
            ros::spin();
            //loop_rate.sleep();
        }
    }
};



int main(int argc, char **argv)
{
    ros::init(argc, argv, "trajectory_reshaping_world_visualizer_node");
    WorldVisualizer world_visualizer;
    ROS_INFO("Initialized...");
    world_visualizer.run();
    ROS_INFO("Exiting...");
    return 0;
}
