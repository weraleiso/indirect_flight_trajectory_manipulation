
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

//===========================
//   DRONESPACE RESHAPING
//       "Hand Mapping"
//    W.A. Isop & M. Hrlec
//    2018 @ ICG (TU-Graz)
//===========================

// References:
// ---

#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>
#include <trajectory_reshaping_test/TestConfig.h>

#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <math.h>
#include <std_msgs/Float32MultiArray.h>
#include <tf/transform_datatypes.h>
#include <iostream>
#include <vector>
//#include <Eigen/Core>
//#include <Eigen/Dense>
//#include <Eigen/Geometry>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>

class Test {
 private:
  ros::NodeHandle nh;

  ros::Publisher pub_posestmpd_bebop2camera_command;
  ros::Subscriber sub_posearray_planner_global_path_points;
  ros::Subscriber sub_posestmpd_realistichandright_location;
  ros::Subscriber sub_pose_bebop2actor_location;

  geometry_msgs::PoseStamped msg_posestmpd_bebop2camera_command;
  geometry_msgs::PoseStamped msg_posestmpd_realistichandright_location;
  geometry_msgs::Pose msg_pose_bebop2actor_location;
  geometry_msgs::PoseArray msg_posearray_planner_global_path_points;

  int PAR_BUFFERED_PUBLISH_RATE;
  double PAR_TAKEOFF_SETPOINT_X;
  double PAR_TAKEOFF_SETPOINT_Y;
  double PAR_TAKEOFF_SETPOINT_Z;
  double PAR_BUFFERED_SETPOINT_SAT_X;
  double PAR_BUFFERED_SETPOINT_SAT_Y;
  double PAR_BUFFERED_SETPOINT_SAT_Z;
  double PAR_BUFFERED_SETPOINT_SAT_YAW;

  dynamic_reconfigure::Server<
      trajectory_reshaping_test::TestConfig>
      server;
  dynamic_reconfigure::Server<
      trajectory_reshaping_test::TestConfig>::CallbackType f;

 public:
  Test();

  void sub_posestmpd_realistichandright_location_callback(
      const geometry_msgs::PoseStamped::ConstPtr &msg);
  void sub_pose_bebop2actor_location_callback(
      const geometry_msgs::Pose::ConstPtr &msg);
  void sub_posearray_planner_global_path_points_callback(
      const geometry_msgs::PoseArray::ConstPtr &msg);
  void adaptViewpoint();
  void dyn_reconf_callback(
      trajectory_reshaping_test::TestConfig &config,
      uint32_t level);
  void run();
};
