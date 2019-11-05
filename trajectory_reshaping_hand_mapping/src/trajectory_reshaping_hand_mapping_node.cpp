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
#include <trajectory_reshaping_hand_mapping/HandMappingConfig.h>

#include <std_msgs/Float32MultiArray.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include <math.h>
#include <vector>
//#include <Eigen/Core>
//#include <Eigen/Dense>
//#include <Eigen/Geometry>

#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>



class HandMapping {

    ros::NodeHandle nh;

    ros::Publisher pub_buffered_setpoint;
    ros::Subscriber sub_interface_setpoint;
    ros::Subscriber sub_drone_position;

    geometry_msgs::PoseStamped msg_posestmpd_drone_setpoint_buffered;
    geometry_msgs::PoseStamped msg_posestmpd_drone_setpoint_out;
    geometry_msgs::PoseStamped msg_posestmpd_drone_pose;

    int PAR_BUFFERED_PUBLISH_RATE;
    double PAR_TAKEOFF_SETPOINT_X;
    double PAR_TAKEOFF_SETPOINT_Y;
    double PAR_TAKEOFF_SETPOINT_Z;
    double PAR_BUFFERED_SETPOINT_SAT_X;
    double PAR_BUFFERED_SETPOINT_SAT_Y;
    double PAR_BUFFERED_SETPOINT_SAT_Z;
    double PAR_BUFFERED_SETPOINT_SAT_YAW;

    dynamic_reconfigure::Server<trajectory_reshaping_hand_mapping::HandMappingConfig> server;
    dynamic_reconfigure::Server<trajectory_reshaping_hand_mapping::HandMappingConfig>::CallbackType f;


public:
    HandMapping()
    {
        // LOAD PARAMETERS
        nh.getParam("trajectory_reshaping_hand_mapping_node/par_system/par_buffered_publish_rate", PAR_BUFFERED_PUBLISH_RATE);
        nh.getParam("trajectory_reshaping_hand_mapping_node/par_geometry/par_takeoff_setpoint_x", PAR_TAKEOFF_SETPOINT_X);
        nh.getParam("trajectory_reshaping_hand_mapping_node/par_geometry/par_takeoff_setpoint_y", PAR_TAKEOFF_SETPOINT_Y);
        nh.getParam("trajectory_reshaping_hand_mapping_node/par_geometry/par_takeoff_setpoint_z", PAR_TAKEOFF_SETPOINT_Z);
        nh.getParam("trajectory_reshaping_hand_mapping_node/par_geometry/par_buffered_setpoint_sat_x", PAR_BUFFERED_SETPOINT_SAT_X);
        nh.getParam("trajectory_reshaping_hand_mapping_node/par_geometry/par_buffered_setpoint_sat_y", PAR_BUFFERED_SETPOINT_SAT_Y);
        nh.getParam("trajectory_reshaping_hand_mapping_node/par_geometry/par_buffered_setpoint_sat_z", PAR_BUFFERED_SETPOINT_SAT_Z);
        nh.getParam("trajectory_reshaping_hand_mapping_node/par_geometry/par_buffered_setpoint_sat_yaw", PAR_BUFFERED_SETPOINT_SAT_YAW);

        f = boost::bind(&HandMapping::dyn_reconf_callback,this, _1, _2);
        server.setCallback(f);

        pub_buffered_setpoint = nh.advertise<geometry_msgs::PoseStamped>("drone/setpoint/pose", 1);
        sub_interface_setpoint = nh.subscribe("drone/setpoint/pose_buffered", 1, &HandMapping::sub_buffered_pose_callback, this);
        sub_drone_position = nh.subscribe("mocapsystem/drone/location/pose", 1, &HandMapping::sub_drone_position_callback, this);

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
    }

    void sub_drone_position_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        msg_posestmpd_drone_pose.header.stamp=ros::Time::now();
        geometry_msgs::PoseStamped msg_current_pose = (*msg);
        msg_posestmpd_drone_pose=msg_current_pose;
    }

    void sub_buffered_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
            geometry_msgs::PoseStamped msg_current_pose = (*msg);
            msg_posestmpd_drone_setpoint_buffered=msg_current_pose;
            msg_posestmpd_drone_setpoint_buffered.header.frame_id=msg_posestmpd_drone_setpoint_out.header.frame_id;
            ROS_INFO("BUFFERED incoming setpoint!");
    }

    void dyn_reconf_callback(trajectory_reshaping_hand_mapping::HandMappingConfig &config, uint32_t level)
    {
        /*ROS_INFO("Reconfigure Request: %f %f %f",
                config.double_sat_x, config.double_sat_y,
                config.double_sat_z);*/
        PAR_BUFFERED_SETPOINT_SAT_X=config.double_sat_x;
        PAR_BUFFERED_SETPOINT_SAT_Y=config.double_sat_y;
        PAR_BUFFERED_SETPOINT_SAT_Z=config.double_sat_z;
        PAR_BUFFERED_SETPOINT_SAT_YAW=config.double_sat_yaw;
    }


    void run()
    {
        ros::Rate loop_rate(PAR_BUFFERED_PUBLISH_RATE);

        double roll=0.0,pitch=0.0,yaw=0.0;

        while(ros::ok())
        {
            msg_posestmpd_drone_setpoint_buffered.header.stamp=ros::Time::now();

            // Saturate positions
            if(msg_posestmpd_drone_setpoint_buffered.pose.position.x>PAR_BUFFERED_SETPOINT_SAT_X)
                msg_posestmpd_drone_setpoint_buffered.pose.position.x=PAR_BUFFERED_SETPOINT_SAT_X;
            if(msg_posestmpd_drone_setpoint_buffered.pose.position.x<-PAR_BUFFERED_SETPOINT_SAT_X)
                msg_posestmpd_drone_setpoint_buffered.pose.position.x=-PAR_BUFFERED_SETPOINT_SAT_X;
            if(msg_posestmpd_drone_setpoint_buffered.pose.position.y>PAR_BUFFERED_SETPOINT_SAT_Y)
                msg_posestmpd_drone_setpoint_buffered.pose.position.y=PAR_BUFFERED_SETPOINT_SAT_Y;
            if(msg_posestmpd_drone_setpoint_buffered.pose.position.y<-PAR_BUFFERED_SETPOINT_SAT_Y)
                msg_posestmpd_drone_setpoint_buffered.pose.position.y=-PAR_BUFFERED_SETPOINT_SAT_Y;
            if(msg_posestmpd_drone_setpoint_buffered.pose.position.z>PAR_BUFFERED_SETPOINT_SAT_Z)
                msg_posestmpd_drone_setpoint_buffered.pose.position.z=PAR_BUFFERED_SETPOINT_SAT_Z;
            if(msg_posestmpd_drone_setpoint_buffered.pose.position.z<-PAR_BUFFERED_SETPOINT_SAT_Z)
                msg_posestmpd_drone_setpoint_buffered.pose.position.z=PAR_BUFFERED_SETPOINT_SAT_Z;

            // Saturate yawing
            tf::Quaternion q_yaw_drone;
            q_yaw_drone.setW(msg_posestmpd_drone_setpoint_buffered.pose.orientation.w);
            q_yaw_drone.setX(msg_posestmpd_drone_setpoint_buffered.pose.orientation.x);
            q_yaw_drone.setY(msg_posestmpd_drone_setpoint_buffered.pose.orientation.y);
            q_yaw_drone.setZ(msg_posestmpd_drone_setpoint_buffered.pose.orientation.z);
            tf::Matrix3x3 m_q_yaw_drone(q_yaw_drone);
            m_q_yaw_drone.getRPY(roll,pitch,yaw);
            if(PAR_BUFFERED_SETPOINT_SAT_YAW!=0.0)
            {
                if(yaw<-PAR_BUFFERED_SETPOINT_SAT_YAW*M_PI/180.0) yaw=-PAR_BUFFERED_SETPOINT_SAT_YAW*M_PI/180.0;
                if(yaw>PAR_BUFFERED_SETPOINT_SAT_YAW*M_PI/180.0) yaw=PAR_BUFFERED_SETPOINT_SAT_YAW*M_PI/180.0;
            }
            m_q_yaw_drone.setRPY(0.0,0.0,yaw); // Overwrite ROLL and PITCH of setpoint with 0!
            m_q_yaw_drone.getRotation(q_yaw_drone);
            msg_posestmpd_drone_setpoint_buffered.pose.orientation.w=q_yaw_drone.getW();
            msg_posestmpd_drone_setpoint_buffered.pose.orientation.x=q_yaw_drone.getX();
            msg_posestmpd_drone_setpoint_buffered.pose.orientation.y=q_yaw_drone.getY();
            msg_posestmpd_drone_setpoint_buffered.pose.orientation.z=q_yaw_drone.getZ();

            pub_buffered_setpoint.publish(msg_posestmpd_drone_setpoint_buffered);
            ros::spinOnce();
            //ROS_INFO("PUBLISHED latest setpoint...");
            loop_rate.sleep();
        }
    }
};



int main(int argc, char **argv)
{
    ros::init(argc, argv, "trajectory_reshaping_hand_mapping_node");
    HandMapping setpointbuff;
    ROS_INFO("Initialized...");
    setpointbuff.run();
    ROS_INFO("Exiting...");
    return 0;
}