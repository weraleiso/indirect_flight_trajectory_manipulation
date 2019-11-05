#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/Joy.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <Eigen/Eigen>

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <sstream>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/fcntl.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>



class VelocityProfile
{
    ros::NodeHandle nh;

    ros::Subscriber sub_pose_bebop2actor_location;
    ros::Subscriber sub_posearray_trajectory_interpolated;
    ros::Publisher pub_posestmpd_position_reference;

    geometry_msgs::Pose msg_pose_reference_from_simulator;
    geometry_msgs::PoseArray msg_posearray_trajectory_interpolated;
    geometry_msgs::PoseStamped msg_posestmpd_position_reference;

    double PAR_SYSTEM_SAMPLE_RATE;

    bool received_new_trajectory;

public:
    VelocityProfile()
    {
        // LOAD PARAMETERS
        nh.getParam("trajectory_reshaping_velocity_profile_node/par_system/PAR_SYSTEM_SAMPLE_RATE", PAR_SYSTEM_SAMPLE_RATE);

        // SUBSCRIBERS
        sub_posearray_trajectory_interpolated = nh.subscribe("/reshaping/bebop2actor/command/trajectory_interpolated_real_world", 1, &VelocityProfile::callback_posearray_trajectory_interpolated, this);
        sub_pose_bebop2actor_location = nh.subscribe("/reshaping/bebop2actor/ground_truth/pose", 1, &VelocityProfile::sub_pose_bebop2actor_location_callback, this);

        // PUBLSIHERS
        pub_posestmpd_position_reference = nh.advertise<geometry_msgs::PoseStamped>("/setpoint/heading",1);

        received_new_trajectory=false;

        msg_posestmpd_position_reference.header.frame_id="world";
        msg_posestmpd_position_reference.header.stamp=ros::Time::now();
        msg_posestmpd_position_reference.pose.position.x=0.0;
        msg_posestmpd_position_reference.pose.position.y=0.0;
        msg_posestmpd_position_reference.pose.position.z=1.0;
        msg_posestmpd_position_reference.pose.orientation.w=1.0;
        msg_posestmpd_position_reference.pose.orientation.x=0.0;
        msg_posestmpd_position_reference.pose.orientation.y=0.0;
        msg_posestmpd_position_reference.pose.orientation.z=0.0;
    }

    ~VelocityProfile()
    {
    }

    void sub_pose_bebop2actor_location_callback(const geometry_msgs::Pose::ConstPtr& msg)
    {
        msg_pose_reference_from_simulator = (*msg);
    }

    void callback_posearray_trajectory_interpolated(const geometry_msgs::PoseArrayConstPtr& msg)
    {
        //msg_posearray_trajectory_interpolated=(*msg);
        //received_new_trajectory=true;

/*
        // Pub marker at Tracker Rate
        marker_fence.header.frame_id = "map";
        marker_fence.header.stamp = ros::Time::now();
        marker_fence.ns = "fence";
        marker_fence.id = 1;
        marker_fence.type = visualization_msgs::Marker::POINTS;
        marker_fence.action = visualization_msgs::Marker::ADD;
        marker_fence.pose.position.x = 0;
        marker_fence.pose.position.y = 0;
        marker_fence.pose.position.z = 0;
        marker_fence.pose.orientation.x = 0.0;
        marker_fence.pose.orientation.y = 0.0;
        marker_fence.pose.orientation.z = 0.0;
        marker_fence.pose.orientation.w = 1.0;
        marker_fence.scale.x = 0.05;
        marker_fence.scale.y = 0.05;
        marker_fence.scale.z = 0.05;
        marker_fence.color.a = 0.9; // Don't forget to set the alpha!
        marker_fence.color.r = 1.0;
        marker_fence.color.g = 0.0;
        marker_fence.color.b = 0.0;
        for(float i=-1.0;i<=4.5;i=i+0.5)
        {
            for(float j=0.0;j<=2.0;j=j+0.5)
            {
                geometry_msgs::Point p_fence;
                p_fence.x=1.0;
                p_fence.y=i;
                p_fence.z=j;
                marker_fence.points.push_back(p_fence);
            }
        }
        for(float i=-1.0;i<=4.5;i=i+0.5)
        {
            for(float j=0.0;j<=2.0;j=j+0.5)
            {
                geometry_msgs::Point p_fence;
                p_fence.x=-1.0;
                p_fence.y=i;
                p_fence.z=j;
                marker_fence.points.push_back(p_fence);
            }
        }

        pub_marker_fence.publish(marker_fence);
        marker_fence.points.clear();*/
    }

    void run()
    {
        ros::Rate loop_rate(PAR_SYSTEM_SAMPLE_RATE);

        while(ros::ok())
        {
            /*
            if(received_new_trajectory)
            {
                for(int i=0;i<msg_posearray_trajectory_interpolated.poses.size();i++)
                {
                    msg_posestmpd_position_reference.pose=msg_posearray_trajectory_interpolated.poses[i];
                    pub_posestmpd_position_reference.publish(msg_posestmpd_position_reference);

                    ros::spinOnce();
                    loop_rate.sleep();
                }
                received_new_trajectory=false;
            }*/
            msg_posestmpd_position_reference.header.stamp=ros::Time::now();
            msg_posestmpd_position_reference.header.frame_id="world";
            msg_posestmpd_position_reference.pose=msg_pose_reference_from_simulator;
            pub_posestmpd_position_reference.publish(msg_posestmpd_position_reference);

            ros::spinOnce();
            loop_rate.sleep();
        }
    }
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "trajectory_reshaping_velocity_profile_node");
    VelocityProfile trajectory_reshaping_velocity_profile;
	ROS_INFO("Initialized. About to spin...");
    trajectory_reshaping_velocity_profile.run();
	ROS_WARN("Exiting...");
    return 0;
}
