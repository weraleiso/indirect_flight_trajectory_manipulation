#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include "spline.h"

double sample_time = 0.0;
double rise_time = 0.0;
double epsilon = 0.0;

bool first_iteration=true;
bool trajectory_arrived=false;
double trajectory_length;

double x_ = -2.0;
double y_ = 0.0;
double z_ = 1.0;
double yaw_ = 0.0;

double q_w_ = 1.0;
double q_x_ = 0.0;
double q_y_ = 0.0;
double q_z_ = 0.0;

ros::Subscriber sub_posestmpd_setpoint;
ros::Subscriber sub_posestmpd_bebop2actor_location;
ros::Subscriber sub_posearray_planner_path_points;
ros::Publisher pub_posestmpd_setpoint_filtered;
ros::Publisher pub_posestmpd_setpoint_actor;
ros::Publisher pub_multidof_traj_bebop2actor;
ros::Publisher pub_posearray_trajectory_interpolated_real_world;
ros::Publisher pub_path_bebop2actor;
ros::Publisher pub_planner_path_points;

nav_msgs::Path msg_path_bebop2actor;
int counter_trajectory_points,counter_trajectory_points_max;
geometry_msgs::Pose msg_posestmpd_bebop2actor_location;

void callback_sub_posestmpd_setpoint(const geometry_msgs::PoseStamped&);
void callback_sub_posestmpd_bebop2actor_location(const geometry_msgs::PoseConstPtr&);
void callback_sub_posearray_planner_path_points(const geometry_msgs::PoseArrayConstPtr&);



int main(int argc, char **argv)
{
    ros::init(argc, argv, "trajectory_reshaping_trajectory_generation_node");
	ros::NodeHandle n;

    n.getParam("trajectory_reshaping_trajectory_generation_node/sys_params/sample_time", sample_time);
    n.getParam("trajectory_reshaping_trajectory_generation_node/sys_params/rise_time", rise_time);
    n.getParam("trajectory_reshaping_trajectory_generation_node/sys_params/epsilon", epsilon);
    n.getParam("trajectory_reshaping_trajectory_generation_node/sys_params/initial_setpoint_x", x_);
    n.getParam("trajectory_reshaping_trajectory_generation_node/sys_params/initial_setpoint_y", y_);
    n.getParam("trajectory_reshaping_trajectory_generation_node/sys_params/initial_setpoint_z", z_);
    n.getParam("trajectory_reshaping_trajectory_generation_node/sys_params/initial_setpoint_yaw", yaw_);

    trajectory_length=0.0;

    sub_posestmpd_setpoint = n.subscribe("/reshaping/bebop2camera/command/pose_to_lowpass", 90, &callback_sub_posestmpd_setpoint);
    sub_posestmpd_bebop2actor_location = n.subscribe("/reshaping/bebop2actor/ground_truth/pose", 1, &callback_sub_posestmpd_bebop2actor_location);
    pub_posestmpd_setpoint_actor = n.advertise<geometry_msgs::PoseStamped>("/reshaping/bebop2actor/command/pose",1);
    pub_posestmpd_setpoint_filtered = n.advertise<geometry_msgs::PoseStamped>("/reshaping/bebop2camera/command/pose",1);
    pub_path_bebop2actor = n.advertise<nav_msgs::Path>("/reshaping/bebop2camera/path",1);
    pub_planner_path_points = n.advertise<geometry_msgs::PoseArray>("/planner/global_path_points/interpolated_original",1);


    sub_posearray_planner_path_points = n.subscribe("/planner/global_path_points", 1, &callback_sub_posearray_planner_path_points);
    pub_multidof_traj_bebop2actor = n.advertise<trajectory_msgs::MultiDOFJointTrajectory>("/reshaping/bebop2actor/command/trajectory",1);
    pub_posearray_trajectory_interpolated_real_world = n.advertise<geometry_msgs::PoseArray>("/reshaping/bebop2actor/command/trajectory_interpolated_real_world",1);

    while(ros::ok())
    {
        ros::spin();
	}

  return 0;
}

double lowPassFilter(double x, double y0, double dt, double T)          // Taken from http://en.wikipedia.org/wiki/Low-pass_filter
{
   double res = y0 + (x - y0) * (dt/(dt+T));

   //if ((res*res) <= epsilon) res = 0;
   //ROS_INFO("RES: %3.3f",res);
   return res;
}

void callback_sub_posestmpd_bebop2actor_location(const geometry_msgs::PoseConstPtr& msg)
{
    msg_posestmpd_bebop2actor_location=(*msg);
}

void callback_sub_posearray_planner_path_points(const geometry_msgs::PoseArrayConstPtr& msg)
{
    msg_path_bebop2actor.header.stamp=ros::Time::now();
    msg_path_bebop2actor.header.frame_id="world";

    geometry_msgs::PoseArray array_pose_planner_path_points=(*msg);

    if(array_pose_planner_path_points.poses.size()==1) // Fix issue for interpolation which requires 2 points minimum!
    {
        if(     array_pose_planner_path_points.poses[0].orientation.w==-1.0 &&
                array_pose_planner_path_points.poses[0].orientation.x==-1.0 &&
                array_pose_planner_path_points.poses[0].orientation.y==-1.0 &&
                array_pose_planner_path_points.poses[0].orientation.z==-1.0)
        {
            ROS_INFO("Received infeasable path-response from PRM planner!");
            return;
        }
        geometry_msgs::PoseStamped pose_single_path_point;
        pose_single_path_point.header.stamp=ros::Time::now();
        pose_single_path_point.header.frame_id="world";
        pose_single_path_point.pose=array_pose_planner_path_points.poses[0];

        tf::Vector3 vec_default(1.0,0.0,0.0);
        tf::Vector3 vec_to_next_sp;
        vec_to_next_sp.setX(pose_single_path_point.pose.position.x-msg_posestmpd_bebop2actor_location.position.x);
        vec_to_next_sp.setY(pose_single_path_point.pose.position.y-msg_posestmpd_bebop2actor_location.position.y);
        vec_to_next_sp.setZ(pose_single_path_point.pose.position.z-msg_posestmpd_bebop2actor_location.position.z);

        tf::Quaternion quat_to_next_sp;
        tf::Vector3 vec_rotated = vec_default.cross(vec_to_next_sp);
        quat_to_next_sp.setX(vec_rotated.getX());
        quat_to_next_sp.setY(vec_rotated.getY());
        quat_to_next_sp.setZ(vec_rotated.getZ());
        quat_to_next_sp.setW(sqrt((vec_default.length()*vec_default.length()) * (vec_to_next_sp.length()*vec_to_next_sp.length())) + vec_default.dot(vec_to_next_sp));
        quat_to_next_sp=quat_to_next_sp.normalize();

        pose_single_path_point.pose.orientation.w=quat_to_next_sp.getW();
        pose_single_path_point.pose.orientation.x=quat_to_next_sp.getX();
        pose_single_path_point.pose.orientation.y=quat_to_next_sp.getY();
        pose_single_path_point.pose.orientation.z=quat_to_next_sp.getZ();

        pub_posestmpd_setpoint_actor.publish(pose_single_path_point);
        return;
    }

    if(array_pose_planner_path_points.poses.size()>1) // Fix issue with "turning on the spot" for first path point!
    {
        geometry_msgs::PoseStamped pose_single_path_point;
        pose_single_path_point.header.stamp=ros::Time::now();
        pose_single_path_point.header.frame_id="world";

        tf::Vector3 vec_to_next_sp;
        tf::Vector3 vec_default(1.0,0.0,0.0);
        int turn_on_spot_index_counter=0;
        do
        {
            pose_single_path_point.pose=array_pose_planner_path_points.poses[turn_on_spot_index_counter]; // Check 2nd path point!
            vec_to_next_sp.setX(pose_single_path_point.pose.position.x-msg_posestmpd_bebop2actor_location.position.x);
            vec_to_next_sp.setY(pose_single_path_point.pose.position.y-msg_posestmpd_bebop2actor_location.position.y);
            vec_to_next_sp.setZ(pose_single_path_point.pose.position.z-msg_posestmpd_bebop2actor_location.position.z);
            turn_on_spot_index_counter++;
        }while(fabs(vec_to_next_sp.length())<2.0);


        tf::Quaternion quat_to_next_sp;
        tf::Vector3 vec_rotated = vec_default.cross(vec_to_next_sp);
        quat_to_next_sp.setX(vec_rotated.getX());
        quat_to_next_sp.setY(vec_rotated.getY());
        quat_to_next_sp.setZ(vec_rotated.getZ());
        quat_to_next_sp.setW(sqrt((vec_default.length()*vec_default.length()) * (vec_to_next_sp.length()*vec_to_next_sp.length())) + vec_default.dot(vec_to_next_sp));
        quat_to_next_sp=quat_to_next_sp.normalize();

        tf::Matrix3x3 mat_bebop2actor_heading(tf::Quaternion(msg_posestmpd_bebop2actor_location.orientation.x,msg_posestmpd_bebop2actor_location.orientation.y,msg_posestmpd_bebop2actor_location.orientation.z,msg_posestmpd_bebop2actor_location.orientation.w));
        double roll_bebop2actor_heading=0.0,pitch_bebop2actor_heading=0.0,yaw_bebop2actor_heading=0.0;
        mat_bebop2actor_heading.getRPY(roll_bebop2actor_heading,pitch_bebop2actor_heading,yaw_bebop2actor_heading);
        tf::Matrix3x3 mat_to_next_sp(quat_to_next_sp);
        double roll_to_next_sp=0.0,pitch_to_next_sp=0.0,yaw_to_next_sp=0.0;
        mat_to_next_sp.getRPY(roll_to_next_sp,pitch_to_next_sp,yaw_to_next_sp);


        geometry_msgs::PoseArray poses_turn_on_spot;
        poses_turn_on_spot.header.stamp=ros::Time::now();
        poses_turn_on_spot.header.frame_id="world";

        ROS_WARN("yaw to next sp %3.3f, heading %3.3f, yaw diff %3.3f",yaw_to_next_sp*180.0/M_PI,yaw_bebop2actor_heading*180.0/M_PI,(yaw_to_next_sp-yaw_bebop2actor_heading)*180.0/M_PI);
        if( (yaw_to_next_sp-yaw_bebop2actor_heading) >= 75.0*M_PI/180.0 )//&& (yaw_to_next_sp-yaw_bebop2actor_heading) <= 170.0*M_PI/180.0)
        {
            for(int k=0;k<turn_on_spot_index_counter;k++)
            {
                array_pose_planner_path_points.poses.erase(array_pose_planner_path_points.poses.begin());
            }

            geometry_msgs::Pose pose_inserted_point;
            for(double c=(yaw_to_next_sp-M_PI);c<(yaw_to_next_sp+yaw_bebop2actor_heading)*0.5;c=c+17.0*M_PI/180.0)
            {
                pose_inserted_point.position.x=msg_posestmpd_bebop2actor_location.position.x+vec_to_next_sp.getX()/2.0;
                pose_inserted_point.position.y=msg_posestmpd_bebop2actor_location.position.y+vec_to_next_sp.getY()/2.0;
                pose_inserted_point.position.z=msg_posestmpd_bebop2actor_location.position.z+vec_to_next_sp.getZ()/2.0;
                pose_inserted_point.position.x+=fabs(vec_to_next_sp.length()/2.0)*cos(c);
                pose_inserted_point.position.y+=fabs(vec_to_next_sp.length()/2.0)*sin(c);
                //pose_inserted_point.position.z+=msg_posestmpd_bebop2actor_location.position.z+vec_to_next_sp.getZ()/2.0;
                pose_inserted_point.orientation.w=1.0;
                pose_inserted_point.orientation.x=0.0;
                pose_inserted_point.orientation.y=0.0;
                pose_inserted_point.orientation.z=0.0;
                poses_turn_on_spot.poses.push_back(pose_inserted_point);
            }
            pose_inserted_point.position.x=(poses_turn_on_spot.poses[poses_turn_on_spot.poses.size()-1].position.x+array_pose_planner_path_points.poses[0].position.x)/2.0;
            pose_inserted_point.position.y=(poses_turn_on_spot.poses[poses_turn_on_spot.poses.size()-1].position.y+array_pose_planner_path_points.poses[0].position.y)/2.0;
            pose_inserted_point.position.z=(poses_turn_on_spot.poses[poses_turn_on_spot.poses.size()-1].position.z+array_pose_planner_path_points.poses[0].position.z)/2.0;
            pose_inserted_point.orientation.w=1.0;
            pose_inserted_point.orientation.x=0.0;
            pose_inserted_point.orientation.y=0.0;
            pose_inserted_point.orientation.z=0.0;
            poses_turn_on_spot.poses.push_back(pose_inserted_point);

            //Rearrange and merge with planner points
            for(int i=poses_turn_on_spot.poses.size()-1;i>=0;i--)
            {
                array_pose_planner_path_points.poses.insert(array_pose_planner_path_points.poses.begin(),poses_turn_on_spot.poses[i]);
            }
        }
        else if( (yaw_to_next_sp-yaw_bebop2actor_heading) <= -75.0*M_PI/180.0 )//&& (yaw_to_next_sp-yaw_bebop2actor_heading) >= -170.0*M_PI/180.0)
        {
            for(int k=0;k<turn_on_spot_index_counter;k++)
            {
                array_pose_planner_path_points.poses.erase(array_pose_planner_path_points.poses.begin());
            }
            geometry_msgs::Pose pose_inserted_point;
            for(double c=(yaw_to_next_sp)+M_PI;c>(yaw_to_next_sp+yaw_bebop2actor_heading)*0.5;c=c-17.0*M_PI/180.0)
            {
                pose_inserted_point.position.x=msg_posestmpd_bebop2actor_location.position.x+vec_to_next_sp.getX()/2.0;
                pose_inserted_point.position.y=msg_posestmpd_bebop2actor_location.position.y+vec_to_next_sp.getY()/2.0;
                pose_inserted_point.position.z=msg_posestmpd_bebop2actor_location.position.z+vec_to_next_sp.getZ()/2.0;
                pose_inserted_point.position.x+=fabs(vec_to_next_sp.length()/2.0)*cos(c);
                pose_inserted_point.position.y+=fabs(vec_to_next_sp.length()/2.0)*sin(c);
                //pose_inserted_point.position.z+=msg_posestmpd_bebop2actor_location.position.z+vec_to_next_sp.getZ()/2.0;
                pose_inserted_point.orientation.w=1.0;
                pose_inserted_point.orientation.x=0.0;
                pose_inserted_point.orientation.y=0.0;
                pose_inserted_point.orientation.z=0.0;
                poses_turn_on_spot.poses.push_back(pose_inserted_point);
            }
            pose_inserted_point.position.x=(poses_turn_on_spot.poses[poses_turn_on_spot.poses.size()-1].position.x+array_pose_planner_path_points.poses[0].position.x)/2.0;
            pose_inserted_point.position.y=(poses_turn_on_spot.poses[poses_turn_on_spot.poses.size()-1].position.y+array_pose_planner_path_points.poses[0].position.y)/2.0;
            pose_inserted_point.position.z=(poses_turn_on_spot.poses[poses_turn_on_spot.poses.size()-1].position.z+array_pose_planner_path_points.poses[0].position.z)/2.0;
            pose_inserted_point.orientation.w=1.0;
            pose_inserted_point.orientation.x=0.0;
            pose_inserted_point.orientation.y=0.0;
            pose_inserted_point.orientation.z=0.0;
            poses_turn_on_spot.poses.push_back(pose_inserted_point);

            //Rearrange and merge with planner points
            for(int i=poses_turn_on_spot.poses.size()-1;i>=0;i--)
            {
                array_pose_planner_path_points.poses.insert(array_pose_planner_path_points.poses.begin(),poses_turn_on_spot.poses[i]);
            }
        }

    }


    // CALCULATION FOR INTERPOLATING SPARSE GLOBAL PATH POINTS
    double desired_velocity=0.3; // For experiments (user study)
    double time_from_start_accumulated=0.0;

    trajectory_msgs::MultiDOFJointTrajectory trajectory;
    geometry_msgs::PoseArray msg_planner_path_points_interpolated;
    msg_planner_path_points_interpolated.header.frame_id="world";
    trajectory.header.stamp=ros::Time::now();
    trajectory.header.frame_id="world";
    trajectory.joint_names.push_back("base_link");

    //double counter_interpolation_points=1000.0;
    std::vector<double> basis_interpolation;
    std::vector<double> x_interpolation;
    std::vector<double> y_interpolation;
    std::vector<double> z_interpolation;
    basis_interpolation.empty();
    x_interpolation.empty();
    y_interpolation.empty();
    z_interpolation.empty();

    for(int n=0;n<array_pose_planner_path_points.poses.size();n++)
    {
        //basis_interpolation.push_back(double(n)*1.0/double(array_pose_planner_path_points.poses.size()-1));
        basis_interpolation.push_back(double(n));
        x_interpolation.push_back(array_pose_planner_path_points.poses[n].position.x);
        y_interpolation.push_back(array_pose_planner_path_points.poses[n].position.y);
        z_interpolation.push_back(array_pose_planner_path_points.poses[n].position.z);
    }
    /*basis_interpolation.push_back(double(n)*1.0/double(msg_planner_path_points.poses.size()));
    x_interpolation.push_back(msg_planner_path_points.poses[n-1].position.x);
    y_interpolation.push_back(msg_planner_path_points.poses[n-1].position.y);
    z_interpolation.push_back(msg_planner_path_points.poses[n-1].position.z);*/

    tk::spline s_x, s_y, s_z;
    s_x.set_points(basis_interpolation, x_interpolation);
    s_y.set_points(basis_interpolation, y_interpolation);
    s_z.set_points(basis_interpolation, z_interpolation);

    geometry_msgs::PoseArray array_pose_planner_path_points_interpolated;
    //for(double k=0;k<1.0;k=k+1.0/counter_interpolation_points)
    for(double k=0;k<array_pose_planner_path_points.poses.size()-1;k=k+1.0/(10.0))
    {
        geometry_msgs::Pose pose_tmp;
        pose_tmp.position.x=s_x(k);
        pose_tmp.position.y=s_y(k);
        pose_tmp.position.z=s_z(k);
        array_pose_planner_path_points_interpolated.poses.push_back(pose_tmp);
    }
    ROS_WARN("Number of poses after interpolation: %d Sending trajectory...",int(array_pose_planner_path_points_interpolated.poses.size()));

    geometry_msgs::PoseArray msg_planner_path_points; // Publishing raw unchanged interpolated trajectory


    for(int i=0;i<array_pose_planner_path_points_interpolated.poses.size();i++)
    {
        trajectory_msgs::MultiDOFJointTrajectoryPoint trajectory_point;

        geometry_msgs::Transform tp_tf;
        tp_tf.translation.x=array_pose_planner_path_points_interpolated.poses[i].position.x;
        tp_tf.translation.y=array_pose_planner_path_points_interpolated.poses[i].position.y;
        tp_tf.translation.z=array_pose_planner_path_points_interpolated.poses[i].position.z;

        tf::Vector3 vec_default(1.0,0.0,0.0);
        tf::Vector3 vec_to_next_sp;
        if(i<(array_pose_planner_path_points_interpolated.poses.size()-1))
        {
            vec_to_next_sp.setX(array_pose_planner_path_points_interpolated.poses[i+1].position.x-array_pose_planner_path_points_interpolated.poses[i].position.x);
            vec_to_next_sp.setY(array_pose_planner_path_points_interpolated.poses[i+1].position.y-array_pose_planner_path_points_interpolated.poses[i].position.y);
            vec_to_next_sp.setZ(array_pose_planner_path_points_interpolated.poses[i+1].position.z-array_pose_planner_path_points_interpolated.poses[i].position.z);
        }
        else
        {
            vec_to_next_sp.setX(array_pose_planner_path_points_interpolated.poses[i].position.x-array_pose_planner_path_points_interpolated.poses[i-1].position.x);
            vec_to_next_sp.setY(array_pose_planner_path_points_interpolated.poses[i].position.y-array_pose_planner_path_points_interpolated.poses[i-1].position.y);
            vec_to_next_sp.setZ(array_pose_planner_path_points_interpolated.poses[i].position.z-array_pose_planner_path_points_interpolated.poses[i-1].position.z);
        }
        //trajectory_length=0.0;
        //trajectory_length+=vec_to_next_sp.length();
        tf::Quaternion quat_to_next_sp;
        tf::Vector3 vec_rotated = vec_default.cross(vec_to_next_sp);
        quat_to_next_sp.setX(vec_rotated.getX());
        quat_to_next_sp.setY(vec_rotated.getY());
        quat_to_next_sp.setZ(vec_rotated.getZ());
        quat_to_next_sp.setW(sqrt((vec_default.length()*vec_default.length()) * (vec_to_next_sp.length()*vec_to_next_sp.length())) + vec_default.dot(vec_to_next_sp));
        quat_to_next_sp=quat_to_next_sp.normalize();

        array_pose_planner_path_points_interpolated.poses[i].orientation.w=quat_to_next_sp.getW();
        array_pose_planner_path_points_interpolated.poses[i].orientation.x=quat_to_next_sp.getX();
        array_pose_planner_path_points_interpolated.poses[i].orientation.y=quat_to_next_sp.getY();
        array_pose_planner_path_points_interpolated.poses[i].orientation.z=quat_to_next_sp.getZ();


        // Complement data for trajectory message
        tp_tf.rotation.w=array_pose_planner_path_points_interpolated.poses[i].orientation.w;
        tp_tf.rotation.x=array_pose_planner_path_points_interpolated.poses[i].orientation.x;
        tp_tf.rotation.y=array_pose_planner_path_points_interpolated.poses[i].orientation.y;
        tp_tf.rotation.z=array_pose_planner_path_points_interpolated.poses[i].orientation.z;
        trajectory_point.transforms.push_back(tp_tf);

        time_from_start_accumulated+=vec_to_next_sp.length()/desired_velocity;
        trajectory_point.time_from_start=ros::Duration(time_from_start_accumulated);
        // Generate velocity profile
        tf::Vector3 tf_vec_to_next_sp_speed;
        tf_vec_to_next_sp_speed=vec_to_next_sp.normalized()*desired_velocity;

        if(i==(array_pose_planner_path_points_interpolated.poses.size()-1))tf_vec_to_next_sp_speed=vec_to_next_sp.normalized()*0.0;

        geometry_msgs::Twist pw_twist;
        pw_twist.linear.x=tf_vec_to_next_sp_speed.getX();
        pw_twist.linear.y=tf_vec_to_next_sp_speed.getY();
        pw_twist.linear.z=tf_vec_to_next_sp_speed.getZ();
        pw_twist.angular.x=0.0;
        pw_twist.angular.y=0.0;
        pw_twist.angular.z=0.0;
        trajectory_point.velocities.push_back(pw_twist);
        // Generate accaleration profile
        tf::Vector3 tf_vec_to_next_sp_acc;
        double acc_at_next_point=tf_vec_to_next_sp_speed.length()/trajectory_point.time_from_start.toSec();// time_between_points;
        tf_vec_to_next_sp_acc=tf_vec_to_next_sp_speed.normalize()*acc_at_next_point;
        geometry_msgs::Twist pw_accaleration;
        pw_accaleration.linear.x=0.0;
        pw_accaleration.linear.y=0.0;
        pw_accaleration.linear.z=0.0;
        pw_accaleration.angular.x=0.0;
        pw_accaleration.angular.y=0.0;
        pw_accaleration.angular.z=0.0;
        trajectory_point.accelerations.push_back(pw_accaleration);
        trajectory.points.push_back(trajectory_point);

        // Add publishing as Path
        geometry_msgs::PoseStamped posestmpd_temp;
        posestmpd_temp.header.stamp=ros::Time::now();
        posestmpd_temp.header.frame_id="world";
        posestmpd_temp.pose.position.x=array_pose_planner_path_points_interpolated.poses[i].position.x;
        posestmpd_temp.pose.position.y=array_pose_planner_path_points_interpolated.poses[i].position.y;
        posestmpd_temp.pose.position.z=array_pose_planner_path_points_interpolated.poses[i].position.z;
        posestmpd_temp.pose.orientation.w=array_pose_planner_path_points_interpolated.poses[i].orientation.w;
        posestmpd_temp.pose.orientation.x=array_pose_planner_path_points_interpolated.poses[i].orientation.x;
        posestmpd_temp.pose.orientation.y=array_pose_planner_path_points_interpolated.poses[i].orientation.y;
        posestmpd_temp.pose.orientation.z=array_pose_planner_path_points_interpolated.poses[i].orientation.z;
        msg_path_bebop2actor.poses.push_back(posestmpd_temp);
        msg_planner_path_points_interpolated.poses.push_back(posestmpd_temp.pose); // For sampling simple velocity profile in real-world

        geometry_msgs::Pose pose_temp;
        pose_temp=posestmpd_temp.pose;
        msg_planner_path_points.poses.push_back(pose_temp); // For publishing original interpolated trajectory points
    }
    //msg_planner_path_points.poses.clear();
    //msg_planner_path_points.poses.swap(array_pose_planner_path_points_interpolated.poses);
    //trajectory_arrived=true;

    pub_multidof_traj_bebop2actor.publish(trajectory); // Disabled for now
    msg_planner_path_points_interpolated.header.stamp=ros::Time::now();
    pub_posearray_trajectory_interpolated_real_world.publish(msg_planner_path_points_interpolated); // Publish simple velocity profile in real-world
    pub_planner_path_points.publish(msg_planner_path_points);
    pub_path_bebop2actor.publish(msg_path_bebop2actor);
    msg_path_bebop2actor.poses.clear(); // Remove old path from visualization
}

void callback_sub_posestmpd_setpoint(const geometry_msgs::PoseStamped& PoseSetpoint)
{
    double x = PoseSetpoint.pose.position.x;
    double y = PoseSetpoint.pose.position.y;
    double z = PoseSetpoint.pose.position.z;

    double q_w=PoseSetpoint.pose.orientation.w;
    double q_x=PoseSetpoint.pose.orientation.x;
    double q_y=PoseSetpoint.pose.orientation.y;
    double q_z=PoseSetpoint.pose.orientation.z;

    double roll=0.0, pitch=0.0, yaw=0.0;
    tf::Quaternion q_setp_rot;
    q_setp_rot.setW(PoseSetpoint.pose.orientation.w);
    q_setp_rot.setX(PoseSetpoint.pose.orientation.x);
    q_setp_rot.setY(PoseSetpoint.pose.orientation.y);
    q_setp_rot.setZ(PoseSetpoint.pose.orientation.z);
    tf::Matrix3x3 m_setp_rot(q_setp_rot);
    m_setp_rot.getRPY(roll,pitch,yaw);

    if(first_iteration)
    {
        x_=x;
        y_=y;
        z_=z;

        yaw_=yaw;

        q_w_=q_w;
        q_x_=q_x;
        q_y_=q_y;
        q_z_=q_z;

        first_iteration=false;
    }


    x_ = lowPassFilter(x, x_, sample_time, rise_time);
    y_ = lowPassFilter(y, y_, sample_time, rise_time);
    z_ = lowPassFilter(z, z_, sample_time, rise_time);

    yaw_ = lowPassFilter(yaw, yaw_, sample_time, rise_time);

    q_w_= lowPassFilter(q_w, q_w_, sample_time, rise_time);
    q_x_= lowPassFilter(q_x, q_x_, sample_time, rise_time);
    q_y_= lowPassFilter(q_y, q_y_, sample_time, rise_time);
    q_z_= lowPassFilter(q_z, q_z_, sample_time, rise_time);

    tf::Quaternion q_setp_rot_filtered;
    q_setp_rot_filtered.setRPY(0.0,0.0,yaw_);

    /*q_setp_rot_filtered.setW(q_w_); // Directly filter rotation on qutarnion
    q_setp_rot_filtered.setX(q_x_);
    q_setp_rot_filtered.setY(q_y_);
    q_setp_rot_filtered.setZ(q_z_);*/

    geometry_msgs::PoseStamped t;
    t.header.stamp=ros::Time::now();
    t.header.frame_id="world";

    // Prevent wrong setpoint poses for camera UAV during startup! Camera UAV has to slightly hover above ACTOR!
    //if(z_<1.1 || msg_posestmpd_bebop2actor_location.position.z<0.9)return;

    t.pose.position.x = x_;
    t.pose.position.y = y_;
    t.pose.position.z = z_;
    t.pose.orientation.w=q_setp_rot_filtered.getW();
    t.pose.orientation.x=q_setp_rot_filtered.getX();
    t.pose.orientation.y=q_setp_rot_filtered.getY();
    t.pose.orientation.z=q_setp_rot_filtered.getZ();

    pub_posestmpd_setpoint_filtered.publish(t);
}
