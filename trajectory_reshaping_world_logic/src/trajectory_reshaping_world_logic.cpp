
#include "trajectory_reshaping_world_logic/trajectory_reshaping_world_logic.h"

WorldLogic::WorldLogic(ros::NodeHandle &nh) : nodeHandle(nh)
{
    sub_pose_bebop2camera = nh.subscribe("/reshaping/bebop2camera/odometry_sensor1/pose", 1, &WorldLogic::callback_sub_pose_bebop2camera, this);
    gazebo_set_model_state_client = nodeHandle.serviceClient<gazebo_msgs::SetModelState>("/reshaping/gazebo/set_model_state");
    collapse_successfull=false;
    tree_collapsed=false;
}

void WorldLogic::run()
{
    while (ros::ok())
    {
        ros::spin();
    }
}

void WorldLogic::callback_sub_pose_bebop2camera(const geometry_msgs::Pose::ConstPtr& msg)
{
    msg_pose_bebop2camera = (*msg);
    if(tree_collapsed==false && msg_pose_bebop2camera.position.x>=10.0 && msg_pose_bebop2camera.position.y<=-10.0 )
    {
        if(CollapseTree()==1)
        {
            //ros::shutdown();
            tree_collapsed=true;
            time_tree_collapsed=ros::Time::now();
        }
    }
    if(tree_collapsed && (ros::Time::now()-time_tree_collapsed).toSec()>2.0)
    {
        gazebo_msgs::SetModelState set_model_state;
        gazebo_msgs::ModelState model_state;
        model_state.model_name="tree";
        model_state.reference_frame="world";
        model_state.pose.position.x=27.4996971795;
        model_state.pose.position.y=-4.00702873676;
        model_state.pose.position.z=0.00999999998038;
        model_state.pose.orientation.w=0.70868136425;
        model_state.pose.orientation.x=0.705528684012;
        model_state.pose.orientation.y=0.0;
        model_state.pose.orientation.z=0.0;
        model_state.twist.linear.x=0.0;
        model_state.twist.linear.y=0.0;
        model_state.twist.linear.z=0.0;
        model_state.twist.angular.x=0.0;
        model_state.twist.angular.y=0.0;
        model_state.twist.angular.z=0.0;
        set_model_state.request.model_state = model_state;
        gazebo_set_model_state_client.call(set_model_state);

        time_tree_collapsed=ros::Time::now();
        //ROS_WARN("Recollapsing!");
    }

}

int WorldLogic::CollapseTree()
{
    gazebo_msgs::SetModelState set_model_state;
    gazebo_msgs::ModelState model_state;
    model_state.model_name="tree";
    model_state.reference_frame="world";
    model_state.pose.position.x=27.5;
    model_state.pose.position.y=-4.0;
    model_state.pose.position.z=0.1;
    model_state.pose.orientation.w=1.0;
    model_state.pose.orientation.x=0.0;
    model_state.pose.orientation.y=0.0;
    model_state.pose.orientation.z=0.0;
    model_state.twist.linear.x=0.0;
    model_state.twist.linear.y=0.0;
    model_state.twist.linear.z=0.0;
    model_state.twist.angular.x=2.0; // 1.75
    model_state.twist.angular.y=0.0;
    model_state.twist.angular.z=0.0;
    set_model_state.request.model_state = model_state;

    if (gazebo_set_model_state_client.call(set_model_state))
    {
        ROS_WARN("Successfully collapsed tree!");
    }
    else
    {
        ROS_ERROR("Failed to call service!");
        return 0;
    }

    return 1;
}
