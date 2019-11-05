/*
 * Copyright (c) 2010, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

// %Tag(FULLTEXT)%
// %Tag(INCLUDES)%
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

// %EndTag(INCLUDES)%

// %Tag(INIT)%
int main(int argc, char** argv) {
  ros::init(argc, argv, "basic_shapes");
  ros::NodeHandle n;
  ros::Rate r(100);
  ros::Publisher marker_pub =
      n.advertise<visualization_msgs::Marker>("test_visualization_marker", 1);
  ros::Publisher camera_pose_pub =
      n.advertise<geometry_msgs::PoseStamped>("test_camera_pose", 1);
  // %EndTag(INIT)%

  // Set our initial shape type to be a cube
  // %Tag(SHAPE_INIT)%
  uint32_t shape = visualization_msgs::Marker::CUBE;
  // %EndTag(SHAPE_INIT)%

  // %Tag(MARKER_INIT)%

  double angle = 0;
  double radius = 2;

  while (ros::ok()) {
    geometry_msgs::PoseStamped camera_pose;
    camera_pose.header.frame_id = "/nav";
    camera_pose.header.stamp = ros::Time::now();

    angle += 0.01;  // or some other value.  Higher numbers = circles faster

    double x = cos(angle) * radius;
    double y = sin(angle) * radius;

    camera_pose.pose.position.x = x;
    camera_pose.pose.position.y = y;
    camera_pose.pose.position.z = 0.5;

    camera_pose.pose.orientation.x = 0;
    camera_pose.pose.orientation.y = 0;
    camera_pose.pose.orientation.z = 0;
    camera_pose.pose.orientation.w = 1;

    camera_pose_pub.publish(camera_pose);

    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on
    // these.
    marker.header.frame_id = "/world";
    marker.header.stamp = ros::Time::now();
    // %EndTag(MARKER_INIT)%

    // Set the namespace and id for this marker.  This serves to create a unique
    // ID
    // Any marker sent with the same namespace and id will overwrite the old one
    // %Tag(NS_ID)%
    marker.ns = "basic_shapes";
    marker.id = 0;
    // %EndTag(NS_ID)%

    // Set the marker type.  Initially this is CUBE, and cycles between that and
    // SPHERE, ARROW, and CYLINDER
    // %Tag(TYPE)%
    marker.type = shape;
    // %EndTag(TYPE)%

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3
    // (DELETEALL)
    // %Tag(ACTION)%
    marker.action = visualization_msgs::Marker::ADD;
    // %EndTag(ACTION)%

    // Set the pose of the marker.  This is a full 6DOF pose relative to the
    // frame/time specified in the header
    // %Tag(POSE)%
    marker.pose.position.x = 5;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0.5;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    // %EndTag(POSE)%

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    // %Tag(SCALE)%
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;
    // %EndTag(SCALE)%

    // Set the color -- be sure to set alpha to something non-zero!
    // %Tag(COLOR)%
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;
    // %EndTag(COLOR)%

    // %Tag(LIFETIME)%
    marker.lifetime = ros::Duration();
    // %EndTag(LIFETIME)%

    // Publish the marker
    // %Tag(PUBLISH)%
    /*while (marker_pub.getNumSubscribers() < 1) {
      if (!ros::ok()) {
        return 0;
      }
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }*/
    marker_pub.publish(marker);
    // %EndTag(PUBLISH)%

    // Cycle between different shapes
    // %Tag(CYCLE_SHAPES)%
    if ((int)(angle * 100) % 100 == 0) {
      switch (shape) {
        case visualization_msgs::Marker::CUBE:
          shape = visualization_msgs::Marker::SPHERE;
          break;
        case visualization_msgs::Marker::SPHERE:
          shape = visualization_msgs::Marker::ARROW;
          break;
        case visualization_msgs::Marker::ARROW:
          shape = visualization_msgs::Marker::CYLINDER;
          break;
        case visualization_msgs::Marker::CYLINDER:
          shape = visualization_msgs::Marker::CUBE;
          break;
      }
    }
    // %EndTag(CYCLE_SHAPES)%

    // %Tag(SLEEP_END)%
    r.sleep();
  }
  // %EndTag(SLEEP_END)%
}
// %EndTag(FULLTEXT)%
