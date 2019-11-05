#!/usr/bin/env python

import rospy
import tf
import sys
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose

import tf2_ros

"""
Publish a frame 3D pose as a PoseStamped continuously.

Author: Sammy Pfeiffer <Sammy.Pfeiffer at student.uts.edu.au>
"""


class PublishFrameAsPoseStamped(object):
    def __init__(self, frame_to_posestamped,
                 reference_frame,
                 header_frame_id,
                 controller_topic_name,
                 rate,
                 verbose=False):
        """
        Class to publish a frame as a PoseStamped.
        :param frame_to_posestamped str: frame that will be published its
                pose as PoseStamped.
        :param reference_frame str: frame that will be the header.frame_id
                of the PoseStamped.
        :param rate int: rate at which to compute and publish the pose.
        :param verbose bool: print to screen the transformations.
        """
        self.tf_l = tf.TransformListener()
        topic_name = controller_topic_name
        self.pose_pub = rospy.Publisher(topic_name,
                                        PoseStamped, queue_size=1)
        self.frame_to_posestamped = frame_to_posestamped
        self.reference_frame = reference_frame
        self.header_frame_id = header_frame_id
        self.controller_topic_name = controller_topic_name
        self.rate = rospy.Rate(rate)
        self.verbose = verbose

    def transform_pose(self, pose, from_frame, to_frame):
        """
        Transform the 'pose' from frame 'from_frame'
         to frame 'to_frame'

        :param geometry_msgs/Pose pose: 3D Pose to transform.
        :param str from_frame: frame that the pose belongs to.
        :param str to_frame: to what frame transform.
        """
        ps = PoseStamped()
        # ps.header.stamp = #self.tf_l.getLatestCommonTime(from_frame,
        # to_frame)
        ps.header.frame_id = from_frame
        ps.pose = pose
        transform_ok = False
        min_time_in_between_warns = rospy.Duration(5.0)
        #last_warn = rospy.Time.now() - min_time_in_between_warns
        while not transform_ok and not rospy.is_shutdown():
            try:
                target_ps = self.tf_l.transformPose(to_frame, ps)
                transform_ok = True
            except tf.ExtrapolationException as e:
                #if rospy.Time.now() > (last_warn + min_time_in_between_warns):
                #    rospy.logwarn(
                #        "ExtrapolationException on transforming pose... trying again \n(" +
                #        str(e) + ")")
                #    last_warn = rospy.Time.now()
                print("ExtrapolationException happened!!")
                rospy.sleep(0.2)
                ps.header.stamp = self.tf_l.getLatestCommonTime(
                    from_frame, to_frame)
            except tf.LookupException as e:
                #if rospy.Time.now() > (last_warn + min_time_in_between_warns):
                #    rospy.logwarn(
                #        "LookupException on transforming pose... trying again \n(" +
                #        str(e) + ")")
                #    last_warn = rospy.Time.now()
                print("LookupException happened!!")
                rospy.sleep(1.0)

        target_ps.header.stamp = rospy.Time.now()
        return target_ps

    def run(self):
        br = tf2_ros.TransformBroadcaster()
        ps = Pose()
        ps.orientation.w = 1.0  # Quaternion must be correct
        while not rospy.is_shutdown():
            # We transform a pose with reference frame
            # self.frame_to_posestamped
            # which is 0.0, 0.0, 0.0
            # to the reference frame to get it's pose
            tfed_ps = self.transform_pose(ps,
                                          self.frame_to_posestamped,
                                          self.reference_frame)
            ## IMPORTANT
            # Sets controllers' position relative to the gimble/vive
            tfed_ps.header.frame_id=self.header_frame_id
            self.pose_pub.publish(tfed_ps)

            t = TransformStamped()
            t.header.stamp = rospy.Time.now()
            t.header.frame_id = self.header_frame_id
            t.child_frame_id = self.header_frame_id+"_"+self.frame_to_posestamped
            t.transform.translation.x = tfed_ps.pose.position.x
            t.transform.translation.y = tfed_ps.pose.position.y
            t.transform.translation.z = tfed_ps.pose.position.z

            t.transform.rotation.x = tfed_ps.pose.orientation.x
            t.transform.rotation.y = tfed_ps.pose.orientation.y
            t.transform.rotation.z = tfed_ps.pose.orientation.z
            t.transform.rotation.w = tfed_ps.pose.orientation.w
            
            br.sendTransform(t)
            if self.verbose:
                print(tfed_ps)
            self.rate.sleep()


if __name__ == '__main__':
    rospy.init_node('frame_to_posestamped')
    argv = rospy.myargv(sys.argv)
    if len(argv) < 3:
        print("Usage:")
        print(argv[0] + " frame_to_posestamped reference_frame [rate]")
        exit(0)
    frame_to_posestamped = argv[1]
    reference_frame = argv[2]
    header_frame_id = argv[3]
    controller_topic_name = argv[4]
    if len(argv) == 6:
        rate = int(argv[5])
    else:
        rate = 10
    pfaps = PublishFrameAsPoseStamped(frame_to_posestamped,
                                      reference_frame,
                                      header_frame_id,
                                      controller_topic_name,
                                      rate,
                                      verbose=False)
    pfaps.run()
