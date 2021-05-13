#!/usr/bin/env python

import numpy as np

import rospy
import sensor_msgs.point_cloud2 as pc2
import tf.transformations

from geometry_msgs.msg import Pose, PoseArray
from jsk_topic_tools import ConnectionBasedTransport
from sensor_msgs.msg import PointCloud2
from sklearn.decomposition import PCA

class DetectGraspablePosesPcabase(ConnectionBasedTransport):

    def __init__(self):

        rospy.logwarn("This node is experiential one.")

        super(DetectGraspablePosesPcabase, self).__init__()

        self.pub_target_poses = self.advertise("~output/can_grasp_poses", PoseArray, queue_size=1)

    def subscribe(self):
        rospy.Subscriber('~input', PointCloud2, self.callback)

    def unsubscribe(self):
        self.sub.unregister()

    def callback(self, point_cloud):

        points = np.array(list(pc2.read_points(point_cloud, skip_nans=True)))
        if points.size == 0:
          pub_msg = PoseArray()
          pub_msg.header = point_cloud.header
          self.pub_target_poses.publish(pub_msg)
          return

        index = np.argsort(points[:,2])
        points_sorted = points[index,:]

        pose = Pose()

        pose.position.x = points_sorted[-1][0]
        pose.position.y = points_sorted[-1][1]
        pose.position.z = points_sorted[-1][2]

        pose.orientation.x = 0
        pose.orientation.y = 0
        pose.orientation.z = 0
        pose.orientation.w = 1

        pub_msg = PoseArray()
        pub_msg.header = point_cloud.header

        pub_msg.poses.append(pose)

        self.pub_target_poses.publish(pub_msg)

if __name__ == '__main__':
    print("test")
    rospy.init_node('get_highest_pos')
    DetectGraspablePosesPcabase()
    rospy.spin()
