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

        self.hand_size = rospy.get_param('~hand_size', 0.3) # maximum hand width
        self.deep_z = rospy.get_param('~deep_z', 0.03) # maximum hand width

        self.pub_target_poses = self.advertise("~output/can_grasp_poses", PoseArray, queue_size=1)
        self.pub_debug_highest_poses = self.advertise("~output/debug_highest_pos", PoseArray, queue_size=1)

    def subscribe(self):
        rospy.Subscriber('~input', PointCloud2, self.callback)

    def unsubscribe(self):
        self.sub.unregister()

    def callback(self, point_cloud):
        print("callback")

        points = np.array(list(pc2.read_points(point_cloud, skip_nans=True)))

        if points.size == 0:
          pub_msg = PoseArray()
          pub_msg.header = point_cloud.header
          self.pub_target_poses.publish(pub_msg)
          self.pub_debug_highest_poses.publish(pub_msg)
          return

        index = np.argsort(points[:,2])
        points_sorted = points[index,:]
        max_z = points_sorted[-1][2]

        points_surface = points[:, [0, 1]] # choose x, y

        pca = PCA(n_components=2)
        pca.fit(points_surface)
        new_points = pca.transform(points_surface)
        new_points = new_points[np.argsort(new_points[:, 0])]

        pub_msg = PoseArray()
        pub_msg.header = point_cloud.header

        trans_matrix = tf.transformations.identity_matrix()
        
        trans_matrix[0, 0] = 0
        trans_matrix[1, 0] = 0
        trans_matrix[2, 0] = -1
        trans_matrix[0, 1] = -1 * pca.components_[0, 1]
        trans_matrix[1, 1] = pca.components_[0, 0]
        trans_matrix[0, 2] = pca.components_[0, 0]
        trans_matrix[1, 2] = pca.components_[0, 1]
        trans_matrix[2, 2] = 0

        quaternion = tf.transformations.quaternion_from_matrix(trans_matrix)

        pose = Pose()

        pose.position.x = pca.mean_[0]
        pose.position.y = pca.mean_[1]
        pose.position.z = max_z - self.deep_z

        pose.orientation.x = quaternion[0]
        pose.orientation.y = quaternion[1]
        pose.orientation.z = quaternion[2]
        pose.orientation.w = quaternion[3]

        pub_msg.poses.append(pose)


        self.pub_target_poses.publish(pub_msg)

        pub_msg = PoseArray()
        pub_msg.header = point_cloud.header

        pose.position.z = max_z
        pub_msg.poses.append(pose)

        self.pub_debug_highest_poses.publish(pub_msg)


if __name__ == '__main__':
    print("test")
    rospy.init_node('detect_graspable_poses_pcabase')
    DetectGraspablePosesPcabase()
    rospy.spin()
