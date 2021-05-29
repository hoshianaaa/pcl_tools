#!/usr/bin/env python

import numpy as np

import rospy

from geometry_msgs.msg import Pose, PoseArray, Vector3
from jsk_topic_tools import ConnectionBasedTransport

from jsk_recognition_msgs.msg import BoundingBox

import random

class PoseArraySelector(ConnectionBasedTransport):

    def __init__(self):

        rospy.logwarn("This node is experiential one.")

        super(PoseArraySelector, self).__init__()

        self.distance = rospy.get_param('~distance', '0.03')
        self.queue_num = rospy.get_param('~queue_num', '3')
        self.frame_id = rospy.get_param('~frame_id', 'base_link')

        self.pub_output_poses = self.advertise("~output/select_poses", PoseArray, queue_size=1)

    def subscribe(self):
        rospy.Subscriber('~input', PoseArray, self.callback)

    def unsubscribe(self):
        self.sub.unregister()

    def callback(self, msg):

        print("pose array clipper callback")

        posearray = msg.poses
        select_pose = posearray[random.randint(0,len(posearray)) - 1]

        #sorted_posearray = posearray[posearray[:,2].argsort(), :][::-1]

        pub_msg = PoseArray()
        pub_msg.header = msg.header

        pub_msg.poses.append(select_pose)

        self.pub_output_poses.publish(pub_msg)

if __name__ == '__main__':
    print("test")
    rospy.init_node('posearray_clipper')
    PoseArraySelector()
    rospy.spin()
