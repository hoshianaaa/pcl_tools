#!/usr/bin/env python

import numpy as np

import rospy

from geometry_msgs.msg import Pose, PoseArray, Vector3
from jsk_topic_tools import ConnectionBasedTransport

from jsk_recognition_msgs.msg import BoundingBox

class PoseArrayClipper(ConnectionBasedTransport):

    def __init__(self):

        rospy.logwarn("This node is experiential one.")

        super(PoseArrayClipper, self).__init__()

        self.initial_pos = rospy.get_param('~initial_pos', '[0.3, 0, 0.11]')
        self.dimension_x = rospy.get_param('~dimension_x', '0.11')
        self.dimension_y = rospy.get_param('~dimension_y', '0.26')
        self.dimension_z = rospy.get_param('~dimension_z', '0.2')
        self.frame_id = rospy.get_param('~frame_id', 'base_link')

        self.pub_output_poses = self.advertise("~output", PoseArray, queue_size=1)

        self.pub_clipper_bbox = self.advertise("~output/clipper_bbox", BoundingBox, queue_size=1)

    def subscribe(self):
        rospy.Subscriber('~input', PoseArray, self.callback)

    def unsubscribe(self):
        self.sub.unregister()

    def callback(self, msg):

        print("pose array clipper callback")

        posearray = msg.poses

        x_min = self.initial_pos[0] - self.dimension_x / 2
        x_max = self.initial_pos[0] + self.dimension_x / 2

        y_min = self.initial_pos[1] - self.dimension_y / 2
        y_max = self.initial_pos[1] + self.dimension_y / 2

        z_min = self.initial_pos[2] - self.dimension_z / 2
        z_max = self.initial_pos[2] + self.dimension_z / 2

        pub_msg = PoseArray()
        pub_msg.header = msg.header

        for input_pose in posearray:

          x = input_pose.position.x
          y = input_pose.position.y
          z = input_pose.position.z

          if (x >= x_min) and (x <= x_max):
            if (y >= y_min) and (y <= y_max):
              if (z >= z_min) and (z <= z_max):
                pub_msg.poses.append(input_pose)

        self.pub_output_poses.publish(pub_msg)


        bbox = BoundingBox()

        pose = Pose()
        pose.position.x = self.initial_pos[0]
        pose.position.y = self.initial_pos[1]
        pose.position.z = self.initial_pos[2]
        bbox.pose = pose

        vec = Vector3()
        vec.x = self.dimension_x
        vec.y = self.dimension_y
        vec.z = self.dimension_z
        bbox.dimensions = vec

        bbox.header = msg.header

        self.pub_clipper_bbox.publish(bbox)

if __name__ == '__main__':
    print("test")
    rospy.init_node('posearray_clipper')
    PoseArrayClipper()
    rospy.spin()
