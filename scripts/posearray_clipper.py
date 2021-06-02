#!/usr/bin/env python

import numpy as np

import rospy

from geometry_msgs.msg import Pose, PoseArray, Vector3, PoseStamped

from jsk_topic_tools import ConnectionBasedTransport

from jsk_recognition_msgs.msg import BoundingBox

import tf2_ros
import tf2_geometry_msgs

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

        self.tf_buffer = tf2_ros.Buffer() #tf buffer length
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

    def subscribe(self):
        rospy.Subscriber('~input', PoseArray, self.callback)

    def unsubscribe(self):
        self.sub.unregister()

    def callback(self, msg):

        target_frame = self.frame_id
        input_frame = msg.header.frame_id
        
        transform = self.tf_buffer.lookup_transform(target_frame, 
                                               input_frame, #source frame
                                               rospy.Time(0),
                                               rospy.Duration(1.0))


        print("pose array clipper callback")

        posearray = msg.poses


        x_min = self.initial_pos[0] - self.dimension_x / 2
        x_max = self.initial_pos[0] + self.dimension_x / 2

        y_min = self.initial_pos[1] - self.dimension_y / 2
        y_max = self.initial_pos[1] + self.dimension_y / 2

        z_min = self.initial_pos[2] - self.dimension_z / 2
        z_max = self.initial_pos[2] + self.dimension_z / 2

        pub_msg = PoseArray()

        for input_pose in posearray:

          pose_stamped = PoseStamped()
          pose_stamped.pose = input_pose
          tf_pose = tf2_geometry_msgs.do_transform_pose(pose_stamped , transform)

          x = tf_pose.pose.position.x
          y = tf_pose.pose.position.y
          z = tf_pose.pose.position.z

          if (x >= x_min) and (x <= x_max):
            if (y >= y_min) and (y <= y_max):
              if (z >= z_min) and (z <= z_max):
                pub_msg.poses.append(tf_pose.pose)

        pub_msg.header.frame_id = self.frame_id
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

        bbox.header.frame_id = self.frame_id

        self.pub_clipper_bbox.publish(bbox)

if __name__ == '__main__':
    print("test")
    rospy.init_node('posearray_clipper')
    PoseArrayClipper()
    rospy.spin()
