#!/usr/bin/env python

import numpy as np

import rospy
import sensor_msgs.point_cloud2 as pc2
import tf.transformations

from geometry_msgs.msg import Pose, PoseArray
from jsk_topic_tools import ConnectionBasedTransport
from sensor_msgs.msg import PointCloud2
from sklearn.decomposition import PCA

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2

class DetectGraspablePosesPcabase(ConnectionBasedTransport):

    def __init__(self):

        rospy.logwarn("This node is experiential one.")

        super(DetectGraspablePosesPcabase, self).__init__()

        self.one_pixel_length = rospy.get_param('~one_pixel_length', 0.0005) # 1 pixel length (/m)
        self.pub_target_poses = self.advertise("~output/can_grasp_poses", PoseArray, queue_size=1)

    def subscribe(self):
        rospy.Subscriber('~input', PointCloud2, self.callback)

    def unsubscribe(self):
        self.sub.unregister()

    def callback(self, point_cloud):

        print("callback")

        points = np.array(list(pc2.read_points(point_cloud, skip_nans=True)))
        points_surface = points[:, [0, 1]] # choose x, y

        points_surface = points_surface[np.argsort(points_surface[:, 0])] # sort x

        min_x = points_surface[0, 0]
        max_x = points_surface[-1, 0]

        points_surface = points_surface[np.argsort(points_surface[:, 1])] # sort x

        min_y = points_surface[0, 1]
        max_y = points_surface[-1, 1]

        image_pixel_zero_pos = [min_x, min_y]

        pixel_size_x = ((max_x - min_x) / self.one_pixel_length)
        pixel_size_y = ((max_y - min_y) / self.one_pixel_length)

        pixel_size_x_int = int((max_x - min_x) / self.one_pixel_length)
        pixel_size_y_int = int((max_y - min_y) / self.one_pixel_length)

        if((pixel_size_x - pixel_size_x_int) >= 0.5):
          pixel_size_x_int = pixel_size_x_int + 1
        if((pixel_size_y - pixel_size_y_int) >= 0.5):
          pixel_size_y_int = pixel_size_y_int + 1

        np_image = np.zeros((pixel_size_x_int,pixel_size_y_int,3))
        print("image size:",pixel_size_x_int, pixel_size_y_int)

        for i in points_surface:
          x = i[0]
          y = i[1]

          pix_x = (x - image_pixel_zero_pos[0]) / self.one_pixel_length
          pix_y = (y - image_pixel_zero_pos[1]) / self.one_pixel_length

          pix_x_int = int((x - image_pixel_zero_pos[0]) / self.one_pixel_length)
          pix_y_int = int((y - image_pixel_zero_pos[1]) / self.one_pixel_length)

          if (pix_x - pix_x_int >= 0.5):
            pix_x_int = pix_x_int + 1
          if (pix_y - pix_y_int >= 0.5):
            pix_y_int = pix_y_int + 1

          np_image[pix_x_int-1][pix_y_int-1] = [255,255,255]

        img = np_image.astype(np.uint8)

        gray= cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        gray = cv2.bitwise_not(gray)

        corners= cv2.goodFeaturesToTrack(gray, 100, 0.01, 50)

        for corner in corners:
          x,y= corner[0]
          x= int(x)
          y= int(y)
          cv2.rectangle(img, (x-10,y-10),(x+10,y+10),(255,0,0),-1)

        print(img[100,100])
        print(img.dtype)
        cv2.imshow('image', img)
        cv2.waitKey(1)

        pub_msg = PoseArray()
        pub_msg.header = point_cloud.header

        self.pub_target_poses.publish(pub_msg)

if __name__ == '__main__':
    print("test")
    rospy.init_node('find_rectangle_pos')
    DetectGraspablePosesPcabase()
    rospy.spin()
