#!/usr/bin/env python

import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import numpy as np

import cv2

_one_pixel_length = 0.0005 # 1 pixel length (/m)
_image_pixel_zero_pos = []
_max_z = 0

def callback(point_cloud):
        points = np.array(list(pc2.read_points(point_cloud, skip_nans=True)))
        z_points = points[:, [2]]
        _max_z = max(z_points)

        points_surface = points[:, [0, 1]] # choose x, y

        points_surface = points_surface[np.argsort(points_surface[:, 0])] # sort x

        min_x = points_surface[0, 0]
        max_x = points_surface[-1, 0]

        points_surface = points_surface[np.argsort(points_surface[:, 1])] # sort x

        min_y = points_surface[0, 1]
        max_y = points_surface[-1, 1]

        _image_pixel_zero_pos = [min_x, min_y]

        pixel_size_x = ((max_x - min_x) / _one_pixel_length)
        pixel_size_y = ((max_y - min_y) / _one_pixel_length)

        pixel_size_x_int = int((max_x - min_x) / _one_pixel_length)
        pixel_size_y_int = int((max_y - min_y) / _one_pixel_length)

        if((pixel_size_x - pixel_size_x_int) >= 0.5):
          pixel_size_x_int = pixel_size_x_int + 1
        if((pixel_size_y - pixel_size_y_int) >= 0.5):
          pixel_size_y_int = pixel_size_y_int + 1

        np_image = np.zeros((pixel_size_x_int,pixel_size_y_int,3))
        print("image size:",pixel_size_x_int, pixel_size_y_int)

        for i in points_surface:
          x = i[0]
          y = i[1]

          pix_x = (x - _image_pixel_zero_pos[0]) / _one_pixel_length
          pix_y = (y - _image_pixel_zero_pos[1]) / _one_pixel_length

          pix_x_int = int((x - _image_pixel_zero_pos[0]) / _one_pixel_length)
          pix_y_int = int((y - _image_pixel_zero_pos[1]) / _one_pixel_length)

          if (pix_x - pix_x_int >= 0.5):
            pix_x_int = pix_x_int + 1
          if (pix_y - pix_y_int >= 0.5):
            pix_y_int = pix_y_int + 1

          np_image[pix_x_int-1][pix_y_int-1] = [255,255,255]

        img = np_image.astype(np.uint8)  

        kernel = np.ones((5,5),np.uint8)
        img = cv2.morphologyEx(img, cv2.MORPH_CLOSE, kernel)

        cv2.imshow('image', img)
        cv2.imwrite('image.png', img)
        cv2.waitKey(1)

        bridge = CvBridge()
        imgMsg = bridge.cv2_to_imgmsg(img, "bgr8")
        pub.publish(imgMsg)


rospy.init_node('find_rectangle_pos')
rospy.Subscriber('output', PointCloud2, callback)
pub = rospy.Publisher('/usb_cam/image_raw', Image, queue_size=10)
rospy.spin()
