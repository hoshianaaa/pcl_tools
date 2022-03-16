#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy

from sensor_msgs.msg import PointCloud2

def callback(msg):
  print(msg)

rospy.init_node("cropper")
rospy.Subscriber("/hand_cream3", PointCloud2 , callback)
rospy.spin()
