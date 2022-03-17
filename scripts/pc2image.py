#!/usr/bin/env python

import rospy

def callback(msg):
  pass

rospy.init_node('find_rectangle_pos')
rospy.Subscriber('~input', PointCloud2, callback)
rospy.spin()
