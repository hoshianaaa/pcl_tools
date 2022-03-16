#!/usr/bin/python
# -*- coding: utf-8 -*-

from __future__ import print_function

import rospy
from visualization_msgs.msg import *
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import Point
import laser_geometry.laser_geometry as lg
import tf2_ros
import tf2_geometry_msgs
import tf2_sensor_msgs.tf2_sensor_msgs

def point(x,y,z):
  p = Point()
  p.x = x
  p.y = y
  p.z = z
  return p



class Raytrace:
    def __init__(self, pub_name, sub_name, sub_type):
        self.tfBuffer = tf2_ros.Buffer()
        self.listerner = tf2_ros.TransformListener(self.tfBuffer)
        self.lp = lg.LaserProjection()
        self.pub = rospy.Publisher(pub_name, Marker, queue_size=10)
        self.sub = rospy.Subscriber(
            sub_name, sub_type, self.pc2_callback)

    def pc2_callback(self, cloud):
        target_frame = "base_link"
        local_sensor_origin = tf2_geometry_msgs.PointStamped()
        local_sensor_origin.header.frame_id = cloud.header.frame_id
        local_sensor_origin.header.stamp = rospy.Time(0)

        while True:
            try:
                trans = self.tfBuffer.lookup_transform(
                    target_frame, cloud.header.frame_id, rospy.Time(0), rospy.Duration(1.0))
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                continue
            marker = Marker()
            marker.header.frame_id = target_frame
            marker.ns = "ray"
            marker.id = 0
            marker.type = Marker.LINE_LIST
            # LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
            marker.scale.x = 0.001
            marker.color.b = 1.0
            marker.color.a = 1.0
            global_sensor_origin = tf2_geometry_msgs.do_transform_point(
                local_sensor_origin, trans)

            global_cloud = tf2_sensor_msgs.tf2_sensor_msgs.do_transform_cloud(
                cloud, trans)

            marker.points.append(point(0,-5,0))
            marker.points.append(point(0,5,0))

            for p in pc2.read_points(global_cloud, field_names=(
                    "x", "y", "z"), skip_nans=True):
                target_point = Point()
                target_point.x = p[0]
                target_point.y = p[1]
                target_point.z = p[2]
            self.pub.publish(marker)
            break

if __name__ == "__main__":
    rospy.init_node("visualize_raytrace")
    pub_name = "/visualize_ray/maker"
    sub_name = "/hand_cream3"
    sub_type = PointCloud2
    raytrace = Raytrace(pub_name, sub_name, sub_type)
    rospy.spin()

