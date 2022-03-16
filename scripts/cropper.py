#!/usr/bin/python
# -*- coding: utf-8 -*-

from __future__ import print_function

import rospy
from visualization_msgs.msg import *
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import Point
import laser_geometry.laser_geometry as lg
import tf2_ros
import tf2_geometry_msgs
import tf2_sensor_msgs.tf2_sensor_msgs
from std_msgs.msg import Header
from sensor_msgs import point_cloud2

def list2pc2(list, frame_id):

  points = []

  fields = [PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
            # PointField('rgb', 12, PointField.UINT32, 1),
            PointField('rgba', 12, PointField.UINT32, 1),
            ]

  for i in range(len(list)):

    x = list[i][0]
    y = list[i][1]
    z = list[i][2]
    r = list[i][3] # 0 ~ 255
    g = list[i][4] # 0 ~ 255
    b = list[i][5] # 0 ~ 255
    a = list[i][6] # 0 ~ 255
    rgb = struct.unpack('I', struct.pack('BBBB', b, g, r, a))[0]

    pt = [x, y, z, rgb]
    points.append(pt)

  header = Header()
  header.frame_id = frame_id
  pc2 = point_cloud2.create_cloud(header, fields, points)
  pc2.header.stamp = rospy.Time.now()

  return pc2

X = 0
Y = 0
Z = 0.25
DX = 0.5
DY = 0.5
DZ = 0.5

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
        self.pub_pc2 = rospy.Publisher("output", PointCloud2, queue_size=10)
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

            global X,Y,Z,DX,DY,DZ
            
            x = X - DX/2            
            y = Y - DY/2
            z = Z - DZ/2

            p1 = point(x,y,z)
            p2 = point(x + DX,y,z)
            p3 = point(x,y + DY,z)
            p4 = point(x + DX,y + DY,z)

            p5 = point(x,y,z + DZ)
            p6= point(x + DX,y,z + DZ)
            p7 = point(x,y + DY,z + DZ)
            p8 = point(x + DX,y + DY,z + DZ)

            marker.points.append(p1)
            marker.points.append(p2)

            marker.points.append(p1)
            marker.points.append(p3)

            marker.points.append(p4)
            marker.points.append(p3)

            marker.points.append(p4)
            marker.points.append(p2)

            marker.points.append(p5)
            marker.points.append(p6)

            marker.points.append(p5)
            marker.points.append(p7)

            marker.points.append(p8)
            marker.points.append(p6)

            marker.points.append(p8)
            marker.points.append(p7)

            marker.points.append(p1)
            marker.points.append(p5)

            marker.points.append(p2)
            marker.points.append(p6)

            marker.points.append(p3)
            marker.points.append(p7)

            marker.points.append(p4)
            marker.points.append(p8)

            p_list = []

            for p in pc2.read_points(global_cloud , field_names=(
                    "x", "y", "z"), skip_nans=True):

              px = p[0]
              py = p[1]
              pz = p[2]

              if px >= X - DX/2 and px <= X + DX/2:
                if py >= Y - DY/2 and py <= Y + DY/2:
                  if pz >= Z - DZ/2 and pz <= Z + DZ/2:
                    p_list.append([px,py,pz,255,255,255,255])

            self.pub_pc2.publish(list2pc2(p_list, target_frame))
            
            self.pub.publish(marker)
            break

if __name__ == "__main__":
    rospy.init_node("visualize_raytrace")
    pub_name = "/visualize_ray/maker"
    sub_name = "/hand_cream3"
    sub_type = PointCloud2
    raytrace = Raytrace(pub_name, sub_name, sub_type)
    rospy.spin()

