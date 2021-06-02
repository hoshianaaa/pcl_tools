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

import math

from jsk_recognition_msgs.msg import PolygonArray
from geometry_msgs.msg import Polygon, PolygonStamped, Point32
from std_msgs.msg import Header

square_edge_len = 18 # mm
pixel_len = 0.5 # mm

square_edge_pixel_len = square_edge_len / pixel_len

square_area = square_edge_len * square_edge_len # mm^2
one_pixel_area = pixel_len * pixel_len # mm^2

square_area_pixel = square_area / one_pixel_area

print(square_area_pixel)

area_th = square_area_pixel * 0.8

right_angle_error_th =  0.4

white_rate_th = 0.8

def get_polygon_white_rate(polygon,img):

  black_num = 0
  white_num = 0

  for i in range(len(img)):
    for j in range(len(img[0])):
      pt = (j,i)
      if cv2.pointPolygonTest(polygon, pt, False) >= 0:

        if(img[i][j][0] == 0):
          black_num = black_num + 1
        else:
          white_num = white_num + 1

  print("w:",white_num)
  print("b:",black_num)

  s = white_num + black_num

  rate = float(white_num) / s

  print("rate:",rate)

  return rate

def angle(pt1, pt2, pt0):
    dx1 = float(pt1[0,0] - pt0[0,0])
    dy1 = float(pt1[0,1] - pt0[0,1])
    dx2 = float(pt2[0,0] - pt0[0,0])
    dy2 = float(pt2[0,1] - pt0[0,1])
    v = math.sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) )
    return (dx1*dx2 + dy1*dy2)/ v

def unit_vec(pt1, pt2):

    v = pt2 - pt1
    v = v.astype(np.float64)
    l = np.linalg.norm(v)

    return v / l

def rotate(vec,deg):
  rad = math.radians(deg)
  r_vec_x = math.cos(rad) * vec[0] - math.sin(rad) * vec[1]
  r_vec_y = math.sin(rad) * vec[0] + math.cos(rad) * vec[1]
  return np.array([r_vec_x,r_vec_y])

def square_points(pt1, pt2, pt0):

    print("pt1 pt2 pt0")
    print(pt1,pt2,pt0)

    v1 = unit_vec(pt0, pt1)
    v2 = unit_vec(pt0, pt2)

    v_mean = (v1 + v2) / np.linalg.norm(v1 + v2)

    l = square_edge_pixel_len * np.sqrt(2)

    sq_pt2 = (pt0 + v_mean * l).astype(np.int64)

    sq_pt1 = (pt0 + rotate(v_mean,45) * square_edge_pixel_len).astype(np.int64)
    sq_pt3 = (pt0 + rotate(v_mean,-45) * square_edge_pixel_len).astype(np.int64)

    return np.array([pt0, sq_pt1, sq_pt2, sq_pt3])

def findSquares(bin_image, image, cond_area = 1000):

    polygons = []
    centers = []

    i, contours, _ = cv2.findContours(bin_image, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE) #python2
    # contours, _ = cv2.findContours(bin_image, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE) #python3

    origin_img = image.copy()
    resimg = image.copy()
    for i, cnt in enumerate(contours):
        arclen = cv2.arcLength(cnt, True)
        approx = cv2.approxPolyDP(cnt, arclen*0.01, True)

        cv2.polylines(resimg, [approx.reshape(-1,2)], True, (255,0,0), thickness=2, lineType=cv2.LINE_8)

        area = abs(cv2.contourArea(approx))
        if area > area_th:
          print("area:",area)
          print("approx:",approx)
          print("convex:",cv2.isContourConvex(approx))

          for j in range(2,len(approx)):
            cosine = abs(angle(approx[j], approx[j-2], approx[j-1]))

            if cosine < right_angle_error_th:
              sq_points = square_points(approx[j][0], approx[j-2][0], approx[j-1][0])
              sq_points_rs = sq_points.reshape(1, -1, 2)
              cv2.polylines(resimg, sq_points_rs, isClosed=True, color=(0, 255, 0), thickness=2)
              w_rate = get_polygon_white_rate(sq_points, origin_img)
              print("white rate:",w_rate)
              if w_rate > white_rate_th:
                cv2.polylines(resimg, sq_points_rs, isClosed=True, color=(0, 255, 0), thickness=2)
                polygons.append(sq_points)
                x = float(sq_points[0][0] + sq_points[2][0]) / 2
                y = float(sq_points[0][1] + sq_points[2][1]) / 2
                centers.append([x,y])
              else:
                cv2.polylines(resimg, sq_points_rs, isClosed=True, color=(0, 0, 255), thickness=2)

   
    cv2.imshow('image', resimg)
    cv2.waitKey(1)

    return [polygons,centers]

class DetectGraspablePosesPcabase(ConnectionBasedTransport):

    def __init__(self):

        rospy.logwarn("This node is experiential one.")

        super(DetectGraspablePosesPcabase, self).__init__()

        self.one_pixel_length = rospy.get_param('~one_pixel_length', 0.0005) # 1 pixel length (/m)
        self.pub_target_poses = self.advertise("~output/can_grasp_poses", PoseArray, queue_size=1)

        self.pub_polygons = rospy.Publisher("~output/polygons", PolygonArray)

        self.image_pixel_zero_pos = []

        self.max_z = 0

    def pixel_to_pos(self,pixel):

        x = pixel[1] * self.one_pixel_length + self.image_pixel_zero_pos[0]
        y = pixel[0] * self.one_pixel_length + self.image_pixel_zero_pos[1]

        return [x,y]

    def subscribe(self):
        rospy.Subscriber('~input', PointCloud2, self.callback)

    def unsubscribe(self):
        self.sub.unregister()
    
    def callback(self, point_cloud):

        print("callback")

        points = np.array(list(pc2.read_points(point_cloud, skip_nans=True)))
        z_points = points[:, [2]]
        self.max_z = max(z_points)

        points_surface = points[:, [0, 1]] # choose x, y

        points_surface = points_surface[np.argsort(points_surface[:, 0])] # sort x

        min_x = points_surface[0, 0]
        max_x = points_surface[-1, 0]

        points_surface = points_surface[np.argsort(points_surface[:, 1])] # sort x

        min_y = points_surface[0, 1]
        max_y = points_surface[-1, 1]

        self.image_pixel_zero_pos = [min_x, min_y]

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

          pix_x = (x - self.image_pixel_zero_pos[0]) / self.one_pixel_length
          pix_y = (y - self.image_pixel_zero_pos[1]) / self.one_pixel_length

          pix_x_int = int((x - self.image_pixel_zero_pos[0]) / self.one_pixel_length)
          pix_y_int = int((y - self.image_pixel_zero_pos[1]) / self.one_pixel_length)

          if (pix_x - pix_x_int >= 0.5):
            pix_x_int = pix_x_int + 1
          if (pix_y - pix_y_int >= 0.5):
            pix_y_int = pix_y_int + 1

          np_image[pix_x_int-1][pix_y_int-1] = [255,255,255]

        img = np_image.astype(np.uint8)

        kernel = np.ones((5,5),np.uint8)
        img = cv2.morphologyEx(img, cv2.MORPH_CLOSE, kernel)

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        _, bw = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)
        ret = findSquares(bw, img)

        img_polygons = ret[0]
        img_centers = ret[1]

        polygons = []
        centers = []

        for i in range(len(img_polygons)):
          polygon = []
          for j in range(len(img_polygons[0])):
            pos_xy = self.pixel_to_pos(img_polygons[i][j])
            polygon.append([pos_xy[0],pos_xy[1],self.max_z])

          polygons.append(polygon)

          pos_xy = self.pixel_to_pos(img_centers[i])
          centers.append([pos_xy[0],pos_xy[1],self.max_z])

        header = point_cloud.header

        msg = PolygonArray()
        msg.header = header    

        for i in range(len(polygons)):
          p = PolygonStamped()
          p.header = header
          for j in range(len(polygons[0])):
            xyz = polygons[i][j]
            p.polygon.points.append(Point32(x = xyz[0], y = xyz[1], z = xyz[2]))

          msg.polygons.append(p)

        self.pub_polygons.publish(msg)    
       
        pub_msg = PoseArray()
        pub_msg.header = point_cloud.header

        for i in range(len(centers)):
          pose = Pose()
          pose.position.x = centers[i][0]
          pose.position.y = centers[i][1]
          pose.position.z = centers[i][2]

          pub_msg.poses.append(pose)

        self.pub_target_poses.publish(pub_msg)

if __name__ == '__main__':
    print("test")
    rospy.init_node('find_rectangle_pos')
    DetectGraspablePosesPcabase()
    rospy.spin()
