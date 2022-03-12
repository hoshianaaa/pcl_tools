#!/usr/bin/env python

import sys
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import *

import rospy
import tf

import math

import json
import os

def read_json_file(file_name):
  if os.path.exists(file_name):
    with open(file_name, 'r') as f:
      json_load = json.load(f)
      return json_load, True
  return None, False

def write_json_file(file_name, dict_data):
  with open(file_name, 'w') as f:
    json.dump(dict_data, f, indent=2)

X = 0.00
Y = 0.00
Z = 0.00
EX = 0
EY = 0
EZ = 0

class Window(QWidget):

    def __init__(self):
        super().__init__()

        self.initUI()
    
    def setLableValue(self):

        global X,Y,Z
        global EX,EY,EZ

        self.angle_label[0].setText("X:" + str(round(X,2)))
        self.angle_label[1].setText("Y:" + str(round(Y,2)))
        self.angle_label[2].setText("Z:" + str(round(Z,2)))
        self.angle_label[3].setText("EX:" + str(round(EX,2)))
        self.angle_label[4].setText("EY:" + str(round(EY,2)))
        self.angle_label[5].setText("EZ:" + str(round(EZ,2)))

    def initUI(self):
        
        self.angle_label = [QLabel() for x in range(6)]

        self.setLableValue()
 
        self.angle_slider = [QSlider(Qt.Orientation.Horizontal) for x in range(6)]
        
        for each_slider in self.angle_slider:
            each_slider.setMaximum(100)
            each_slider.setMinimum(-100)
            each_slider.setValue(0)
            each_slider.valueChanged.connect(self.value_change)
        
        self.angle_slider[0].setValue(int(X * 100))
        self.angle_slider[1].setValue(int(Y * 100))
        self.angle_slider[2].setValue(int(Z * 100))
        self.angle_slider[3].setValue(int(EX * 100 / math.pi))
        self.angle_slider[4].setValue(int(EY * 100 / math.pi))
        self.angle_slider[5].setValue(int(EZ * 100 / math.pi))

        servo_frame = [QFrame() for x in range(6)]
        servo_layout = [QHBoxLayout() for x in range(6)]

        vbox = QVBoxLayout()

        for i in range(6):
            servo_layout[i].addWidget(self.angle_label[i])
            servo_layout[i].addWidget(self.angle_slider[i])

            servo_frame[i].setLayout(servo_layout[i])
            vbox.addWidget(servo_frame[i])  
            
        self.setLayout(vbox)
        self.setGeometry(300, 300, 450, 300)
        self.show()
    
    def value_change(self):
        global X,Y,Z
        global EX,EY,EZ

        X = self.angle_slider[0].value() / 100.0
        Y = self.angle_slider[1].value() / 100.0
        Z = self.angle_slider[2].value() / 100.0
        EX = self.angle_slider[3].value() / 100.0 * math.pi
        EY = self.angle_slider[4].value() / 100.0 * math.pi
        EZ = self.angle_slider[5].value() / 100.0 * math.pi

        self.setLableValue()


try:

  rospy.init_node('tf_publisher_gui')
  br = tf.TransformBroadcaster()

  args = sys.argv
  file_name = args[1]
  print(file_name)
  data, read_sucess = read_json_file(file_name)
  print(data)

  X = data["x"]
  Y = data["y"]
  Z = data["z"]
  EX = data["ex"]
  EY = data["ey"]
  EZ = data["ez"]

  app = QApplication(sys.argv)
  ex =Window()

  r = rospy.Rate(10)
  while not rospy.is_shutdown():
      QApplication.processEvents()

      br.sendTransform(( X, Y, Z),
                       tf.transformations.quaternion_from_euler(EX, EY, EZ),
                       rospy.Time.now(),
                       "camera",
                       "base_link")

      r.sleep()

finally:
  print("finally!!!!!!!!!!!!!!!!!!!!!!!!!!!!")

  new_data = {'x': X, 'y': Y, 'z': Z, 'ex': EX, 'ey': EY, 'ez': EZ}
  data.update(new_data)
  write_json_file(file_name, new_data)

