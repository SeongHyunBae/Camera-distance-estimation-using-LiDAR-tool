# -*- coding: utf-8 -*-
import sys
import os
import cv2
import numpy as np
import signal
from PIL import Image

import rospy
import roslib
import roslaunch
import rosbag
import ros_numpy
import pyrosbag as prb
import pcl_ros
import subprocess

import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import PyQt5
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtWidgets import QApplication, QWidget, QInputDialog, QLineEdit, QFileDialog
from PyQt5.QtWidgets import QFrame
from PyQt5.QtGui import QIcon, QImage, QPainter, QPalette, QPixmap
from PyQt5.QtCore import QTimer

import PySide2
from PySide2.QtUiTools import QUiLoader
from PySide2.QtWidgets import QWidget, QApplication, QPushButton, QLineEdit, QLabel, QCheckBox, QComboBox, QTextBrowser, QDoubleSpinBox, QSpinBox
from PySide2.QtCore import QFile, QObject

class Form(QObject):
    def __init__(self, ui_file, parent=None):
        super(Form, self).__init__(parent)

        ui_file = QFile(ui_file)
        ui_file.open(QFile.ReadOnly)
        loader = QUiLoader()
        self.window = loader.load(ui_file)
        ui_file.close()
        
        # get objects from ui
        self.widget = self.window.findChild(QWidget, 'centralwidget')

        self.camera_frame = self.window.findChild(QLabel, 'label')
        self.lidar_frame = self.window.findChild(QLabel, 'label_2')
        self.lidar_spin_box_1 = self.window.findChild(QDoubleSpinBox, 'doubleSpinBox')
        self.lidar_spin_box_2 = self.window.findChild(QDoubleSpinBox, 'doubleSpinBox_2')
        self.lidar_spin_box_3 = self.window.findChild(QSpinBox, 'spinBox')
        self.lidar_spin_box_4 = self.window.findChild(QDoubleSpinBox, 'doubleSpinBox_3')
        self.clibration_spin_box_pt1_x = self.window.findChild(QDoubleSpinBox, 'doubleSpinBox_4')
        self.clibration_spin_box_pt1_y = self.window.findChild(QDoubleSpinBox, 'doubleSpinBox_5')
        self.clibration_spin_box_pt1_xp = self.window.findChild(QDoubleSpinBox, 'doubleSpinBox_6')
        self.clibration_spin_box_pt1_yp = self.window.findChild(QDoubleSpinBox, 'doubleSpinBox_7')
        self.clibration_spin_box_pt2_x = self.window.findChild(QDoubleSpinBox, 'doubleSpinBox_8')
        self.clibration_spin_box_pt2_y = self.window.findChild(QDoubleSpinBox, 'doubleSpinBox_9')
        self.clibration_spin_box_pt2_xp = self.window.findChild(QDoubleSpinBox, 'doubleSpinBox_10')
        self.clibration_spin_box_pt2_yp = self.window.findChild(QDoubleSpinBox, 'doubleSpinBox_11')
        self.clibration_spin_box_pt3_x = self.window.findChild(QDoubleSpinBox, 'doubleSpinBox_12')
        self.clibration_spin_box_pt3_y = self.window.findChild(QDoubleSpinBox, 'doubleSpinBox_13')
        self.clibration_spin_box_pt3_xp = self.window.findChild(QDoubleSpinBox, 'doubleSpinBox_14')
        self.clibration_spin_box_pt3_yp = self.window.findChild(QDoubleSpinBox, 'doubleSpinBox_15')
        self.clibration_spin_box_pt4_x = self.window.findChild(QDoubleSpinBox, 'doubleSpinBox_16')
        self.clibration_spin_box_pt4_y = self.window.findChild(QDoubleSpinBox, 'doubleSpinBox_17')
        self.clibration_spin_box_pt4_xp = self.window.findChild(QDoubleSpinBox, 'doubleSpinBox_18')
        self.clibration_spin_box_pt4_yp = self.window.findChild(QDoubleSpinBox, 'doubleSpinBox_19')
        self.draw_spin_box_line_x = self.window.findChild(QDoubleSpinBox, 'doubleSpinBox_20')
        self.draw_spin_box_line_y = self.window.findChild(QDoubleSpinBox, 'doubleSpinBox_21')
        self.lidar_x_line_check_box = self.window.findChild(QCheckBox, 'checkBox')
        self.lidar_y_line_check_box = self.window.findChild(QCheckBox, 'checkBox_2')
        self.pt1_check_box = self.window.findChild(QCheckBox, 'checkBox_3')
        self.pt1_p_check_box = self.window.findChild(QCheckBox, 'checkBox_4')
        self.pt2_check_box = self.window.findChild(QCheckBox, 'checkBox_5')
        self.pt2_p_check_box = self.window.findChild(QCheckBox, 'checkBox_6')
        self.pt3_check_box = self.window.findChild(QCheckBox, 'checkBox_7')
        self.pt3_p_check_box = self.window.findChild(QCheckBox, 'checkBox_8')
        self.pt4_check_box = self.window.findChild(QCheckBox, 'checkBox_9')
        self.pt4_p_check_box = self.window.findChild(QCheckBox, 'checkBox_10')
        self.camera_x_line_check_box = self.window.findChild(QCheckBox, 'checkBox_11')
        self.camera_y_line_check_box = self.window.findChild(QCheckBox, 'checkBox_12')

        self.window.comboBox.deleteLater()
        self.window.comboBox_2.deleteLater()
        self.lidar_combo_box = ComboBox()
        self.camera_combo_box = ComboBox()
        self.lidar_combo_box.setFixedSize(200, 25)
        self.camera_combo_box.setFixedSize(200, 25)
        self.window.horizontalLayout_2.addWidget(self.lidar_combo_box)
        self.window.horizontalLayout_3.addWidget(self.camera_combo_box)
        
        self.window.label_37.deleteLater()
        self.window.label_38.deleteLater()
        blank = PySide2.QtWidgets.QLabel()
        blank_2 = PySide2.QtWidgets.QLabel()
        self.window.horizontalLayout_2.addWidget(blank)
        self.window.horizontalLayout_3.addWidget(blank_2)

        # set variables
        self.sub_camera = None
        self.sub_lidar = None
        self.camera_msg = None
        self.lidar_msg = None
        self.camera_map = None
        self.lidar_map = None
        self.bridge = CvBridge()

        self.lidar_x = 30.0
        self.lidar_y = 8.0
        self.intensity = 30
        self.scale = 20.0
        self.h = int(self.lidar_x * self.scale)
        self.w = int(2 * self.lidar_y * self.scale)

        self.pt1_x = 444.0
        self.pt1_y = 374.5
        self.pt1_xp = 41.25
        self.pt1_yp = 5.5
        self.pt2_x = 265.0
        self.pt2_y = 492.0
        self.pt2_xp = 3.0
        self.pt2_yp = 2.0
        self.pt3_x = 562.5
        self.pt3_y = 377.5
        self.pt3_xp = 41.25
        self.pt3_yp = -4.95
        self.pt4_x = 700.0
        self.pt4_y =  507.0
        self.pt4_xp = 3.0
        self.pt4_yp = -2.0

        self.line_x = 20.0
        self.line_y = 0.0

        # set Timer
        self.qTimer = QTimer()
        self.qTimer.setInterval(1) # 1000 ms = 1 s
        self.qTimer.timeout.connect(self.update)
        self.qTimer.start()

        # initilization
        rospy.init_node('lidar_camera_calibration', anonymous=True)
        self.init_combo_box()
        self.init_spin_box() 
        self.init_check_box()

        # show
        self.window.show()

    def init_combo_box(self):
        self.lidar_combo_box.popupAboutToBeShown.connect(self.lidar_combo_box_clicked)
        self.lidar_combo_box.activated.connect(self.lidar_topic_selected)
        self.camera_combo_box.popupAboutToBeShown.connect(self.camera_combo_box_clicked)
        self.camera_combo_box.activated.connect(self.camera_topic_selected)

    def lidar_combo_box_clicked(self):
        try:
            # update rostopic list
            self.lidar_combo_box.clear()
            topic_list = []
        
            topic_list_dict = dict(rospy.get_published_topics())

            for key in topic_list_dict.keys():
                if topic_list_dict[key] == 'sensor_msgs/PointCloud2':
                    topic_list.append(key)

            topic_list.sort()
            self.lidar_combo_box.addItems(topic_list)
        except:
            pass

    def lidar_topic_selected(self, i):
        selected_topic = self.lidar_combo_box.currentText()

        if selected_topic != '':
            if self.sub_lidar is not None:
                self.sub_lidar.unregister()
            self.sub_lidar = rospy.Subscriber(selected_topic, PointCloud2, self.lidar_msg_callback)

    def camera_combo_box_clicked(self):
        try:
            # update rostopic list
            self.camera_combo_box.clear()
            topic_list = []
        
            topic_list_dict = dict(rospy.get_published_topics())

            for key in topic_list_dict.keys():
                if topic_list_dict[key] == 'sensor_msgs/Image':
                    topic_list.append(key)

            topic_list.sort()
            self.camera_combo_box.addItems(topic_list)
        except:
            pass

    def camera_topic_selected(self, i):
        selected_topic = self.camera_combo_box.currentText()

        if selected_topic != '':
            if self.sub_camera is not None:
                self.sub_camera.unregister()
            self.sub_camera = rospy.Subscriber(selected_topic, Image, self.camera_msg_callback)

    def init_spin_box(self):
        self.lidar_spin_box_1.setRange(0.0, 100.0)
        self.lidar_spin_box_2.setRange(0.0, 100.0)
        self.lidar_spin_box_3.setRange(0, 255)
        self.lidar_spin_box_4.setRange(0.0, 100.0)
        self.clibration_spin_box_pt1_x.setRange(-10000.0, 10000.0)
        self.clibration_spin_box_pt1_y.setRange(-10000.0, 10000.0)
        self.clibration_spin_box_pt1_xp.setRange(-10000.0, 10000.0)
        self.clibration_spin_box_pt1_yp.setRange(-10000.0, 10000.0)
        self.clibration_spin_box_pt2_x.setRange(-10000.0, 10000.0)
        self.clibration_spin_box_pt2_y.setRange(-10000.0, 10000.0)
        self.clibration_spin_box_pt2_xp.setRange(-10000.0, 10000.0)
        self.clibration_spin_box_pt2_yp.setRange(-10000.0, 10000.0)
        self.clibration_spin_box_pt3_x.setRange(-10000.0, 10000.0)
        self.clibration_spin_box_pt3_y.setRange(-10000.0, 10000.0)
        self.clibration_spin_box_pt3_xp.setRange(-10000.0, 10000.0)
        self.clibration_spin_box_pt3_yp.setRange(-10000.0, 10000.0)
        self.clibration_spin_box_pt4_x.setRange(-10000.0, 10000.0)
        self.clibration_spin_box_pt4_y.setRange(-10000.0, 10000.0)
        self.clibration_spin_box_pt4_xp.setRange(-10000.0, 10000.0)
        self.clibration_spin_box_pt4_yp.setRange(-10000.0, 10000.0)
        self.draw_spin_box_line_x.setRange(0.0, 100.0)
        self.draw_spin_box_line_y.setRange(-100.0, 100.0)

        self.lidar_spin_box_1.setValue(30.0)
        self.lidar_spin_box_2.setValue(8.0)
        self.lidar_spin_box_3.setValue(30)
        self.lidar_spin_box_4.setValue(20.0)
        self.clibration_spin_box_pt1_x.setValue(444.0)
        self.clibration_spin_box_pt1_y.setValue(374.5)
        self.clibration_spin_box_pt1_xp.setValue(41.25)
        self.clibration_spin_box_pt1_yp.setValue(5.05)
        self.clibration_spin_box_pt2_x.setValue(265.0)
        self.clibration_spin_box_pt2_y.setValue(492.0)
        self.clibration_spin_box_pt2_xp.setValue(3.0)
        self.clibration_spin_box_pt2_yp.setValue(2.0)
        self.clibration_spin_box_pt3_x.setValue(562.5)
        self.clibration_spin_box_pt3_y.setValue(377.5)
        self.clibration_spin_box_pt3_xp.setValue(41.25)
        self.clibration_spin_box_pt3_yp.setValue(-4.95)
        self.clibration_spin_box_pt4_x.setValue(700.0)
        self.clibration_spin_box_pt4_y.setValue(507.0)
        self.clibration_spin_box_pt4_xp.setValue(3.0)
        self.clibration_spin_box_pt4_yp.setValue(-2.0)
        self.draw_spin_box_line_x.setValue(20.0)
        self.draw_spin_box_line_y.setValue(0.0)

        self.lidar_spin_box_1.valueChanged.connect(self.spin_box_changed)
        self.lidar_spin_box_2.valueChanged.connect(self.spin_box_changed)
        self.lidar_spin_box_3.valueChanged.connect(self.spin_box_changed)
        self.lidar_spin_box_4.valueChanged.connect(self.spin_box_changed)
        self.clibration_spin_box_pt1_x.valueChanged.connect(self.spin_box_changed)
        self.clibration_spin_box_pt1_y.valueChanged.connect(self.spin_box_changed)
        self.clibration_spin_box_pt1_xp.valueChanged.connect(self.spin_box_changed)
        self.clibration_spin_box_pt1_yp.valueChanged.connect(self.spin_box_changed)
        self.clibration_spin_box_pt2_x.valueChanged.connect(self.spin_box_changed)
        self.clibration_spin_box_pt2_y.valueChanged.connect(self.spin_box_changed)
        self.clibration_spin_box_pt2_xp.valueChanged.connect(self.spin_box_changed)
        self.clibration_spin_box_pt2_yp.valueChanged.connect(self.spin_box_changed)
        self.clibration_spin_box_pt3_x.valueChanged.connect(self.spin_box_changed)
        self.clibration_spin_box_pt3_y.valueChanged.connect(self.spin_box_changed)
        self.clibration_spin_box_pt3_xp.valueChanged.connect(self.spin_box_changed)
        self.clibration_spin_box_pt3_yp.valueChanged.connect(self.spin_box_changed)
        self.clibration_spin_box_pt4_x.valueChanged.connect(self.spin_box_changed)
        self.clibration_spin_box_pt4_y.valueChanged.connect(self.spin_box_changed)
        self.clibration_spin_box_pt4_xp.valueChanged.connect(self.spin_box_changed)
        self.clibration_spin_box_pt4_yp.valueChanged.connect(self.spin_box_changed)
        self.draw_spin_box_line_x.valueChanged.connect(self.spin_box_changed)
        self.draw_spin_box_line_y.valueChanged.connect(self.spin_box_changed)

        self.clibration_spin_box_pt1_x.setSingleStep(0.1)
        self.clibration_spin_box_pt1_y.setSingleStep(0.1)
        self.clibration_spin_box_pt1_xp.setSingleStep(0.1)
        self.clibration_spin_box_pt1_yp.setSingleStep(0.1)
        self.clibration_spin_box_pt2_x.setSingleStep(0.1)
        self.clibration_spin_box_pt2_y.setSingleStep(0.1)
        self.clibration_spin_box_pt2_xp.setSingleStep(0.1)
        self.clibration_spin_box_pt2_yp.setSingleStep(0.1)
        self.clibration_spin_box_pt3_x.setSingleStep(0.1)
        self.clibration_spin_box_pt3_y.setSingleStep(0.1)
        self.clibration_spin_box_pt3_xp.setSingleStep(0.1)
        self.clibration_spin_box_pt3_yp.setSingleStep(0.1)
        self.clibration_spin_box_pt4_x.setSingleStep(0.1)
        self.clibration_spin_box_pt4_y.setSingleStep(0.1)
        self.clibration_spin_box_pt4_xp.setSingleStep(0.1)
        self.clibration_spin_box_pt4_yp.setSingleStep(0.1)
        self.draw_spin_box_line_x.setSingleStep(0.1)
        self.draw_spin_box_line_y.setSingleStep(0.1)

    def spin_box_changed(self):
        self.lidar_x = self.lidar_spin_box_1.value()
        self.lidar_y = self.lidar_spin_box_2.value()
        self.intensity = self.lidar_spin_box_3.value()
        self.scale = self.lidar_spin_box_4.value()
        self.h = int(self.lidar_x * self.scale)
        self.w = int(2 * self.lidar_y * self.scale)
        self.line_x = self.draw_spin_box_line_x.value()
        self.line_y = self.draw_spin_box_line_y.value()
        self.pt1_x = self.clibration_spin_box_pt1_x.value()
        self.pt1_y = self.clibration_spin_box_pt1_y.value()
        self.pt1_xp = self.clibration_spin_box_pt1_xp.value()
        self.pt1_yp = self.clibration_spin_box_pt1_yp.value()
        self.pt2_x = self.clibration_spin_box_pt2_x.value()
        self.pt2_y = self.clibration_spin_box_pt2_y.value()
        self.pt2_xp = self.clibration_spin_box_pt2_xp.value()
        self.pt2_yp = self.clibration_spin_box_pt2_yp.value()
        self.pt3_x = self.clibration_spin_box_pt3_x.value()
        self.pt3_y = self.clibration_spin_box_pt3_y.value()
        self.pt3_xp = self.clibration_spin_box_pt3_xp.value()
        self.pt3_yp = self.clibration_spin_box_pt3_yp.value()
        self.pt4_x = self.clibration_spin_box_pt4_x.value()
        self.pt4_y = self.clibration_spin_box_pt4_y.value()
        self.pt4_xp = self.clibration_spin_box_pt4_xp.value()
        self.pt4_yp = self.clibration_spin_box_pt4_yp.value()

    def init_check_box(self):
        self.lidar_x_line_check_box.setChecked(True)
        self.lidar_y_line_check_box.setChecked(True)
        self.pt1_check_box.setChecked(True)
        self.pt1_p_check_box.setChecked(True)
        self.pt2_check_box.setChecked(True)
        self.pt2_p_check_box.setChecked(True)
        self.pt3_check_box.setChecked(True)
        self.pt3_p_check_box.setChecked(True)
        self.pt4_check_box.setChecked(True)
        self.pt4_p_check_box.setChecked(True)
        self.camera_x_line_check_box.setChecked(True)
        self.camera_y_line_check_box.setChecked(True)

    def camera_msg_callback(self, msg):
        self.camera_msg = msg
        
    def lidar_msg_callback(self, msg):
        self.lidar_msg = msg

    def draw_frames(self):
        if self.camera_msg is not None:
            cv_image = self.bridge.imgmsg_to_cv2(self.camera_msg, desired_encoding="passthrough")
            h, w, _ = cv_image.shape
            self.camera_frame.resize(h, w)
            self.camera_map = cv_image
            self.camera_map = cv2.cvtColor(self.camera_map, cv2.COLOR_BGR2RGB)

            if self.pt1_check_box.isChecked() == True:
                cv2.circle(self.camera_map, (int(self.pt1_x), int(self.pt1_y)), 3, (0, 0, 255), -1)
                cv2.putText(self.camera_map, 'pt1', (int(self.pt1_x) + 5, int(self.pt1_y)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)

            if self.pt2_check_box.isChecked() == True:
                cv2.circle(self.camera_map, (int(self.pt2_x), int(self.pt2_y)), 3, (0, 0, 255), -1)
                cv2.putText(self.camera_map, 'pt2', (int(self.pt2_x) + 5, int(self.pt2_y)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA) 
            
            if self.pt3_check_box.isChecked() == True:
                cv2.circle(self.camera_map, (int(self.pt3_x), int(self.pt3_y)), 3, (0, 0, 255), -1) 
                cv2.putText(self.camera_map, 'pt3', (int(self.pt3_x) + 5, int(self.pt3_y)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)

            if self.pt4_check_box.isChecked() == True:
                cv2.circle(self.camera_map, (int(self.pt4_x), int(self.pt4_y)), 3, (0, 0, 255), -1)
                cv2.putText(self.camera_map, 'pt4', (int(self.pt4_x) + 5, int(self.pt4_y)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)

            if self.camera_x_line_check_box.isChecked() == True:
                for i in np.arange(-abs(self.lidar_y), abs(self.lidar_y), 0.1):
                    x, y = self.distance_to_pixel(self.line_x, i)
                    if x < w and y < h:
                        self.camera_map = cv2.circle(self.camera_map, (int(x), int(y)), 2, (255, 0, 0), -1)

            if self.camera_y_line_check_box.isChecked() == True:
                for i in np.arange(-abs(self.lidar_x), abs(self.lidar_x), 0.1):
                    x, y = self.distance_to_pixel(i, self.line_y)
                    if x < w and y < h:
                        self.camera_map = cv2.circle(self.camera_map, (int(x), int(y)), 2, (0, 255, 0), -1)
                        
            height, width, channels = np.shape(self.camera_map)
            totalBytes = self.camera_map.nbytes
            bytesPerLine = int(totalBytes / height)
            qimg = PySide2.QtGui.QImage(self.camera_map.data, self.camera_map.shape[1], self.camera_map.shape[0], bytesPerLine, PySide2.QtGui.QImage.Format_RGB888)
            pixmap = PySide2.QtGui.QPixmap.fromImage(qimg)
            self.camera_frame.setPixmap(pixmap)
            self.camera_frame.show()

        if self.lidar_msg is not None:
            self.lidar_map = np.zeros((self.h, self.w, 3), dtype=np.uint8)

            for p in pc2.read_points(self.lidar_msg):
                x, y, z, i, r = 0, 0, 0, 0, 0
                
                if len(p) == 4:
                    x, y, z, i = p

                elif len(p) == 5:
                    x, y, z, i, r = p

                resized_x = self.scale * x
                resized_y = self.scale * y

                if self.h - resized_x > 0 and self.h - resized_x < self.h and self.w/2 - resized_y > 0 and self.w/2 - resized_y < self.w:
                    if i > self.intensity:
                        self.lidar_map[int(self.h - resized_x)][int(self.w/2 - resized_y)][0] = 255
                        self.lidar_map[int(self.h - resized_x)][int(self.w/2 - resized_y)][1] = 0
                        self.lidar_map[int(self.h - resized_x)][int(self.w/2 - resized_y)][2] = 255
                    else:
                        self.lidar_map[int(self.h - resized_x)][int(self.w/2 - resized_y)][0] = 255
                        self.lidar_map[int(self.h - resized_x)][int(self.w/2 - resized_y)][1] = 255
                        self.lidar_map[int(self.h - resized_x)][int(self.w/2 - resized_y)][2] = 255
            
            if self.lidar_x_line_check_box.isChecked() == True:
                cv2.line(self.lidar_map, (0, int((self.lidar_x - self.line_x)*self.scale)), (int(self.w), int((self.lidar_x - self.line_x)*self.scale)), (255, 0, 0), 2)
            
            if self.lidar_y_line_check_box.isChecked() == True:
                cv2.line(self.lidar_map, (int((self.lidar_y - self.line_y)*self.scale), 0), (int((self.lidar_y - self.line_y)*self.scale), self.h), (0, 255, 0), 2)
            
            if self.pt1_p_check_box.isChecked() == True:
                cv2.circle(self.lidar_map, (int(self.w/2 - self.pt1_yp*self.scale), int(self.h - self.pt1_xp*self.scale)), 3, (0, 0, 255), -1) 
                cv2.putText(self.lidar_map, 'pt1\'', (int(self.w/2 - self.pt1_yp*self.scale) + 5, int(self.h - self.pt1_xp*self.scale)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
            
            if self.pt2_p_check_box.isChecked() == True:
                cv2.circle(self.lidar_map, (int(self.w/2 - self.pt2_yp*self.scale), int(self.h - self.pt2_xp*self.scale)), 3, (0, 0, 255), -1) 
                cv2.putText(self.lidar_map, 'pt2\'', (int(self.w/2 - self.pt2_yp*self.scale) + 5, int(self.h - self.pt2_xp*self.scale)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
            
            if self.pt3_p_check_box.isChecked() == True:
                cv2.circle(self.lidar_map, (int(self.w/2 - self.pt3_yp*self.scale), int(self.h - self.pt3_xp*self.scale)), 3, (0, 0, 255), -1) 
                cv2.putText(self.lidar_map, 'pt3\'', (int(self.w/2 - self.pt3_yp*self.scale) + 5, int(self.h - self.pt3_xp*self.scale)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
            
            if self.pt4_p_check_box.isChecked() == True:
                cv2.circle(self.lidar_map, (int(self.w/2 - self.pt4_yp*self.scale), int(self.h - self.pt4_xp*self.scale)), 3, (0, 0, 255), -1) 
                cv2.putText(self.lidar_map, 'pt4\'', (int(self.w/2 - self.pt4_yp*self.scale) + 5, int(self.h - self.pt4_xp*self.scale)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)

            self.lidar_frame.resize(self.h, self.w)
            height, width, channels = np.shape(self.lidar_map)
            totalBytes = self.lidar_map.nbytes
            bytesPerLine = int(totalBytes / height)
            qimg = PySide2.QtGui.QImage(self.lidar_map.data, self.lidar_map.shape[1], self.lidar_map.shape[0], bytesPerLine, PySide2.QtGui.QImage.Format_RGB888)
            pixmap = PySide2.QtGui.QPixmap.fromImage(qimg)
            self.lidar_frame.setPixmap(pixmap)
            self.lidar_frame.show()
            
        self.init_frame()

    def init_frame(self):
        self.camera_map = None
        self.lidar_map = None
        self.calibration_map = None

    def update(self):
        try:
            # update frames
            self.draw_frames()
        except Exception as e:
            print e
            pass
   
    def distance_to_pixel(self, x, y):
        pixel = np.float32([[self.pt1_x, self.pt1_y], [self.pt2_x, self.pt2_y], [self.pt3_x, self.pt3_y], [self.pt4_x, self.pt4_y]])
        distance = np.float32([[self.pt1_xp, self.pt1_yp], [self.pt2_xp, self.pt2_yp], [self.pt3_xp, self.pt3_yp], [self.pt4_xp, self.pt4_yp]])

        IM = cv2.getPerspectiveTransform(distance, pixel)

        IMa = IM[0, 0]
        IMb = IM[0, 1]
        IMc = IM[0, 2]
        IMd = IM[1, 0]
        IMe = IM[1, 1]
        IMf = IM[1, 2]
        IMg = IM[2, 0]
        IMh = IM[2, 1]
        IMi = IM[2, 2]
        
        xa = (IMa * x + IMb * y + IMc) / (IMg * x + IMh * y + 1)
        ya = (IMd * x + IMe * y + IMf) / (IMg * x + IMh * y + 1)

        return xa, ya

class ComboBox(PySide2.QtWidgets.QComboBox):
    popupAboutToBeShown = PySide2.QtCore.Signal()

    def showPopup(self):
        self.popupAboutToBeShown.emit()
        super(ComboBox, self).showPopup()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    form = Form('lidar_camera.ui')
    sys.exit(app.exec_())