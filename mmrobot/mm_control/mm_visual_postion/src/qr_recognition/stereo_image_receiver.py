#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import division, print_function
import roslib
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from stereo_camera_arm_model import StereoCameraArmModel
class StereoImageReceiver():
    def __init__(self):
        self.model = StereoCameraArmModel()
        self.model.loadDefaultParams()
    
    def getLatestRawImages(self):
        left_image_msg = rospy.wait_for_message("/zed/left/image_raw_color", Image)
        right_image_msg = rospy.wait_for_message("/zed/right/image_raw_color", Image)
        try:
            bridge = CvBridge()
            cv_left_image = bridge.imgmsg_to_cv2(left_image_msg, "passthrough")
            cv_right_image = bridge.imgmsg_to_cv2(right_image_msg, "passthrough")

            return cv_left_image, cv_right_image
        except CvBridgeError as e:
            print(e)

    def getLatestImages(self):
        left_image_msg = rospy.wait_for_message("/zed/left/image_raw_color", Image)
        right_image_msg = rospy.wait_for_message("/zed/right/image_raw_color", Image)
        try:
            bridge = CvBridge()
            cv_left_image = bridge.imgmsg_to_cv2(left_image_msg, "passthrough")
            cv_right_image = bridge.imgmsg_to_cv2(right_image_msg, "passthrough")

            left_undistort_image = self.model.rectifyLeftImage(cv_left_image)
            right_undistort_image = self.model.rectifyRightImage(cv_right_image)
            return left_undistort_image, right_undistort_image
        except CvBridgeError as e:
            print(e)