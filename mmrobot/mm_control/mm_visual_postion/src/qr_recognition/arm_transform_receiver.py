#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import sys
import tf
import numpy as np
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class ArmTransformReceiver:
    def __init__(self):
        self.listener = tf.TransformListener()
    def getLatestTransformInMM(self):
        (trans1, rot1) = self.listener.lookupTransform(
                '/base', '/tool0_controller', rospy.Time(0))
        homo_T_base_to_endeffector = np.eye(4)
        homo_T_base_to_endeffector= tf.transformations.quaternion_matrix(rot1)
        homo_T_base_to_endeffector[0:3,3] = np.array(trans1) * 1000.0 # mm
        print("homo_T_base_to_endeffector", homo_T_base_to_endeffector)
        return homo_T_base_to_endeffector