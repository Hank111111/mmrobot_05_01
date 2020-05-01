#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, sys
import math
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler


def rpy2rec():
    rospy.init_node('rpy2rec')
    rospy.loginfo('started')
    rpy = [-1.421229,0.134953,2.479898]
    rospy.loginfo('rpy: %s\n', rpy)
    q = tf.transformations.quaternion_from_euler(rpy[0],rpy[1],rpy[2])
    rospy.loginfo('quaternion: %s\n', q)
    theta = math.acos(q[3])
    rec = [q[0]*2*theta/math.sin(theta),q[1]*2*theta/math.sin(theta),q[2]*2*theta/math.sin(theta)]
    rospy.loginfo('rec: %s\n', rec)
    
if __name__ == "__main__":
    rpy2rec()
    
