#!/usr/bin/env python
#coding=utf-8
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that listens to std_msgs/Strings published 
## to the 'chatter' topic



import rospy
import math
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler
#from std_msgs.msg import String 

def jointlistener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.

    rospy.init_node('tflistener', anonymous=True)
    listener = tf.TransformListener()
    print '1. 阻塞直到frame相通'
    listener.waitForTransform("/base", "/tool0_controller", rospy.Time(), rospy.Duration(4.0))
    while not rospy.is_shutdown():
        try:
             print'2. 监听对应的tf，返回平移和旋转'
             (trans,rot) = listener.lookupTransform('/base', '/tool0_controller', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
             continue
        rpy = tf.transformations.euler_from_quaternion([rot[0],rot[1],rot[2],rot[3]])
        rospy.loginfo('x=%f ,y=%f ,z=%f',trans[0],trans[1],trans[2]) 
        rospy.loginfo('x=%f ,y=%f ,z=%f ,w=%f',rot[0],rot[1],rot[2],rot[3]) 
        rospy.loginfo('r=%f ,p=%f ,y=%f',rpy[0],rpy[1],rpy[2]) 
        rospy.sleep(3)
  
if __name__ == '__main__':
    jointlistener()


