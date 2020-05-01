#!/usr/bin/env python
# -*- coding: UTF-8 -*-
import rospy
import sys
from socket import *
import tf
import numpy as np
from mm_visual_postion.msg import EquipmentPose
import json
class EquipmentPoseSender():
    def __init__(self):
        rospy.init_node("equipment_pose_sender",anonymous=True)
        IP_PROT = ('', 12397)
        
        #IP_PROT = ('10.161.8.142', 9995)
        self.ss = socket(AF_INET, SOCK_STREAM)
        self.ss.setsockopt(SOL_SOCKET, SO_REUSEADDR, 1)
        self.ss.bind(IP_PROT)
        self.ss.listen(5)
        self.conn, self.addr = self.ss.accept()
        data = None
        while data != "Ready to receive" and not rospy.is_shutdown():
            data = self.conn.recv(1024)
            print("received msg: ", data)
            rospy.sleep(1.0)
        if not rospy.is_shutdown():
            rospy.Subscriber('mm_visual/wrapper/result_cabinet_pose', EquipmentPose, self.callback)
            self.conn.send('ready')
            print("ready to send")
        
    def callback(self, msg):
        rpy = tf.transformations.euler_from_quaternion(msg.pose.orientation)
        data = {"equipment_name":msg.equipment_name, "x": msg.pose.position.x, "y": msg.pose.position.y, "theta": rpy[2] * 180.0/np.pi}
        data_string = json.dumps(data) #data serialized
        print(data_string)
        self.conn.send(data_string)

if __name__ == "__main__":
    sender = EquipmentPoseSender()
    rospy.spin()
