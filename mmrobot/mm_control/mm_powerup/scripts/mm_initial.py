#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import sys
import tf
from mm_robot_decision.msg import pose4
from std_msgs.msg import String, UInt8
import socket 
import os
from youibot_msgs.msg import Battery
import Queue



bmsq = Queue.Queue()


def robotinitialize():
    bms_sub = rospy.Subscriber('/bms_data', Battery, bmscallback)
    ini_pub = rospy.Publisher('/mm_initialize/state', String, queue_size=10)
    power_pub = rospy.Publisher('/mm_powerup/switch_command', UInt8, queue_size=10)   
    HOST = '192.168.0.11'  # 服务器的主机名或者 IP 地址
    PORT = 29999        # 服务器使用的端口
     
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)  
   
    if not bmsq.get():
        ini_pub.publish(String("lowBattery"))
    else:
        pass
    bms_sub.unregister() 
    rospy.sleep(1)

#    try:
#        s.connect((HOST, PORT))
#        data = s.recv(1024)
#      	s.close()  
#	print data
#    except socket.error:
    power_pub.publish(UInt8(1))   

    

    while not rospy.is_shutdown():
        try:
            s.connect((HOST, PORT))
            data = s.recv(1024)
            print('Received', repr(data)) 
            break   
        except socket.error:
            print("wait to connect")
            rospy.sleep(5)     
    rospy.sleep(10)


    while not rospy.is_shutdown():
        s.sendall('robotmode\n')
        data = s.recv(1024)
        print(data, len(data)) 
        if data == 'Robotmode: RUNNING\n':
            break
    	power_pub.publish(UInt8(2))  
        rospy.sleep(5)

    s.close()
    ini_pub.publish(String("initialized"))  
      
    os.system('roslaunch mm_arm_control ur10_test_run.launch')


def bmscallback(bmsinfo):
    print(bmsinfo.rsoc)
    if bmsinfo.rsoc < 30:
        bmsq.put(False)
    else:
        bmsq.put(True)
    rospy.sleep(2)
               
    
 


if __name__ == '__main__':
    rospy.init_node('robot_initialize',anonymous=True)
    robotinitialize()

