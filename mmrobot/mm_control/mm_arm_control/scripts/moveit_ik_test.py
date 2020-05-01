#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import division
import rospy
import sys
import tf
import math
import numpy as np
from trajectory_msgs.msg import JointTrajectoryPoint
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import PoseStamped, Pose
from mm_robot_decision.msg import pose4
from copy import deepcopy
from std_msgs.msg import String, Int8
from scipy.spatial.transform import Rotation #it need scipy version >= 1.2.1
from threading import Lock

import rospkg
from os.path import join
# 0307更新说明：重大更新，删除了moveit部分

# 定义动作完成的状态发布
over_pub = rospy.Publisher('/mm_arm/isArrived', Int8, queue_size=10)

# URscript publisher
string_pub = rospy.Publisher('/ur_driver/URScript', String, queue_size=10)

def loadOffsetEndeffectorToToolEnd():
    import cv2
    # change the value in the yaml,
    # DO NOT change this value directly in this python script
    rospack = rospkg.RosPack()
    path = rospack.get_path('mm_visual_postion')
    path = join(path, "endeffector_to_tool_length.yaml")
    fs = cv2.FileStorage(path, cv2.FILE_STORAGE_READ)
    return fs.getNode("endeffector_to_tool").real()
    
offset_endeffector_to_tool_end = loadOffsetEndeffectorToToolEnd()


def transform2NpMatrix(transform):
    '''
    :param transform: (tf transform (trans, q))
    :return np.ndarray (4x4 homogenous transformation matrix)
    '''
    homo_T = np.eye(4)
    translate, q = transform
    homo_T= tf.transformations.quaternion_matrix(q)
    homo_T[0:3,3] = translate

    return homo_T

def transEuler2NpMatrix(x, y, z, euler_x, euler_y, euler_z):
    """
    :param x: (float) translation in x
    :param y: (float) translation in y
    :param z: (float) translation in z
    :param euler_x: (float) euler angle in x-axis (XYZ order)
    :param euler_y: (float) translation in y-axis
    :param euler_z: (float) translation in z-axis
    :return np.ndarray (4x4 homogenous transformation matrix)
    """
    homo_T = np.eye(4)  
    homo_T = tf.transformations.euler_matrix(euler_x, euler_y, euler_z, 'rxyz')
    homo_T[0,3] = x
    homo_T[1,3] = y
    homo_T[2,3] = z

    return homo_T

def transRot2NpHomoMatrix(x, y, z, rot_mat):
    """
    :param x: (float) translation in x
    :param y: (float) translation in y
    :param z: (float) translation in z
    :param rot_mat: (np.ndarray) (3x3 rotation matrix)
    :return np.ndarray (4x4 homogenous transformation matrix)
    """
    homo_T = np.eye(4)  
    homo_T[0:3,0:3] = rot_mat
    homo_T[0,3] = x
    homo_T[1,3] = y
    homo_T[2,3] = z

    return homo_T
    


def equalTransformMatrix(T_1, T_2):
    '''
    :param T_1: (np.ndarray 4x4) homogenous transfromation matrix
    :param T_2: (np.ndarray 4x4) another homogenous transfromation matrix
    '''
    diff = np.max(np.abs(T_1 - T_2))
    #print("max diff: ",diff)
    if diff < 0.001: 
        return True
    else:
        return False



class MoveItIkTest:

    def __init__(self):

        # 配置信息
        # 初始化ROS节点
        rospy.init_node('moveit_ik_test')
        rospy.loginfo('started')
        self.listener = tf.TransformListener()
        self.new_cmd_arrived = False
        self.cmd_timestamp = rospy.Time.now()
        self.cmd = None
        self.mutex = Lock()
        # 订阅位姿信息
        rospy.Subscriber('/mm_arm/goal', pose4, self.callback)
        
        r = rospy.Rate(10) # 10hz 
        while not rospy.is_shutdown():
            self.mutex.acquire()
            if self.new_cmd_arrived:
                self.new_cmd_arrived = False
                self.mutex.release()

                self.excuteOrder()
            else:
                self.mutex.release()
            r.sleep()   

        rospy.spin()




    def waitArrived(self, base_tool0_controller_T_cmd):
        # base_tool0_controller_T_cmd: type np.ndarray of size 4x4, which indicate the target transformation (command) from base to tool0_controller

        while not rospy.is_shutdown():
            if self.new_cmd_arrived:
                print("New command arrives, aborts the old one")
                return False

            (trans1, rot1) = self.listener.lookupTransform(
                '/base', '/tool0_controller', rospy.Time(0))
            curr_T = transform2NpMatrix((trans1, rot1))
            # check curr_T == base_tool0_controller_T_cmd
            #print("curr_T\n", curr_T)
            #print("base_tool0_controller_T_cmd\n", base_tool0_controller_T_cmd)
            if(equalTransformMatrix(curr_T, base_tool0_controller_T_cmd)):
                rospy.sleep(0.5)
                (trans2, rot2) = self.listener.lookupTransform(
                    '/base', '/tool0_controller', rospy.Time(0))
                if np.max(np.abs(np.array(trans2) - np.array(trans1))) < 0.001 \
                    and np.max(np.abs(np.array(rot1) - np.array(rot2))) < 0.0001:
                    print("arrived")
                    rospy.sleep(0.5)
                    break #arrived
            rospy.sleep(0.2)
        return True

    def callback(self, pose4):
        self.mutex.acquire()

        self.cmd = pose4
        self.cmd_timestamp = rospy.Time.now()
        self.new_cmd_arrived = True
        
        self.mutex.release()
        # 七种控制指令，四种运动模式＋两种待机位+退出
        # 1. WorkSpacePlanning，相机（工具）坐标系，相机拍摄后两步运动到位套进旋钮
        # 2. jointSpacePlanning，关节坐标系，关节空间规划
        # 3. MoveIntool，工具坐标系，工具坐标系中直接运动
        # 4. BaseSpacePlanning,基座坐标系，工作空间规划
        # 5. StandBy，取工具准备状态
        # 6. BackHome，打包状态
        # 7. shutdown，退出，重启需重新运行本脚本
    def excuteOrder(self):
        pose4 = self.cmd
        if pose4.state == 'WorkSpacePlanning':

            # 输入的姿态四元数的旋转矢量表示法的转化
            norm = abs(math.sqrt(pose4.a*pose4.a+pose4.b*pose4.b+pose4.c*pose4.c+pose4.w*pose4.w))
            r2q = [pose4.a/norm, pose4.b/norm, pose4.c/norm, pose4.w/norm]
            theta = math.acos(r2q[3])
            if theta == 0.0:
                rec = [0.0, 0.0, 0.0]
            else:
                rec = [r2q[0]*2*theta/math.sin(theta), r2q[1]*2*theta /
                       math.sin(theta), r2q[2]*2*theta/math.sin(theta)]

            # 采用urscript直接发布，先到达过渡位置，再沿工具坐标系z轴前进到达目标位置（防碰处理）
            pose_format = '''def myProg():
  p_a1=get_actual_tcp_pose()
  offset1=p[%s,%s,%s,%s,%s,%s]
  p_g1=pose_trans(p_a1,offset1)
  offset2=p[0,0,%s,0,0,0]
  p_g2=pose_trans(p_g1,offset2)
  movej(p_g2,a=0.2,v=0.1,r=0)
  sleep(0.2)
  offset3=p[0,0,0.1,0,0,0]
  p_g3=pose_trans(p_g2,offset3)
  movel(p_g3,a=0.06,v=0.03,r=0)
  sleep(0.2)
end'''




            self.listener.waitForTransform(
                "/base", "/tool0_controller", rospy.Time(0), rospy.Duration(4.0))
            (trans1, rot1) = self.listener.lookupTransform(
                '/base', '/tool0_controller', rospy.Time(0))
            T_curr = transform2NpMatrix((trans1, rot1))

            
            values = (("%.5f" % pose4.x), ("%.5f" % pose4.y), ("%.5f" % pose4.z),
                  ("%.5f" % rec[0]), ("%.5f" % rec[1]), ("%.5f" % rec[2]), (offset_endeffector_to_tool_end - 0.1))
            true_pose = pose_format % values
            print(pose4.x, pose4.y, pose4.z, rec[0], rec[1], rec[2])
            string_pub.publish(String(true_pose))

            # 监听机械臂反馈位姿信息，判断是否到位，发布状态信息
            r_cmd_mat = Rotation.from_quat([pose4.a, pose4.b, pose4.c, pose4.w]).as_dcm()

            T_move = transRot2NpHomoMatrix(pose4.x, pose4.y, pose4.z, r_cmd_mat)
            T_offset = transRot2NpHomoMatrix(0,0,offset_endeffector_to_tool_end, np.eye(3))
            T_cmd =   np.matmul(np.matmul(T_curr, T_move ), T_offset)
            rospy.loginfo("wait for arm to arrive the target pose")
            if not self.waitArrived(T_cmd): 
                return

            rpy = tf.transformations.euler_from_quaternion(
                [rot1[0], rot1[1], rot1[2], rot1[3]])
            rospy.loginfo('Actual pose： \n x=%f ,y=%f ,z=%f ,r=%f ,p=%f ,y=%f',
                          trans1[0], trans1[1], trans1[2], rpy[0], rpy[1], rpy[2])
            #rospy.sleep(0.5)


        elif pose4.state == 'jointSpacePlanning':

            # urscript,谨慎使用，必须先测试
            pose_format = 'movej([%s,%s,%s,%s,%s,%s],a=0.8,v=0.4,r=0)'
            values = (("%.5f" % pose4.x), ("%.5f" % pose4.y), ("%.5f" % pose4.z),
                      ("%.5f" % pose4.a), ("%.5f" % pose4.b), ("%.5f" % pose4.c))
            true_pose = pose_format % values
            rospy.loginfo(true_pose)
            string_pub.publish(String(true_pose))

            self.listener.waitForTransform(
                "/base", "/tool0_controller", rospy.Time(), rospy.Duration(4.0))
            (trans1, rot1) = self.listener.lookupTransform(
                '/base', '/tool0_controller', rospy.Time(0))
            rospy.sleep(0.2)
            (trans2, rot2) = self.listener.lookupTransform(
                '/base', '/tool0_controller', rospy.Time(0))
            while abs(trans2[0]-trans1[0]) > 0.0001 or abs(trans2[1]-trans1[1]) > 0.0001 or abs(trans2[2]-trans1[2]) > 0.0001:
                (trans1, rot1) = self.listener.lookupTransform(
                    '/base', '/tool0_controller', rospy.Time(0))
                rospy.sleep(0.2)
                (trans2, rot2) = self.listener.lookupTransform(
                    '/base', '/tool0_controller', rospy.Time(0))


        elif pose4.state == 'MoveInTool':
            # 用于直接在当前位置工具坐标系中运动，urscript。如后退100mm，则发布0,0,-0.1,0,0,0,0
            # 输入的姿态rpy的旋转矢量表示法的转化,用于urscript
            norm = abs(math.sqrt(pose4.a*pose4.a+pose4.b*pose4.b+pose4.c*pose4.c+pose4.w*pose4.w))
            r2q = [pose4.a/norm, pose4.b/norm, pose4.c/norm, pose4.w/norm]
            theta = math.acos(r2q[3])
            if theta == 0.0:
                rec = [0.0, 0.0, 0.0]
            else:
                rec = [r2q[0]*2*theta/math.sin(theta), r2q[1]*2*theta /
                       math.sin(theta), r2q[2]*2*theta/math.sin(theta)]

            pose_format = '''def myProg():
  p_a=get_actual_tcp_pose()
  offset=p[%s,%s,%s,%s,%s,%s]
  p_g=pose_trans(p_a,offset)
  movel(p_g,a=0.15,v=0.075,r=0)
  sleep(0.2)
end'''
            values = (("%.5f" % pose4.x), ("%.5f" % pose4.y), ("%.5f" % pose4.z),
                      ("%.5f" % rec[0]), ("%.5f" % rec[1]), ("%.5f" % rec[2]))

            true_pose = pose_format % values
            rospy.loginfo(true_pose)

            self.listener.waitForTransform(
                "/base", "/tool0_controller", rospy.Time(0), rospy.Duration(4.0))
            (trans1, rot1) = self.listener.lookupTransform(
                '/base', '/tool0_controller', rospy.Time(0))
            T_curr = transform2NpMatrix((trans1, rot1))
            r = Rotation.from_quat([pose4.a, pose4.b, pose4.c, pose4.w])
            T_move = transRot2NpHomoMatrix(pose4.x, pose4.y, pose4.z, r.as_dcm())
            T_cmd = np.matmul(T_curr, T_move)
            #rospy.sleep(0.5)
            string_pub.publish(String(true_pose))
            rospy.loginfo("wait for arm to arrive the target pose")
            if not self.waitArrived(T_cmd): 
                return
            estate = 'done'
            rospy.loginfo(estate)
            

        elif pose4.state == 'BaseSpacePlanning':

            # urscript,谨慎使用，必须先测试，姿态角使用轴角表示法，用于固定的示教拍照位置
            # meter
            pose_format = 'movel(p[%s,%s,%s,%s,%s,%s],a=0.2,v=0.1,r=0)'
            values = (("%.5f" % pose4.x), ("%.5f" % pose4.y), ("%.5f" % pose4.z),
                      ("%.5f" % pose4.a), ("%.5f" % pose4.b), ("%.5f" % pose4.c))
            true_pose = pose_format % values
            rospy.loginfo(true_pose)
            #rospy.sleep(0.5)
            string_pub.publish(String(true_pose))

            # compute command transform matrix
            r = Rotation.from_quat([pose4.a, pose4.b, pose4.c, pose4.w])
            T_cmd = transRot2NpHomoMatrix(pose4.x, pose4.y, pose4.z, r.as_dcm())
            rospy.loginfo("wait for arm to arrive the target pose")
            if not self.waitArrived(T_cmd): 
                return



#        elif pose4.state == 'StandBy':

#            pose_format = '''def myProg():
#  movej([%s,%s,%s,%s,%s,%s],a=0.5,v=0.25,r=0)
#  movej([%s,%s,%s,%s,%s,%s],a=0.5,v=0.25,r=0)
#end'''
#            values = (("%.5f" % (202.26*3.14159/180)), ("%.5f" % (-45.21*3.14159/180)), ("%.5f" % (-126.66*3.14159/180)), ("%.5f" % (-7.56*3.14159/180)), ("%.5f" % (-34.61*3.14159/180)), ("%.5f" % (0.09*3.14159/180)),
#                      ("%.5f" % (101.99*3.14159/180)), ("%.5f" % (-99.09*3.14159/180)), ("%.5f" % (-137.70*3.14159/180)), ("%.5f" % (-32.73*3.14159/180)), ("%.5f" % (89.59*3.14159/180)), ("%.5f" % (12.11*3.14159/180)))

#            true_pose = pose_format % values
#            rospy.loginfo(true_pose)
#            rospy.sleep(0.5)
#            string_pub.publish(String(true_pose))

#            self.listener.waitForTransform(
#                "/base", "/tool0_controller", rospy.Time(), rospy.Duration(4.0))
#            (trans1, rot1) = self.listener.lookupTransform(
#                '/base', '/tool0_controller', rospy.Time(0))
#            rospy.sleep(0.2)
#            (trans2, rot2) = self.listener.lookupTransform(
#                '/base', '/tool0_controller', rospy.Time(0))
#            while abs(trans2[0]-trans1[0]) > 0.0001 or abs(trans2[1]-trans1[1]) > 0.0001 or abs(trans2[2]-trans1[2]) > 0.0001:
#                (trans1, rot1) = self.listener.lookupTransform(
#                    '/base', '/tool0_controller', rospy.Time(0))
#                rospy.sleep(0.2)
#                (trans2, rot2) = self.listener.lookupTransform(
#                    '/base', '/tool0_controller', rospy.Time(0))


#        elif pose4.state == 'BackHome':

#            pose_format = '''def myProg():
#  movej([%s,%s,%s,%s,%s,%s],a=0.5,v=0.25,r=0)
#  movej([%s,%s,%s,%s,%s,%s],a=0.5,v=0.25,r=0)
#end'''
#            values = (("%.5f" % (202.26*3.14159/180)), ("%.5f" % (-45.21*3.14159/180)), ("%.5f" % (-126.66*3.14159/180)), ("%.5f" % (-7.56*3.14159/180)), ("%.5f" % (-34.61*3.14159/180)), ("%.5f" % (0.09*3.14159/180)),
#                      ("%.5f" % (267.22*3.14159/180)), ("%.5f" % (10.96*3.14159/180)), ("%.5f" % (-163.66*3.14159/180)), ("%.5f" % (-28.24*3.14159/180)), ("%.5f" % (-87.22*3.14159/180)), ("%.5f" % (0.22*3.14159/180)))

#            true_pose = pose_format % values
#            rospy.loginfo(true_pose)
#            rospy.sleep(0.5)
#            string_pub.publish(String(true_pose))

#            self.listener.waitForTransform(
#                "/base", "/tool0_controller", rospy.Time(), rospy.Duration(4.0))
#            (trans1, rot1) = self.listener.lookupTransform(
#                '/base', '/tool0_controller', rospy.Time(0))
#            rospy.sleep(0.2)
#            (trans2, rot2) = self.listener.lookupTransform(
#                '/base', '/tool0_controller', rospy.Time(0))
#            while abs(trans2[0]-trans1[0]) > 0.0001 or abs(trans2[1]-trans1[1]) > 0.0001 or abs(trans2[2]-trans1[2]) > 0.0001:
#                (trans1, rot1) = self.listener.lookupTransform(
#                    '/base', '/tool0_controller', rospy.Time(0))
#                rospy.sleep(0.2)
#                (trans2, rot2) = self.listener.lookupTransform(
#                    '/base', '/tool0_controller', rospy.Time(0))



#        # 位姿输入节点发出关闭指令
#        elif pose4.state == 'shutdown':

#            exit(0)

        isArrived_msg = 1
        over_pub.publish(isArrived_msg)


if __name__ == "__main__":
    try:
        MoveItIkTest()
    except rospy.ROSInterruptException:
        pass
