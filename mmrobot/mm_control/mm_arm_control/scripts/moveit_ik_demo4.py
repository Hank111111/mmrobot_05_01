#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, sys
import moveit_commander
import tf
from tf.transformations import *
from moveit_msgs.msg import RobotTrajectory, PlanningScene, ObjectColor
from trajectory_msgs.msg import JointTrajectoryPoint
from moveit_commander import MoveGroupCommander, PlanningSceneInterface
from geometry_msgs.msg import PoseStamped, Pose
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from test_planning.msg import runstate
from std_msgs.msg import String


class MoveItIkDemo:
         
    def __init__(self):
        
        # 初始化move_group的API
        moveit_commander.roscpp_initialize(sys.argv)
        
        # 初始化ROS节点
        rospy.init_node('moveit_ik_demo')
        rospy.loginfo('started')
        self.state_pub = rospy.Publisher('excuted_state', runstate , queue_size=10)
        self.string_pub = rospy.Publisher('/ur_driver/URScript', String , queue_size=10)

        # 初始化场景对象
        scene = PlanningSceneInterface()
        rospy.sleep(1)
        self.scene_pub = rospy.Publisher('planning_scene', PlanningScene, queue_size=10)
        
        # 初始化需要使用move group控制的机械臂中的manipulator group
        arm = moveit_commander.MoveGroupCommander('manipulator')
                
        # 获取终端link的名称
        end_effector_link = arm.get_end_effector_link()
                        
        # 设置目标位置所使用的参考坐标系
        reference_frame = 'base'
        arm.set_pose_reference_frame(reference_frame)
                
        # 当运动规划失败后，允许重新规划
        arm.allow_replanning(True)
        
        # 设置位置(单位：米)和姿态（单位：弧度）的允许误差
        arm.set_goal_position_tolerance(0.0001)
        arm.set_goal_orientation_tolerance(0.0001)

        # 设置场景物体的名称
        table_id = 'table'
        box_id='box'
        column1_id = 'column1'
        column2_id = 'column2'
        column3_id = 'column3'
        column4_id = 'column4'
        cabinet_id = 'cabinet'
        
        # 移除场景中之前运行残留的物体
        scene.remove_world_object(table_id)
        scene.remove_world_object(box_id)
        scene.remove_world_object(column1_id) 
        scene.remove_world_object(column2_id) 
        scene.remove_world_object(column3_id) 
        scene.remove_world_object(column4_id) 
        scene.remove_world_object(cabinet_id)   
        rospy.sleep(1)
       
        # 设置障碍的三维尺寸
        table_size = [1.2, 0.7, 0.65]
        box_size = [0.2, 0.78, 0.038]
        column1_size = column2_size = column3_size = column4_size = [0.045, 0.06, 0.86]
        cabinet_size = [1, 0.6, 2.5]

        # 设置颜色
        self.colors = dict()
        self.setColor(table_id, 0.9, 0.9, 0.9, 1.0)
        self.setColor(box_id, 0.9, 0.9, 0.9, 1.0)
        self.setColor(column1_id, 0.9, 0.9, 0.9, 1.0)
        self.setColor(column2_id, 0.9, 0.9, 0.9, 1.0)
        self.setColor(column3_id, 0.9, 0.9, 0.9, 1.0)
        self.setColor(column4_id, 0.9, 0.9, 0.9, 1.0)
        self.setColor(cabinet_id, 0.9, 0.9, 0.8, 1.0)
        self.sendColors()
        
        # 将障碍物体加入场景当中
        table_pose = PoseStamped()
        table_pose.header.frame_id = reference_frame
        table_pose.pose.position.x = 0
        table_pose.pose.position.y = 0
        table_pose.pose.position.z = 0 - box_size[2] - table_size[2] / 2.0
        table_pose.pose.orientation.w = 1.0
        scene.add_box(table_id, table_pose, table_size)

        box_pose = PoseStamped()
        box_pose.header.frame_id = reference_frame
        box_pose.pose.position.x = 0
        box_pose.pose.position.y = 0 + table_size[1] / 2.0 - box_size[1] / 2.0
        box_pose.pose.position.z = 0 - box_size[2] / 2.0 -0.001
        box_pose.pose.orientation.w = 1.0
        scene.add_box(box_id, box_pose, box_size)

        column1_pose = PoseStamped()
        column1_pose.header.frame_id = reference_frame
        column1_pose.pose.position.x = 0 + table_size[0] / 2.0 - column1_size[0] / 2.0
        column1_pose.pose.position.y = 0 + table_size[1] / 2.0 + column1_size[1] / 2.0
        column1_pose.pose.position.z = 0 - box_size[2] - table_size[2] + column1_size[2] / 2.0
        column1_pose.pose.orientation.w = 1.0
        scene.add_box(column1_id, column1_pose, column1_size)
 
        column2_pose = PoseStamped()
        column2_pose.header.frame_id = reference_frame
        column2_pose.pose.position.x = 0 + table_size[0] / 2.0 - column2_size[0] / 2.0
        column2_pose.pose.position.y = 0 - table_size[1] / 2.0 - column2_size[1] / 2.0
        column2_pose.pose.position.z = 0 - box_size[2] - table_size[2] + column2_size[2] / 2.0
        column2_pose.pose.orientation.w = 1.0
        scene.add_box(column2_id, column2_pose, column2_size)

        column3_pose = PoseStamped()
        column3_pose.header.frame_id = reference_frame
        column3_pose.pose.position.x = 0 - table_size[0] / 2.0 + column3_size[0] / 2.0
        column3_pose.pose.position.y = 0 - table_size[1] / 2.0 - column3_size[1] / 2.0
        column3_pose.pose.position.z = 0 - box_size[2] - table_size[2] + column3_size[2] / 2.0
        column3_pose.pose.orientation.w = 1.0
        scene.add_box(column3_id, column3_pose, column3_size)

        column4_pose = PoseStamped()
        column4_pose.header.frame_id = reference_frame
        column4_pose.pose.position.x = 0 - table_size[0] / 2.0 + column4_size[0] / 2.0
        column4_pose.pose.position.y = 0 + table_size[1] / 2.0 + column4_size[1] / 2.0
        column4_pose.pose.position.z = 0 - box_size[2] - table_size[2] + column4_size[2] / 2.0
        column4_pose.pose.orientation.w = 1.0
        scene.add_box(column4_id, column4_pose, column4_size)

        cabinet_pose = PoseStamped()
        cabinet_pose.header.frame_id = reference_frame
        cabinet_pose.pose.position.x = -0.12
        cabinet_pose.pose.position.y = -0.9 - cabinet_size[1] / 2.0
        cabinet_pose.pose.position.z = 0 - box_size[2] - table_size[2] + cabinet_size[2] / 2.0
        cabinet_pose.pose.orientation.w = 1.0
        scene.add_box(cabinet_id, cabinet_pose, cabinet_size)
        rospy.sleep(1)


        # 控制机械臂先回到standby位置
        #arm.set_named_target('start_pose')
        #arm.go()
        #rospy.sleep(2)

        # 设置机械臂工作空间中的目标位姿，位置使用x、y、z坐标描述，
        # 姿态使用四元数描述
        target_pose = PoseStamped()
        target_pose.header.frame_id = reference_frame
        target_pose.header.stamp = rospy.Time.now()     
        target_pose.pose.position.x = -0.3924
        target_pose.pose.position.y = -0.5414
        target_pose.pose.position.z = 0.8719
        point = tf.transformations.quaternion_from_euler(0,0,-1.57)
        target_pose.pose.orientation.x = point[0]
        target_pose.pose.orientation.y = point[1]
        target_pose.pose.orientation.z = point[2]
        target_pose.pose.orientation.w = point[3]
        rospy.loginfo('get goal:\n%s', target_pose.pose)
        estate = 'moving'
        self.state_pub.publish(runstate(estate))
        
        
        true_pose = 'movel(p[-0.15,0.118,0.655,1.23,2.53,-2.64],a=0.1,v=0.1,r=0)'
        
        

  
        # 设置机器臂当前的状态作为运动初始状态
        arm.set_start_state_to_current_state()
        
        # 设置机械臂终端运动的目标位姿
        arm.set_pose_target(target_pose, end_effector_link)
        
        # 规划运动路径并执行
        #traj = arm.plan()
        #arm.execute(traj)

      
        rospy.sleep(3)
        estate = 'compensating'
        self.state_pub.publish(runstate(estate))
        rospy.loginfo(estate)
        rospy.sleep(1)
        self.string_pub.publish(String(true_pose))
 
        #判断是否到位
        listener = tf.TransformListener()
        listener.waitForTransform("/base", "/tool0_controller", rospy.Time(), rospy.Duration(4.0))
        (trans,rot) = listener.lookupTransform('/base', '/tool0_controller', rospy.Time(0))
        while abs(target_pose.pose.position.x-trans[0])>0.0001 or abs(target_pose.pose.position.y-trans[1])>0.0001 or abs(target_pose.pose.position.z-trans[2])>0.0001:
           (trans,rot) = listener.lookupTransform('/base', '/tool0_controller', rospy.Time(0))
           rospy.sleep(1)
        rpy = tf.transformations.euler_from_quaternion([rot[0],rot[1],rot[2],rot[3]])
        rospy.loginfo('Actual pose： \n x=%f ,y=%f ,z=%f ,r=%f ,p=%f ,y=%f',trans[0],trans[1],trans[2],rpy[0],rpy[1],rpy[2]) 
        estate = 'done'
        rospy.loginfo(estate)
        self.state_pub.publish(runstate(estate))
        rospy.sleep(1)    


           
        # 控制机械臂回到standby位置
        #arm.set_named_target('start_pose')
        #arm.go()


        # 关闭并退出moveit
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)

    # 设置颜色
    def setColor(self,name,r,g,b,a=0.9):
        color = ObjectColor()
        color.id =name
        color.color.r = r
        color.color.g = g
        color.color.b = b
        color.color.a = a
        self.colors[name] = color

    # 颜色发布
    def sendColors(self):
        p = PlanningScene()
        p.is_diff = True
        for color in self.colors.values():
            p.object_colors.append(color)
        self.scene_pub.publish(p)

        

if __name__ == "__main__":
    try:
        MoveItIkDemo()
    except rospy.ROSInterruptException:
        pass
    
