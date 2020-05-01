#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, sys
import moveit_commander
import moveit_msgs.msg
from moveit_msgs.msg import RobotTrajectory,PlanningScene, ObjectColor
from moveit_commander import MoveGroupCommander, PlanningSceneInterface
from geometry_msgs.msg import PoseStamped, Pose
from copy import deepcopy

class MoveItCartesianDemo:
    def __init__(self):
        # 初始化move_group的API
        moveit_commander.roscpp_initialize(sys.argv)

        # 初始化ROS节点
        rospy.init_node('moveit_cartesian_demo', anonymous=True)
                        
        moveit_commander.roscpp_initialize(sys.argv)
        arm = moveit_commander.MoveGroupCommander('manipulator')       
	end_effector_link = arm.get_end_effector_link()               
	reference_frame = 'base_link'
	arm.set_pose_reference_frame(reference_frame)        
	arm.allow_replanning(True)
	arm.set_goal_position_tolerance(0.0001)
	arm.set_goal_orientation_tolerance(0.0001)

        # 初始化场景对象
        scene = PlanningSceneInterface()
        rospy.sleep(1) 
        self.scene_pub = rospy.Publisher('planning_scene', PlanningScene, queue_size=10)

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
        column1_size = column2_size = column3_size = column4_size = [0.045, 0.06, 0.88]
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
        box_pose.pose.position.y = 0 - table_size[1] / 2.0 + box_size[1] / 2.0
        box_pose.pose.position.z = 0 - box_size[2] / 2.0-0.001
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
        cabinet_pose.pose.position.x = -0.15
        cabinet_pose.pose.position.y = 0.85 + cabinet_size[1] / 2.0
        cabinet_pose.pose.position.z = 0 - box_size[2] - table_size[2] + cabinet_size[2] / 2.0
        cabinet_pose.pose.orientation.w = 1.0
        scene.add_box(cabinet_id, cabinet_pose, cabinet_size)
        rospy.sleep(1)

        # 复位待机
        #arm.set_named_target('start_pose')  
        #arm.go()                 
        #rospy.sleep(5)
                                        

        #joint_positions = [-1.79, -1.21, -1.73, -0.19, 2.03,0]
        #arm.set_joint_value_target(joint_positions)
        #arm.go()
        #rospy.sleep(5)

        # 获取当前位姿数据最为机械臂运动的起始位姿
        start_pose = arm.get_current_pose(end_effector_link).pose
                
        # 初始化路点列表
        waypoints = []
                
        # 将初始位姿加入路点列表
        
        waypoints.append(start_pose)
            
        # 设置第二个路点数据，并加入路点列表
        wpose = deepcopy(start_pose)
        wpose.position.x -= 0.2
        waypoints.append(deepcopy(wpose))

        # 设置第三个路点数据，并加入路点列表
        wpose.position.z -= 0.2
        waypoints.append(deepcopy(wpose))

        # 设置第四个路点数据，并加入路点列表
        #wpose.position.x += 0.2
        #waypoints.append(deepcopy(wpose))

        # 设置第五个路点数据，并加入路点列表
        #wpose.position.z += 0.2
        #waypoints.append(deepcopy(wpose))
            
 
        fraction = 0.0   #路径规划覆盖率
        maxtries = 100   #最大尝试规划次数
        attempts = 0     #已经尝试规划次数
            
        # 设置机器臂当前的状态作为运动初始状态
        arm.set_start_state_to_current_state()
     
        # 尝试规划一条笛卡尔空间下的路径，依次通过所有路点
        while fraction < 1.0 and attempts < maxtries:
            (plan, fraction) = arm.compute_cartesian_path (
                                    waypoints,   # waypoint poses，路点列表
                                    0.01,        # eef_step，终端步进值
                                    0.0,         # jump_threshold，跳跃阈值
                                    True)        # avoid_collisions，避障规划
                
            # 尝试次数累加
            attempts += 1
                
            # 打印运动规划进程
            if attempts % 10 == 0:
                rospy.loginfo("Still trying after " + str(attempts) + " attempts...")
                         
        # 如果路径规划成功（覆盖率100%）,则开始控制机械臂运动
        if fraction == 1.0:
            rospy.loginfo("Path computed successfully. Moving the arm.")
            arm.execute(plan)
            rospy.loginfo("Path execution complete.")
        # 如果路径规划失败，则打印失败信息
        else:
            rospy.loginfo("Path planning failed with only " + str(fraction) + " success after " + str(maxtries) + " attempts.")  

        # 控制机械臂回到初始化位置
        #arm.set_named_target('start_pose')
        #arm.go()
        #rospy.sleep(1)
        
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
        MoveItCartesianDemo()
    except rospy.ROSInterruptException:
        pass
