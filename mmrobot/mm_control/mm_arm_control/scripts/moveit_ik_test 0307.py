#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, sys
import moveit_commander
import tf
import math
from moveit_msgs.msg import RobotTrajectory, PlanningScene, ObjectColor
from trajectory_msgs.msg import JointTrajectoryPoint
from moveit_commander import MoveGroupCommander, PlanningSceneInterface
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import PoseStamped, Pose
from mm_robot_decision.msg import pose4
from copy import deepcopy
#from mm_arm_control.msg import runstate
from std_msgs.msg import String,Int8

# 配置信息
# 初始化move_group的API
moveit_commander.roscpp_initialize(sys.argv)
arm = moveit_commander.MoveGroupCommander('manipulator')       
end_effector_link = arm.get_end_effector_link()  

#设置坐标系，和真实机械臂坐标系统一x,y,z             
reference_frame = 'base'                                                        
arm.set_pose_reference_frame(reference_frame)        
arm.allow_replanning(True)

#moveit阈值
arm.set_goal_position_tolerance(0.0001)
arm.set_goal_orientation_tolerance(0.0001)

#状态publisher（moving/compensating/done)
#state_pub = rospy.Publisher('excuted_state', runstate , queue_size=10)  

#定义动作完成的状态发布
over_pub=rospy.Publisher('/mm_arm/isArrived',Int8 , queue_size=10)

#URscript publisher          
string_pub = rospy.Publisher('/ur_driver/URScript', String , queue_size=10)      

class MoveItIkTest:
         
    def __init__(self):

        # 初始化场景对象
        #scene = PlanningSceneInterface()
        #rospy.sleep(1) 
        #self.scene_pub = rospy.Publisher('planning_scene', PlanningScene, queue_size=10)

        # 初始化ROS节点
        rospy.init_node('moveit_ik_test')
        rospy.loginfo('started')

     
        # 复位待机
        #arm.set_named_target('start_pose')  
        #arm.go()                 
        #rospy.sleep(1)


        # 订阅位姿信息
        rospy.Subscriber('/mm_arm/goal', pose4, self.callback)
        rospy.spin()

    def callback(self,pose4):
               
        # 设置机械臂工作空间中的目标位姿，位置使用x、y、z坐标输入，
        # 姿态使用rpy输入，基于base坐标系
        if pose4.state == 'WorkSpacePlanning':

            # 获取目标位姿
            target_pose = PoseStamped()
            target_pose.header.frame_id = reference_frame
            target_pose.header.stamp = rospy.Time.now()     
            target_pose.pose.position.x = pose4.x
            target_pose.pose.position.y = pose4.y
            target_pose.pose.position.z = pose4.z

            # ee_link和真实坐标系tool0存在(r+pi/2,p,y+pi/2)转换，用于moveit
            point = tf.transformations.quaternion_from_euler((pose4.a+1.57),pose4.b,(pose4.c+1.57))               
            target_pose.pose.orientation.x = point[0]
            target_pose.pose.orientation.y = point[1]
            target_pose.pose.orientation.z = point[2]
            target_pose.pose.orientation.w = point[3]
            rospy.loginfo('set position:\n%s', target_pose.pose)
            cartesian = pose4.wct
            rospy.loginfo("Cartesian is %r", cartesian)
            #estate = 'moving'
            #state_pub.publish(runstate(estate))

            # 输入的姿态rpy的旋转矢量表示法的转化,用于urscript
            r2q = tf.transformations.quaternion_from_euler(pose4.a,pose4.b,pose4.c)
            theta = math.acos(r2q[3])
            if theta == 0.0:
                rec = [0.0,0.0,0.0]
            else:
                rec = [r2q[0]*2*theta/math.sin(theta),r2q[1]*2*theta/math.sin(theta),r2q[2]*2*theta/math.sin(theta)]

            # cartesian = true,采用urscript直接发布，先到达过渡位置，再沿工具坐标系z轴前进150mm到达目标位置（防碰处理）
            if cartesian: 
                pose_format = '''def myProg():
  p_a1=get_actual_tcp_pose()
  offset1=p[%s,%s,%s,%s,%s,%s]
  p_g1=pose_trans(p_a1,offset1)
  offset2=p[0,0,-0.34,0,0,0]
  p_g2=pose_trans(p_g1,offset2)
  movel(p_g2,a=0.05,v=0.05,r=0)
  sleep(0.5)
  offset3=p[0,0,0.1,0,0,0]
  p_g3=pose_trans(p_g2,offset3)
  movel(p_g3,a=0.03,v=0.03,r=0)
  sleep(0.5)
end'''
                values = (("%.5f" % pose4.x),("%.5f" % pose4.y),("%.5f" % pose4.z),("%.5f" % rec[0]),("%.5f" % rec[1]),("%.5f" % rec[2]))
                
                true_pose = pose_format % values 

                string_pub.publish(String(true_pose))

                #监听机械臂反馈位姿信息，判断是否到位，发布状态信息

               
                listener = tf.TransformListener()
                listener.waitForTransform("/base", "/tool0_controller", rospy.Time(), rospy.Duration(4.0))
                (trans1,rot1) = listener.lookupTransform('/base', '/tool0_controller', rospy.Time(0))
	        rospy.sleep(0.8)
                (trans2,rot2)=listener.lookupTransform('/base', '/tool0_controller', rospy.Time(0))
                while abs(trans2[0]-trans1[0])>0.0001 or abs(trans2[1]-trans1[1])>0.0001 or abs(trans2[2]-trans1[2])>0.0001:
                    (trans1,rot1) = listener.lookupTransform('/base', '/tool0_controller', rospy.Time(0))
                    rospy.sleep(0.8)
                    (trans2,rot2) = listener.lookupTransform('/base', '/tool0_controller', rospy.Time(0))

                rpy = tf.transformations.euler_from_quaternion([rot2[0],rot2[1],rot2[2],rot2[3]])
                rospy.loginfo('Actual pose： \n x=%f ,y=%f ,z=%f ,r=%f ,p=%f ,y=%f',trans2[0],trans2[1],trans2[2],rpy[0],rpy[1],rpy[2]) 
                #estate = 'done'
                #state_pub.publish(runstate(estate))
                rospy.sleep(0.5)
                isArrived_msg=1
                over_pub.publish(isArrived_msg)

            # cartesian = false,采用moveit规划，urscript补偿
            else:

                # 设置机器臂当前的状态作为运动初始状态
                arm.set_start_state_to_current_state()
        
                # 设置机械臂终端运动的目标位姿
                arm.set_pose_target(target_pose, end_effector_link)
        
                # 规划运动路径
                traj = arm.plan()
        
                # 按照规划的运动路径控制机械臂运动
                arm.execute(traj)
                rospy.sleep(1)
                
                # 低速补偿，发布状态信息
                pose_format = 'movel(p[%s,%s,%s,%s,%s,%s],a=0.01,v=0.01,r=0)'      
                values = (("%.5f" % pose4.x),("%.5f" % pose4.y),("%.5f" % pose4.z),("%.5f" % rec[0]),("%.5f" % rec[1]),("%.5f" % rec[2]))
                true_pose = pose_format % values
                estate = 'compensating'
                state_pub.publish(runstate(estate))
                rospy.sleep(1)
                string_pub.publish(String(true_pose))
                listener = tf.TransformListener()
                listener.waitForTransform("/base", "/tool0_controller", rospy.Time(), rospy.Duration(4.0))
                (trans1,rot1) = listener.lookupTransform('/base', '/tool0_controller', rospy.Time(0))
                rospy.sleep(0.8)
                (trans2,rot2)=listener.lookupTransform('/base', '/tool0_controller', rospy.Time(0))
                while abs(trans2[0]-trans1[0])>0.0001 or abs(trans2[1]-trans1[1])>0.0001 or abs(trans2[2]-trans1[2])>0.0001:
                    (trans1,rot1) = listener.lookupTransform('/base', '/tool0_controller', rospy.Time(0))
                    rospy.sleep(0.8)
                    (trans2,rot2) = listener.lookupTransform('/base', '/tool0_controller', rospy.Time(0))

                rpy = tf.transformations.euler_from_quaternion([rot2[0],rot2[1],rot2[2],rot2[3]])
                rospy.loginfo('Actual pose： \n x=%f ,y=%f ,z=%f ,r=%f ,p=%f ,y=%f',trans2[0],trans2[1],trans2[2],rpy[0],rpy[1],rpy[2]) 
                #estate = 'done'
                #state_pub.publish(runstate(estate))
                rospy.sleep(0.5)
                isArrived_msg=1
                over_pub.publish(isArrived_msg)
	elif pose4.state=='jointSpacePlanning':
           # joint_positions=[("%.5f" % pose4.x),("%.5f" % pose4.y),("%.5f" % pose4.z),("%.5f" % pose4.a),("%.5f" % pose4.b),("%.5f" % pose4.c)]'''
           # joint_positions=[pose4.x,pose4.y,pose4.z,pose4.a,pose4.b,pose4.c]
           # arm.set_joint_value_target(joint_positions)
           # arm.go() 
            

         

           # urscript,谨慎使用，必须先测试
            pose_format = 'movej([%s,%s,%s,%s,%s,%s],a=0.4,v=0.2,r=0)' 
            values = (("%.5f" % pose4.x),("%.5f" % pose4.y),("%.5f" % pose4.z),("%.5f" % pose4.a),("%.5f" % pose4.b),("%.5f" % pose4.c))
            true_pose = pose_format % values  
            rospy.loginfo(true_pose)
            rospy.sleep(0.5)
            string_pub.publish(String(true_pose))
            
            listener = tf.TransformListener()
            listener.waitForTransform("/base", "/tool0_controller", rospy.Time(), rospy.Duration(4.0))
            (trans1,rot1) = listener.lookupTransform('/base', '/tool0_controller', rospy.Time(0))
            rospy.sleep(0.2)
            (trans2,rot2)=listener.lookupTransform('/base', '/tool0_controller', rospy.Time(0))
            while abs(trans2[0]-trans1[0])>0.0001 or abs(trans2[1]-trans1[1])>0.0001 or abs(trans2[2]-trans1[2])>0.0001:
                (trans1,rot1) = listener.lookupTransform('/base', '/tool0_controller', rospy.Time(0))
                rospy.sleep(0.2)
                (trans2,rot2) = listener.lookupTransform('/base', '/tool0_controller', rospy.Time(0))

            isArrived_msg=1
 	    rospy.sleep(0.2)
            over_pub.publish(isArrived_msg)
            


        elif pose4.state == 'MoveInTool':
            # 用于直接在当前位置工具坐标系中运动，urscript。如后退100mm，则发布0,0,-0.1,0,0,0
            # 输入的姿态rpy的旋转矢量表示法的转化,用于urscript
            r2q = tf.transformations.quaternion_from_euler(pose4.a,pose4.b,pose4.c)
            theta = math.acos(r2q[3])
            if theta == 0.0:
                rec = [0.0,0.0,0.0]
            else:
                rec = [r2q[0]*2*theta/math.sin(theta),r2q[1]*2*theta/math.sin(theta),r2q[2]*2*theta/math.sin(theta)]

            pose_format = '''def myProg():
  p_a=get_actual_tcp_pose()
  offset=p[%s,%s,%s,%s,%s,%s]
  p_g=pose_trans(p_a,offset)
  movel(p_g,a=0.05,v=0.04,r=0)
  sleep(0.2)
end'''
            values = (("%.5f" % pose4.x),("%.5f" % pose4.y),("%.5f" % pose4.z),("%.5f" % rec[0]),("%.5f" % rec[1]),("%.5f" % rec[2]))
            
            true_pose = pose_format % values 
            rospy.loginfo(true_pose)
	    rospy.sleep(1)
            string_pub.publish(String(true_pose))
            #estate = 'done'
            #state_pub.publish(runstate(estate))
            listener = tf.TransformListener()
            listener.waitForTransform("/base", "/tool0_controller", rospy.Time(), rospy.Duration(4.0))
            (trans1,rot1) = listener.lookupTransform('/base', '/tool0_controller', rospy.Time(0))
	    rpy1 = tf.transformations.euler_from_quaternion([rot1[0],rot1[1],rot1[2],rot1[3]])
	    rospy.sleep(0.5)
	    
            (trans2,rot2)=listener.lookupTransform('/base', '/tool0_controller', rospy.Time(0))
	    rpy2 = tf.transformations.euler_from_quaternion([rot2[0],rot2[1],rot2[2],rot2[3]])
            while abs(trans2[0]-trans1[0])>0.0001 or abs(trans2[1]-trans1[1])>0.0001 or abs(trans2[2]-trans1[2])>0.0001 or abs(rpy2[0]-rpy1[0])>0.0001 or abs(rpy2[1]-rpy1[1])>0.0001 or abs(rpy2[2]-rpy1[2])>0.0001:
                (trans1,rot1) = listener.lookupTransform('/base', '/tool0_controller', rospy.Time(0))
		rpy1 = tf.transformations.euler_from_quaternion([rot1[0],rot1[1],rot1[2],rot1[3]])
                rospy.sleep(0.2)
                (trans2,rot2) = listener.lookupTransform('/base', '/tool0_controller', rospy.Time(0))
 		rpy2 = tf.transformations.euler_from_quaternion([rot2[0],rot2[1],rot2[2],rot2[3]])
            estate = 'done'
            rospy.loginfo(estate)
            isArrived_msg=1
            rospy.sleep(0.2) 
            over_pub.publish(isArrived_msg)


	elif pose4.state=='StandBy':
           # joint_positions=[("%.5f" % pose4.x),("%.5f" % pose4.y),("%.5f" % pose4.z),("%.5f" % pose4.a),("%.5f" % pose4.b),("%.5f" % pose4.c)]'''
	#267.22*3.14159/180,10.96*3.14159/180,-163.66*3.14159/180,-28.24*3.14159/180,-87.22*3.14159/180,0.22*3.14159/180
           # joint_positions=[202.26*3.14159/180,-45.21*3.14159/180,-126.66*3.14159/180,-7.56*3.14159/180,-34.61*3.14159/180,0.09*3.14159/180]
           # arm.set_joint_value_target(joint_positions)
           # arm.go() 

           # rospy.sleep(0.2) 

           # joint_positions=[101.99*3.14159/180,-99.09*3.14159/180,-137.70*3.14159/180,-32.73*3.14159/180,89.59*3.14159/180,12.11*3.14159/180]
           # arm.set_joint_value_target(joint_positions)
           # arm.go() 
            
            pose_format = '''def myProg():
  movej([%s,%s,%s,%s,%s,%s],a=0.5,v=0.25,r=0)
  movej([%s,%s,%s,%s,%s,%s],a=0.5,v=0.25,r=0)
end'''
            values = (("%.5f" % (202.26*3.14159/180)),("%.5f" % (-45.21*3.14159/180)),("%.5f" % (-126.66*3.14159/180)),("%.5f" % (-7.56*3.14159/180)),("%.5f" % (-34.61*3.14159/180)),("%.5f" % (0.09*3.14159/180)),("%.5f" % (101.99*3.14159/180)),("%.5f" % (-99.09*3.14159/180)),("%.5f" % (-137.70*3.14159/180)),("%.5f" % (-32.73*3.14159/180)),("%.5f" % (89.59*3.14159/180)),("%.5f" % (12.11*3.14159/180)))


            true_pose = pose_format % values  
            rospy.loginfo(true_pose)
            rospy.sleep(0.5)
            string_pub.publish(String(true_pose))
         
            
            listener = tf.TransformListener()
            listener.waitForTransform("/base", "/tool0_controller", rospy.Time(), rospy.Duration(4.0))
            (trans1,rot1) = listener.lookupTransform('/base', '/tool0_controller', rospy.Time(0))
            rospy.sleep(0.2)
            (trans2,rot2)=listener.lookupTransform('/base', '/tool0_controller', rospy.Time(0))
            while abs(trans2[0]-trans1[0])>0.0001 or abs(trans2[1]-trans1[1])>0.0001 or abs(trans2[2]-trans1[2])>0.0001:
                (trans1,rot1) = listener.lookupTransform('/base', '/tool0_controller', rospy.Time(0))
                rospy.sleep(0.2)
                (trans2,rot2) = listener.lookupTransform('/base', '/tool0_controller', rospy.Time(0))

            isArrived_msg=1
 	    rospy.sleep(0.2)
            over_pub.publish(isArrived_msg)
            

	elif pose4.state=='BackHome':
           # joint_positions=[("%.5f" % pose4.x),("%.5f" % pose4.y),("%.5f" % pose4.z),("%.5f" % pose4.a),("%.5f" % pose4.b),("%.5f" % pose4.c)]'''
	#267.22*3.14159/180,10.96*3.14159/180,-163.66*3.14159/180,-28.24*3.14159/180,-87.22*3.14159/180,0.22*3.14159/180
            #joint_positions=[202.26*3.14159/180,-45.21*3.14159/180,-126.66*3.14159/180,-7.56*3.14159/180,-34.61*3.14159/180,0.09*3.14159/180]
            #arm.set_joint_value_target(joint_positions)
           # arm.go() 

            #rospy.sleep(0.2) 
           # joint_positions=[267.22*3.14159/180,10.96*3.14159/180,-163.66*3.14159/180,-28.24*3.14159/180,-87.22*3.14159/180,0.22*3.14159/180]
           # arm.set_joint_value_target(joint_positions)
           # arm.go() 
           

            pose_format = '''def myProg():
  movej([%s,%s,%s,%s,%s,%s],a=0.5,v=0.25,r=0)
  movej([%s,%s,%s,%s,%s,%s],a=0.5,v=0.25,r=0)
end'''
            values = (("%.5f" % (202.26*3.14159/180)),("%.5f" % (-45.21*3.14159/180)),("%.5f" % (-126.66*3.14159/180)),("%.5f" % (-7.56*3.14159/180)),("%.5f" % (-34.61*3.14159/180)),("%.5f" % (0.09*3.14159/180)),("%.5f" % (267.22*3.14159/180)),("%.5f" % (10.96*3.14159/180)),("%.5f" % (-163.66*3.14159/180)),("%.5f" % (-28.24*3.14159/180)),("%.5f" % (-87.22*3.14159/180)),("%.5f" % (0.22*3.14159/180)))

            true_pose = pose_format % values  
            rospy.loginfo(true_pose)
            rospy.sleep(0.5)
            string_pub.publish(String(true_pose))
            
            listener = tf.TransformListener()
            listener.waitForTransform("/base", "/tool0_controller", rospy.Time(), rospy.Duration(4.0))
            (trans1,rot1) = listener.lookupTransform('/base', '/tool0_controller', rospy.Time(0))
            rospy.sleep(0.2)
            (trans2,rot2)=listener.lookupTransform('/base', '/tool0_controller', rospy.Time(0))
            while abs(trans2[0]-trans1[0])>0.0001 or abs(trans2[1]-trans1[1])>0.0001 or abs(trans2[2]-trans1[2])>0.0001:
                (trans1,rot1) = listener.lookupTransform('/base', '/tool0_controller', rospy.Time(0))
                rospy.sleep(0.2)
                (trans2,rot2) = listener.lookupTransform('/base', '/tool0_controller', rospy.Time(0))

            isArrived_msg=1
 	    rospy.sleep(0.2)
            over_pub.publish(isArrived_msg)



        # 位姿输入节点发出关闭指令，回到start_pose位置并退出moveit
        elif pose4.state == 'shutdown':
            #arm.set_named_target('start_pose')  
            #                
            rospy.sleep(1)
            moveit_commander.roscpp_shutdown()
            moveit_commander.os._exit(0)

if __name__ == "__main__":
    try:
        MoveItIkTest()
    except rospy.ROSInterruptException:
        pass