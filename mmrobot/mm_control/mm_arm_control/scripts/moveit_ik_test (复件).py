#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, sys
import tf
import math
from trajectory_msgs.msg import JointTrajectoryPoint
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import PoseStamped, Pose
from mm_robot_decision.msg import pose4
from copy import deepcopy
from std_msgs.msg import String,Int8


# 0307更新说明：重大更新，删除了moveit部分

#定义动作完成的状态发布
over_pub=rospy.Publisher('/mm_arm/isArrived',Int8 , queue_size=10)

#URscript publisher          
string_pub = rospy.Publisher('/ur_driver/URScript', String , queue_size=10) 
     
class MoveItIkTest:
         
    def __init__(self):

       
	# 配置信息
        # 初始化ROS节点
        rospy.init_node('moveit_ik_test')
        rospy.loginfo('started')

        # 订阅位姿信息
        rospy.Subscriber('/mm_arm/goal', pose4, self.callback)
        rospy.spin()

    def callback(self,pose4):
          
	# 七种控制指令，四种运动模式＋两种待机位+退出
        # 1. WorkSpacePlanning，相机（工具）坐标系，相机拍摄后两步运动到位套进旋钮
	# 2. jointSpacePlanning，关节坐标系，关节空间规划
	# 3. MoveIntool，工具坐标系，工具坐标系中直接运动
	# 4. BaseSpacePlanning,基座坐标系，工作空间规划
	# 5. StandBy，取工具准备状态
	# 6. BackHome，打包状态
	# 7. shutdown，退出，重启需重新运行本脚本     

        if pose4.state == 'WorkSpacePlanning':

        	# 输入的姿态rpy的旋转矢量表示法的转化
        	r2q = tf.transformations.quaternion_from_euler(pose4.a,pose4.b,pose4.c)
       		theta = math.acos(r2q[3])
        	if theta == 0.0:
			rec = [0.0,0.0,0.0]
        	else:
                	rec = [r2q[0]*2*theta/math.sin(theta),r2q[1]*2*theta/math.sin(theta),r2q[2]*2*theta/math.sin(theta)]

            	# 采用urscript直接发布，先到达过渡位置，再沿工具坐标系z轴前进到达目标位置（防碰处理）
                pose_format = '''def myProg():
  p_a1=get_actual_tcp_pose()
  offset1=p[%s,%s,%s,%s,%s,%s]
  p_g1=pose_trans(p_a1,offset1)
  offset2=p[0,0,%s,0,0,0]
  p_g2=pose_trans(p_g1,offset2)
  movel(p_g2,a=0.05,v=0.05,r=0)
  sleep(0.5)
  offset3=p[0,0,0.1,0,0,0]
  p_g3=pose_trans(p_g2,offset3)
  movel(p_g3,a=0.03,v=0.03,r=0)
  sleep(0.5)
end'''

		if pose4.wct == 1:
			rospy.loginfo("一号工具")
                	values = (("%.5f" % pose4.x),("%.5f" % pose4.y),("%.5f" % pose4.z),("%.5f" % rec[0]),("%.5f" % rec[1]),("%.5f" % rec[2]),(-0.34))
		else:
			rospy.loginfo("二号工具")
		        values = (("%.5f" % pose4.x),("%.5f" % pose4.y),("%.5f" % pose4.z),("%.5f" % rec[0]),("%.5f" % rec[1]),("%.5f" % rec[2]),(-0.34))		
                
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
                rospy.sleep(0.5)
                isArrived_msg=1
                over_pub.publish(isArrived_msg)


	elif pose4.state=='jointSpacePlanning':

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


	elif pose4.state=='BaseSpacePlanning':

           	# urscript,谨慎使用，必须先测试，姿态角使用轴角表示法，用于固定的示教拍照位置
		pose_format = 'movel(p[%s,%s,%s,%s,%s,%s],a=0.2,v=0.1,r=0)' 
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



	elif pose4.state == 'StandBy':
            
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
            

	elif pose4.state == 'BackHome':

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



        # 位姿输入节点发出关闭指令
        elif pose4.state == 'shutdown':
		
		exit(0);               


if __name__ == "__main__":
    try:
        MoveItIkTest()
    except rospy.ROSInterruptException:
        pass
