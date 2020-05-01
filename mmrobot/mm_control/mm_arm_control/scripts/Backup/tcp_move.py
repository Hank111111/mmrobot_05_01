#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, sys
import tf
from tf.transformations import *
from moveit_msgs.msg import RobotTrajectory, PlanningScene, ObjectColor
from trajectory_msgs.msg import JointTrajectoryPoint
from moveit_commander import MoveGroupCommander, PlanningSceneInterface
from geometry_msgs.msg import PoseStamped, Pose
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from test_planning.msg import runstate
from std_msgs.msg import String



         

def talker():               
        # 初始化ROS节点
        rospy.init_node('tcp_move')
        rospy.loginfo('started')
        string_pub = rospy.Publisher('/ur_driver/URScript', String , queue_size=10)
        rospy.sleep(3)
        
        pose_format = '''def myProg():
  movel(p[%s,%s,%s,%s,%s,%s],a=0.1,v=0.1,r=0)
  sleep(3)
  p_a=get_actual_tcp_pose()
  offset=p[%s,%s,%s,0,0,0]
  p_g=pose_trans(p_a,offset)
  movel(p_g,a=0.1,v=0.1,r=0)
  sleep(1)
end'''
        values = (-0.4,-0.092,0.7,0.78,2.73,-2.49,0,0,0.2)
        true_pose = pose_format % values
        
        string_pub.publish(String(true_pose))
        
        rospy.sleep(1)

        exit(0)


if __name__ == "__main__":
    talker()
    
