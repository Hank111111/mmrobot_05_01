#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, sys
import moveit_commander

class Initstandby:
    def __init__(self):
        # 初始化move_group的API
        moveit_commander.roscpp_initialize(sys.argv)

        # 初始化ROS节点
        rospy.init_node('init_standby', anonymous=True)
 
        # 初始化需要使用move group控制的机械臂中的manipulator group
        arm = moveit_commander.MoveGroupCommander('manipulator')
             
        # 设置机械臂的允许误差值
        arm.set_goal_joint_tolerance(0.001)
         
        # 控制机械臂回到standby位置
        arm.set_named_target('standby')  #=rviz select goal state
        arm.go()                    #=rviz plan and execute
        rospy.sleep(2)
        
        # 关闭并退出moveit
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)

if __name__ == "__main__":
    try:
        Initstandby()
    except rospy.ROSInterruptException:
        pass
