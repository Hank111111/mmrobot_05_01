#!/usr/bin/env python
# -*- coding: UTF-8 -*-
'''
 * @Author: WangNing
 * @Date: 2018-11-21 10:40:21
 * @LastEditors: ChenMin
 * @LastEditTime: 2018-11-21 10:40:44
 * @Description:移动底盘到某个目标点，到位后发布已就位消息(succeed/failed)到
                话题/trans_location_signal
 * 
 * 
 * 
'''
import rospy
import random
import actionlib
import time
import logging
import logging.config
from logging.handlers import TimedRotatingFileHandler
from actionlib_msgs.msg import *

from geometry_msgs.msg import PoseStamped,Pose, Point, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import Int8,Header

class GoalNav():
    def __init__(self):
        
        rospy.init_node('mm_trans_node', anonymous=False) 
        self.pub=rospy.Publisher('/mm_trans/isArrived',Int8,queue_size=8)
        rospy.Subscriber('/mm_trans/goal',PoseStamped,self.callback)
        rospy.spin()
        
    ''' 创建一个日志器logger并设置其日志级别为DEBUG
        logFilePath = "mm_trans.log"
        #日志的 初始化
        self.logger = logging.getLogger('goal_nav_logger')
        self.logger.setLevel(logging.DEBUG)
        # 1.创建一个流处理器handler并设置其日志级别为DEBUG
        handler = logging.StreamHandler(sys.stdout)
        handler.setLevel(logging.DEBUG)
        # 2.创建一个handler，用于写入日志文件
        fh = logging.FileHandler(logFilePath)
        fh.setLevel(logging.DEBUG)
        # 3.日志回滚
        handle = TimedRotatingFileHandler(
            logFilePath, when="d", interval=1, backupCount=7)
        # 创建一个格式器formatter并将其添加到处理器handler
        formatter = logging.Formatter(
            "%(asctime)s - %(name)s - %(levelname)s - %(message)s")
        handler.setFormatter(formatter)
        fh.setFormatter(formatter)
        handle.setFormatter(formatter)
        # 为日志器logger添加上面创建的处理器handler
        # logger.addHandler(handler)
        # self.logger.addHandler(fh)
        self.logger.addHandler(handle)'''
        


    def callback(self,PoseStamped):
        self.location=PoseStamped
        #rospy.loginfo(self.location)
        goal_states = [
            'PENDING', 'ACTIVE', 'PREEMPTED', 'SUCCEEDED', 'ABORTED',
            'REJECTED', 'PREEMPTING', 'RECALLING', 'RECALLED', 'LOST'
        ]

        # Subscribe to the move_base action server
        self.move_base = actionlib.SimpleActionClient("move_base",
                                                      MoveBaseAction)
        self.move_base.wait_for_server(rospy.Duration(30))
        rospy.loginfo("Connected to move base server")
        #self.logger.info("Connected to move base server")
        rospy.loginfo("Starting to go to the goal ")
        '''self.logger.info(
           "+++++++++++++++++++++Starting position navigation+++++++++++++++++++++"
        )'''
        # Begin the main loop and run through a sequence of locations
 #       while not rospy.is_shutdown():
            

            # Get the next location in the current sequence

        if (self.location.header.frame_id=="trans"):
            rospy.loginfo("Going to: goal" )
            #self.logger.info("Going to: goal ")
            rospy.sleep(1)
            self.send_goal(self.location.pose)
        
        
        # Allow 5 minutes to get there
        finished_within_time = self.move_base.wait_for_result(
                rospy.Duration(300))
            # Check for success or failure
        if not finished_within_time:
            self.move_base.cancel_goal()
            #rospy.logerr("ERROR:Timed out achieving goal")
            #self.logger.error("ERROR:Timed out achieving goal")
        else:
            state = self.move_base.get_state()
            if state == GoalStatus.SUCCEEDED:
                isArrived_msg=1
                #arrived_msg='Trans is arrived'
                    
                #rospy.loginfo(arrived_msg)
                    
                #self.logger.info(arrived_msg)
                rospy.sleep(0.5)
                self.pub.publish(isArrived_msg)
            else:
                isArrived_msg=-1
                self.pub.publish(isArrived_msg)
                rospy.logerr("Goal failed with error code:" +
                                str(goal_states[state]))
#                self.logger.error("Goal failed with error code:" +
#                                     str(goal_states[state]))

           # rospy.sleep(self.rest_time)
    def send_goal(self, locate):
        # Set up the next goal location
        self.goal = MoveBaseGoal()
        self.goal.target_pose.pose = locate
        self.goal.target_pose.header.frame_id = 'map'
        self.goal.target_pose.header.stamp = rospy.Time.now()
        self.move_base.send_goal(self.goal)  #send goal to move_base

    def trunc(self, f, n):
        # Truncates/pads a float f to n decimal places without rounding
        slen = len('%.*f' % (n, f))
        return float(str(f)[:slen])

    def shutdown(self):
        rospy.logwarn("Stopping the patrol...")

if __name__ == '__main__':
    try:
        GoalNav()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.logwarn("patrol navigation exception finished.")
