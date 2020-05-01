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
from geometry_msgs.msg import Pose, Point, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


class GoalNav():
    def __init__(self):
        
        rospy.init_node('goal_nav_node', anonymous=False) 
        #当节点关闭，会调用括弧里面的函数
        rospy.on_shutdown(self.shutdown)
        # 创建一个日志器logger并设置其日志级别为DEBUG
        logFilePath = "goal_nav.log"
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
        self.logger.addHandler(handle)

        # From launch file get parameters
        self.rest_time = rospy.get_param("~rest_time", 5)
        self.keep_patrol = rospy.get_param("~keep_patrol", True)
        self.random_patrol = rospy.get_param("~random_patrol", False)
        self.patrol_type = rospy.get_param("~patrol_type", 0)
        self.patrol_loop = rospy.get_param("~patrol_loop", 1)
        self.patrol_time = rospy.get_param("~patrol_time", 5)

        #set all navigation target pose
        self.locations = dict()

        #self.locations['one'] = Pose(
            #Point(2.22, -0.18, 0.0), 
            #Quaternion(0.0, 0.0, 0.0, 1))
        self.locations['goal one'] = Pose(
            Point(4.20, -0.19, 0.00),
            Quaternion(0.0, 0, 0.0, 1))
       # self.locations['three'] = Pose(
            #Point(1.09, -0.11, 0.00),
            #Quaternion(0.000, 0.000, 0.0,1))
        
        # Goal state return values
        goal_states = [
            'PENDING', 'ACTIVE', 'PREEMPTED', 'SUCCEEDED', 'ABORTED',
            'REJECTED', 'PREEMPTING', 'RECALLING', 'RECALLED', 'LOST'
        ]

        # Subscribe to the move_base action server
        self.move_base = actionlib.SimpleActionClient("move_base",
                                                      MoveBaseAction)
        self.move_base.wait_for_server(rospy.Duration(30))
        rospy.loginfo("Connected to move base server")
        self.logger.info("Connected to move base server")

        # Variables to keep track of success rate, running time etc.
        # loop_cnt = 0
        n_goals = 0
        n_successes = 0
        target_num = 0
        running_time = 0
        location = ""
        start_time = rospy.Time.now()
        #locations_cnt = len(self.locations)
        sequeue = ['goal one', 'goal two', 'goal three', 'goal four', 'goal five', 'goal six']

        rospy.loginfo("Starting to go to the goal ")
        self.logger.info(
            "+++++++++++++++++++++Starting position navigation+++++++++++++++++++++"
        )
        # Begin the main loop and run through a sequence of locations
        while not rospy.is_shutdown():
            

            # Get the next location in the current sequence
            location = sequeue[target_num]
            rospy.loginfo("Going to: " + str(location))
            self.logger.info("Going to: " + str(location))
            self.send_goal(location)

            # Increment the counters
            target_num += 1
            n_goals += 1

            # Allow 5 minutes to get there
            finished_within_time = self.move_base.wait_for_result(
                rospy.Duration(300))
            # Check for success or failure
            if not finished_within_time:
                self.move_base.cancel_goal()
                rospy.logerr("ERROR:Timed out achieving goal")
                self.logger.error("ERROR:Timed out achieving goal")
            else:
                state = self.move_base.get_state()
                if state == GoalStatus.SUCCEEDED:
                    n_successes += 1
                    rospy.loginfo("Goal succeeded!")
                    self.logger.info("Goal succeeded!")
                else:
                    rospy.logerr("Goal failed with error code:" +
                                 str(goal_states[state]))
                    self.logger.error("Goal failed with error code:" +
                                      str(goal_states[state]))

            # How long have we been running?
            running_time = rospy.Time.now() - start_time
            running_time = running_time.secs / 60.0

            # Print a summary success/failure and time elapsed
            rospy.loginfo("!!!!!Success so far: " + str(n_successes) + "/" +
                          str(n_goals) + " = " +
                          str(100 * n_successes / n_goals) + "%" + "!!!!!")
            self.logger.info("!!!!!Success so far: " + str(n_successes) + "/" +
                             str(n_goals) + " = " +
                             str(100 * n_successes / n_goals) + "% !!!!!")
            rospy.loginfo("Running time: " + str(self.trunc(running_time, 1)) +
                          " min")
            self.logger.info("Running time: " +
                             str(self.trunc(running_time, 1)) + " min")
            rospy.sleep(self.rest_time)



    def send_goal(self, locate):
        # Set up the next goal location
        self.goal = MoveBaseGoal()
        self.goal.target_pose.pose = self.locations[locate]
        self.goal.target_pose.header.frame_id = 'map'
        self.goal.target_pose.header.stamp = rospy.Time.now()
        self.move_base.send_goal(self.goal)  #send goal to move_base

    def trunc(self, f, n):
        # Truncates/pads a float f to n decimal places without rounding
        slen = len('%.*f' % (n, f))
        return float(str(f)[:slen])

    def shutdown(self):
        rospy.logwarn("Stopping the patrol...")
        self.logger.warn(
            "+++++++++++++++++++++Stopping the patrol+++++++++++++++++++++")


if __name__ == '__main__':
    try:
        GoalNav()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.logwarn("patrol navigation exception finished.")
