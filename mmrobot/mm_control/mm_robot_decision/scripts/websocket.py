#! /usr/bin/env python
# -*- coding: utf-8 -*-

from SimpleWebSocketServer import SimpleWebSocketServer, WebSocket
import Queue
import rospy
import time
import roslib
import rospy
import actionlib
import threading
import os
import logging
import rospkg
import tf
import tf2_ros
import geometry_msgs.msg

from socket import *
from std_msgs.msg import String, Int8, UInt8
from actionlib_msgs.msg import GoalStatus, GoalID
from mm_robot_decision.msg import robot_executionAction, robot_executionGoal, robot_executionFeedback, robot_executionResult
from mm_endmotor_control.msg import motorCommand
from youibot_msgs.msg import Battery


threadLock = threading.Lock()
threads = []
orderq = Queue.Queue()
feedbackq = Queue.Queue()

global mission_number
mission_number = 0
global excute_number
excuted_number = 0


logging.basicConfig(level=logging.DEBUG,
                    format='%(asctime)s \"%(filename)s\"[line:%(lineno)d] %(levelname)s %(message)s',
                    datefmt='%a, %d %b %Y %H:%M:%S',
                    filename="/home/youibot/mrobot/robotlogs/" +
                    time.strftime('%Y.%m.%d', time.localtime(
                        time.time()))+"-ubuntu_server.log",
                    filemode='a')


class SimpleEcho(WebSocket):

    def handleMessage(self):
        mission = self.data.encode('utf8')

        print(mission_number, excuted_number)

        # 增加急停任务优先级
        if mission == '123cancel':
            threadLock.acquire()
            orderq.put(mission)
            threadLock.release()

        elif mission_number > excuted_number:
            self.sendMessage('Last mission not completed, please wait')

        elif mission_number == excuted_number:
            self.preprocess(mission)

    def handleConnected(self):
        print(self.address, 'connected')
        logging.info('connected')

    def handleClose(self):
        print(self.address, 'closed')
        logging.info('closed')

    def preprocess(self, mission):
        global mission_number, excuted_number
        check_str = mission.replace(' ', '').split('\r\n')
        # 任务有效性检验
        while '' in check_str:
            check_str.remove('')
        print(check_str)
        if check_str[0] == '<<start' and check_str[-1] == 'end>>':
            check_str.remove('<<start')
            check_str.remove('end>>')
            print(check_str)
            threadLock.acquire()
            orderq.put(check_str)
            mission_number += 1
            threadLock.release()
            self.sendMessage(
                'mission accepted, please wait, now excuting mission')
            logging.info("receive command: "+'\r\n'+mission)
        else:
            self.sendMessage('Wrong order, please check')
            logging.info('Wrong order, please check')

    def localtest(self):
        rospack = rospkg.RosPack()
        pkg_pth = rospack.get_path('mm_robot_decision')
        f = open(pkg_pth+'/scripts/1003#.txt')
        mission = f.read()
        # rospy.sleep(10)
        self.preprocess(mission)


def websocketserver():
    server = SimpleWebSocketServer('', 8888, SimpleEcho)
    while not rospy.is_shutdown():
        server.serveonce()


class Ros_action():

    # triggered by result msg, uint8 3 in GoalStatus() means SUCCEED
    def result_cb(self, act_state=GoalStatus(), act_result=robot_executionResult()):
        print("result: "+str(act_result.missions_done)+' done')
        logging.info("result: "+str(act_result.missions_done)+' done')
        if 3 == act_state:
            threadLock.acquire()
            print 'robot 2 finished'
            feedbackq.put('robot 2 finished')
            threadLock.release()
        else:
            threadLock.acquire()
            print 'robot 2 aborted'
            feedbackq.put('robot 2 aborted')
            threadLock.release()

    # triggered by feedback msg
    def check_process(self, act_feedback=robot_executionFeedback()):
        print(act_feedback.current_mission, act_feedback.mission_state)
        logging.info(act_feedback.current_mission +
                     str(act_feedback.mission_state))
        threadLock.acquire()
        feedbackq.put(act_feedback.current_mission+'//' +
                      str(act_feedback.mission_state))
        threadLock.release()

    def execution(self):
        global mission_number, excuted_number
        print(mission_number, excuted_number)
        self.client = actionlib.SimpleActionClient(
            'robot_execute', robot_executionAction)
        self.client.wait_for_server()

        self.goal = robot_executionGoal()

        while not rospy.is_shutdown():
            print("now excuting: ", excuted_number)
            order_check = orderq.get()
            if order_check[0] == '123cancel':
                self.client.cancel_all_goals()
                cancel_system()
            else:
                self.goal.total_missions = order_check
                self.client.send_goal(
                    self.goal, done_cb=self.result_cb, feedback_cb=self.check_process)

                # Time out duration, can break after a long time not responding
#                self.client.wait_for_result(rospy.Duration(60*60))
                # state can be check here
                threadLock.acquire()
                excuted_number += 1
                threadLock.release()
#           print(self.client.get_state())
#            if excuted_number==2:
#                os._exit(0)


class FeedbackSoc():

    def __init__(self):
        IP_PROT = ('', 9998)
        self.ss = socket(AF_INET, SOCK_STREAM)
        self.ss.setsockopt(SOL_SOCKET, SO_REUSEADDR, 1)
        self.ss.bind(IP_PROT)
        self.ss.listen(5)

    def server(self):

        while not rospy.is_shutdown():
            try:
                conn, addr = self.ss.accept()
                data = conn.recv(1024)
#                print(data,data.decode('utf-8'),data.encode('utf-8'))
                if data == 'Ready to receive':
                    conn.send('ready')

                    while not rospy.is_shutdown():
                        mis_feedback = feedbackq.get()
                        rospy.sleep(1)
                        conn.send(mis_feedback)

            except Exception, e:
                print e

        conn.close()
        self.ss.close()


def inicallback():
    try:
        msg = rospy.wait_for_message("/mm_initialize/state", String, 200)
        if msg.data == "initialized":
            feedbackq.put("initialized")
            print("initialized")
        else:
            feedbackq.put("lowBattery")
            print("may not be initialized or low battery")
    except:
        feedbackq.put("may not be initialized or low battery")
        print("may not be initialized or low battery")

# def cancelcallback(msg):
#    print("received: ",msg.data)
#    if msg.data == '123cancel':
#        orderq.put(msg.data)


def bmscallback(bms_msg):
    global mission_number, excuted_number
    if mission_number > excuted_number:
        pass
    elif bms_msg.rsoc < 40 and bms_msg.chargingcur < 200:
        threadLock.acquire()
        orderq.put('goCharge')
        mission_number += 1
        threadLock.release()
    rospy.sleep(60)


def cancel_system():
    print('cancel')
    # for trans
    # trans_pub(GoalID())
    # rospy.sleep(0.2)
    # for arm
    # power_pub.publish(UInt8(4))
    # rospy.sleep(0.2)
    # for motor
    rospy.sleep(0.2)
    motor_stop_pub.publish(String("stop"))
    rospy.sleep(0.2)
    cancel_pub.publish(Int8(1))
    rospy.sleep(0.2)


def broadcast():
    broadcaster = tf2_ros.StaticTransformBroadcaster()
    static_transformStamped = geometry_msgs.msg.TransformStamped()

    static_transformStamped.header.stamp = rospy.Time.now()
    static_transformStamped.header.frame_id = "base_footprint"
    static_transformStamped.child_frame_id = "world"

    static_transformStamped.transform.translation.x = 0.0
    static_transformStamped.transform.translation.y = 0.0
    static_transformStamped.transform.translation.z = 2.0

    static_transformStamped.transform.rotation.x = 0.0
    static_transformStamped.transform.rotation.y = 0.0
    static_transformStamped.transform.rotation.z = 0.0
    static_transformStamped.transform.rotation.w = 1.0

    broadcaster.sendTransform(static_transformStamped)


def motor_initial():
    motorGoal = motorCommand()
    motorGoal.workstate = 'home'
    motorGoal.degree = 0.0
    rospy.sleep(0.2)
    pubMotorCommand.publish(motorGoal)
    rospy.loginfo(motorGoal)


def main():
    print("wait for initilized")
#    inicallback()
    broadcast()
    motor_initial()
    ros_execute = Ros_action()
#    back2win = FeedbackSoc()


    local = SimpleEcho('10.161.8.142', 8888, 0)
    thread1=threading.Thread(target=local.localtest)

#------------------------test for report----------------
#    rospy.sleep(3)   
#    rospy.loginfo("Initilized, Ready to receive")
#    rospy.sleep(5)
#    print("('192.168.3.130',56521),connected')")
#    print("('192.168.3.104',56605),connected')")
#    rospy.sleep(1)

#    rospy.loginfo("Robot 1:['<<start', 'moveToEquipment//101', 'checkSwitchStatus//0//0//101', 'operateRemoteSwitch//open//101', 'operateHandcartSwitch//testToWork//101', 'checkSwitchStatus//1//0//101', 'operateKnifeSwitch//close//101', 'checkSwitchStatus//1//1//101', 'operateRemoteSwitch//close//101', 'end>>']")
#    rospy.loginfo("Robot 2:['<<start', 'moveToEquipment//1003', 'checkSwitchStatus//1//1//1003', 'operateRemoteSwitch//open//1003', 'operateKnifeSwitch//open//1003', 'checkSwitchStatus//1//0//1003', 'operateHandcartSwitch//workToTest//1003', 'checkSwitchStatus//0//0//1003', 'operateRemoteSwitch//close//1003', 'end>>']")
#------------------------test for report----------------

#    thread1 = threading.Thread(target=websocketserver)

    thread2 = threading.Thread(target=ros_execute.execution)
#    thread3 = threading.Thread(target=back2win.server)

    threads.append(thread1)
    threads.append(thread2)
#    threads.append(thread3)

    for t in threads:
        t.start()
    for t in threads:
        t.join()

    print("退出主线程")


if __name__ == '__main__':
    try:
        rospy.init_node('robot_execution_client')
#        rospy.Subscriber('/mm_cancel_test/state', String, cancelcallback)
        rospy.Subscriber('/bms_data', Battery, bmscallback)
        power_pub = rospy.Publisher(
            '/mm_powerup/switch_command', UInt8, queue_size=10)
        pubMotorCommand = rospy.Publisher(
            '/mm_motor/motor_command', motorCommand, queue_size=1000)
        trans_pub = rospy.Publisher('/move_base/cancel', GoalID, queue_size=10)
        cancel_pub = rospy.Publisher(
            '/mm_decision/cancel', Int8, queue_size=10)
        motor_stop_pub = rospy.Publisher("/mm_motor/motor_stop",String, queue_size=10)
        main()
    except rospy.ROSInterruptException:
        pass
