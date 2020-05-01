#!/usr/bin/env python
# -*- coding: UTF-8 -*-
import rospy
import sys
import tf
from socket import *
import threading
import Queue
from youibot_msgs.msg import Battery

threadLock = threading.Lock()
threads = []
feedbackq=Queue.Queue()

class FeedbackSoc():

    def __init__(self):
        IP_PROT = ('', 9999)
        self.ss = socket(AF_INET, SOCK_STREAM)
        self.ss.setsockopt(SOL_SOCKET, SO_REUSEADDR, 1)
        self.ss.bind(IP_PROT)
        self.ss.listen(5)


    def server(self):

        while not rospy.is_shutdown():   
            conn, addr = self.ss.accept()
            try:
                data = conn.recv(1024)
#                print(data,data.decode('utf-8'),data.encode('utf-8'))
                if data == 'Ready to receive':
                    conn.send('ready')
                    feedbackq.queue.clear()
                    while not rospy.is_shutdown():
                        info_feedback=feedbackq.get()
                        conn.send(info_feedback)
                      
            except Exception, e:
                print e

        conn.close()
        self.ss.close()


class PositionInfo():

    def __init__(self):
        self.listener = tf.TransformListener()

    def tflistener(self):
        self.listener.waitForTransform("/map", "/base_footprint", rospy.Time(), rospy.Duration(1000.0)) 
        while not rospy.is_shutdown():
            try:  
                (trans1, rot1) = self.listener.lookupTransform("/map", "/base_footprint", rospy.Time(0))
                rpy = tf.transformations.euler_from_quaternion(
                [rot1[0], rot1[1], rot1[2], rot1[3]])
                poi_str = 'p//%.4f//%.4f//%.4f' % (trans1[0],trans1[1],rpy[2]*180.0/3.1415926)
#                print(trans1[0],trans1[1],rpy[2]*180.0/3.1415926)
                threadLock.acquire()
                feedbackq.put(poi_str)
                threadLock.release()
                rospy.sleep(2)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

def main():
    rospy.init_node("mm_feedback2win",anonymous=True)
    rospy.Subscriber('/bms_data', Battery, bmscallback)
    back2win=FeedbackSoc()
    pp = PositionInfo()
    thread1=threading.Thread(target=back2win.server)
    thread2=threading.Thread(target=pp.tflistener)
    threads.append(thread1)
    threads.append(thread2)
    for t in threads:
        t.start()
    for t in threads:
        t.join()


def bmscallback(bmsinfo):
#    print(bmsinfo.rsoc)
    threadLock.acquire()
    feedbackq.put('b//'+str(bmsinfo.rsoc))
    threadLock.release()
    rospy.sleep(10)    



if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
