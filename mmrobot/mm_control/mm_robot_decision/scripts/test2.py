#! /usr/bin/env python
# -*- coding: utf-8 -*-

import threading
import Queue
import time
import logging

threadLock = threading.Lock()
threads = []
orderq=Queue.Queue()


logging.basicConfig(level=logging.DEBUG, 
				format='%(asctime)s \"%(filename)s\"[line:%(lineno)d] %(levelname)s %(message)s', 
				datefmt='%a, %d %b %Y %H:%M:%S', 
				filename= "/home/youibot/mrobot/robotlogs/"+time.strftime('%Y.%m.%d',time.localtime(time.time()))+"-ubuntu_server.log", 
				filemode='a')

def test1():
    while 1:
        try:
            result1 = orderq.get(timeout = 2)
        except Queue.Empty:
            print("timeout")
        logging.info("receive command: "+'\r\n'+result1) 

def test2():
    while 1:
        result2 = "test";
        threadLock.acquire()
        orderq.put(result2)
        threadLock.release()
        time.sleep(5)


def main():    
    #rospy.Subscriber('/mm_initialize/state', String, inicallback)
    thread1=threading.Thread(target=test1)
    thread2=threading.Thread(target=test2)
    threads.append(thread1)
    threads.append(thread2)
    for t in threads:
        t.start()
    for t in threads:
        t.join()

main()
