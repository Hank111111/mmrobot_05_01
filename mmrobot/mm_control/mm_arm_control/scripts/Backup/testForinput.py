#!/usr/bin/env python
#coding=utf-8
import rospy
import tf
#导入自定义的数据类型
from tf.transformations import *
from hello import pose4

def talker():
    #Publisher 函数第一个参数是话题名称，第二个参数 数据类型，现在就是我们定义的msg 最后一个是缓冲区的大小
    #queue_size: None（不建议）  #这将设置为阻塞式同步收发模式！
    #queue_size: 0（不建议）#这将设置为无限缓冲区模式，很危险！
    #queue_size: 10 or more  #一般情况下，设为10 。queue_size太大了会导致数据延迟不同步。
    pub = rospy.Publisher('Inputpose', pose4 , queue_size=10)
    rospy.init_node('testForinput', anonymous=True)
    rospy.sleep(5)                 #等待moveit初始化和回到待机位置，时间为估算
    while not rospy.is_shutdown():  #主循环，用于发布目标位姿      
          state='working'
          wct=True
          x=-0.410286
          y=-0.801217
          z=0.870228
          a = -1.58375
          b = -0.02355
          c = -3.06498
          pub.publish(pose4(state,wct,x,y,z,a,b,c))
          rospy.sleep(20)
        
          wct=True
          x=-0.267551
          y=-0.795308
          z=0.86936
          a = -1.60868
          b = -0.003176
          c = -3.12238
          pub.publish(pose4(state,wct,x,y,z,a,b,c))
          rospy.sleep(20)


          wct=True
          x=-0.0897181
          y=-0.852683
          z=0.151613
          a = -1.60868
          b = -0.003176
          c = -3.12238
          pub.publish(pose4(state,wct,x,y,z,a,b,c))
          rospy.sleep(20)

          #x = -0.2563
          #wct=True
          #pub.publish(pose4(state,wct,x,y,z,a,b,c))
          #rospy.sleep(5)
    
          #z = 0.7763
          #pub.publish(pose4(state,wct,x,y,z,a,b,c))
          #rospy.sleep(5)

          #x = -0.08
          #y = -0.61
          #z = 0.148
          #pub.publish(pose4(state,wct,x,y,z,a,b,c))
          #rospy.sleep(15)

          #工具末端套进转轴，有风险，无手动校准请勿使用
          #y = -0.685
          #pub.publish(pose4(state,wct,x,y,z,a,b,c))
          #rospy.sleep(10)

          #y = -0.61
          #pub.publish(pose4(state,wct,x,y,z,a,b,c))
          #rospy.sleep(10)


          state = 'shutdown'
          pub.publish(pose4(state,wct,x,y,z,a,b,c))
          rospy.sleep(5)
          exit(0)

if __name__ == '__main__':
    talker()
 
