#!/usr/bin/env python
# -*- coding: UTF-8 -*-
import rospy
import sys
from geometry_msgs.msg import PoseStamped,Pose, Point, Quaternion
from std_msgs.msg import String,Int8
from mm_robot_decision.msg import pose4
from mm_endmotor_control.msg import motorCommand
import linecache

class mm_decision():
    
    def __init__(self):
        rospy.init_node('mm_decision',anonymous=True) 
        self.num=0
        rospy.sleep(1)
        self.pubArm=rospy.Publisher('/mm_arm/goal',pose4,queue_size=10)
        self.readMission()
         
        self.subTrans=rospy.Subscriber('/mm_trans/isArrived',Int8,self.transCallback)
        self.subArm=rospy.Subscriber('/mm_arm/isArrived',Int8,self.armCallback)
        
        self.subSwitchPose=rospy.Subscriber('/mm_visual/switchPose',pose4,self.visualPoseCallback)
        self.subEndMotor=rospy.Subscriber('/mm_motor/isArrived',Int8,self.endMotorCallback)

	self.subRecognization=rospy.Subscriber('/mm_recognization/lightState',pose4,self.recognizationCallback)

        rospy.spin()
        
        #打开任务文件，循环读取任务并执行

    def readMission(self):
        self.num=self.num+1
        path='/home/youibot/mrobot/src/mmrobot/mm_control/mm_robot_decision/scripts/mission.txt'
        f=open(path)
        count=len(f.readlines())
       
        
        if self.num<=count:
            print(self.num)
            task=linecache.getline(path,self.num)
            self.taskList=task.split('//')
            print(task)
            print(self.taskList)
       
            #根据任务进入对应子函数进行任务的执行 
            if self.taskList[0]=='mm_trans':
                self.mm_trans()
            elif self.taskList[0]=='mm_arm':
                self.mm_arm()
            elif self.taskList[0]=='mm_visual_position':
                self.mm_visual_position()
            elif self.taskList[0]=='mm_end_motor':
                self.mm_end_motor()
	    elif self.taskList[0]=='mm_recognization':
		self.mm_recognization()
           
        else:
            rospy.loginfo('Mission is over, go back to charge')
            self.pubChargeMsg=rospy.Publisher('/is_charge', Int8, queue_size=10)
            chargeData=1
            rospy.sleep(0.5)
            self.pubChargeMsg.publish(chargeData)
            
            sys.exit(0) 
    
    def transCallback(self,data):
            transIsArrived=data.data
            if (transIsArrived==1):
                rospy.loginfo('Trans is arrived')
		rospy.sleep(2)
                self.readMission()
           
            else:
                rospy.loginfo('Trans is failed')
                sys.exit(0)       
           
            

    def mm_trans(self):
        
         #定义底盘要发布和订阅的消息
        self.pubTrans=rospy.Publisher('/mm_trans/goal',PoseStamped,queue_size=10)      
        
        #读取底盘的目标位姿
        transGoal=PoseStamped()
        transGoal.header.stamp=rospy.Time.now()
        transGoal.header.frame_id=self.taskList[1]
        TransPoint=tuple(eval(self.taskList[2]))
        TransQuaternion=tuple(eval((self.taskList[3])))
        transGoal.pose=Pose(
                    Point(TransPoint[0],TransPoint[1],TransPoint[2]),
                    Quaternion(TransQuaternion[0],TransQuaternion[1],TransQuaternion[2],TransQuaternion[3]))
        
        
        
        #发布Trans底盘运动目标
        rospy.sleep(1)
        self.pubTrans.publish(transGoal)
        rospy.loginfo(transGoal)
       
       
    def armCallback(self,data):
        armIsArirved=data.data
        if armIsArirved==1:
            rospy.loginfo('Arm is arrived')
            self.readMission()
            
                
        else:
            rospy.loginfo('Arm is failed')
            sys.exit(0)
       
    def mm_arm(self):
        #定义机械臂要发布消息
               
        
        
        #读取机械臂的目标位置并发送
        

        armGoal=pose4()
        armGoal.state=self.taskList[1]
        armGoal.wct=int(self.taskList[2])
        if armGoal.state=='workSpacePlanning':
            armPoint=tuple(eval(self.taskList[3]))
            armQuaternion=tuple(eval(self.taskList[4]))
            armGoal.x=armPoint[0]
            armGoal.y=armPoint[1]
            armGoal.z=armPoint[2]
        
            armGoal.a=armQuaternion[0]
            armGoal.b=armQuaternion[1]
            armGoal.c=armQuaternion[2]
        elif armGoal.state=='jointSpacePlanning':
            armJoint=tuple(eval(self.taskList[3]))            
            armGoal.x=armJoint[0]
            armGoal.y=armJoint[1]
            armGoal.z=armJoint[2]
            armGoal.a=armJoint[3]
            armGoal.b=armJoint[4]
            armGoal.c=armJoint[5]

     	elif armGoal.state=='MoveInTool':
            armPoint=tuple(eval(self.taskList[3]))
            armQuaternion=tuple(eval(self.taskList[4]))
            armGoal.x=armPoint[0]
            armGoal.y=armPoint[1]
            armGoal.z=armPoint[2]
        
            armGoal.a=armQuaternion[0]
            armGoal.b=armQuaternion[1]
            armGoal.c=armQuaternion[2]
            
            
        print(armGoal)
        rospy.sleep(0.5)
        self.pubArm.publish(armGoal)
        rospy.sleep(0.1)
        
    #定义 视觉定位对应的回调函数
    
    def  visualPoseCallback(self,calcuPose4):
        if calcuPose4.state=='WorkSpacePlanning':
            
            armSwitchPose=pose4()
            armSwitchPose.state=calcuPose4.state
            
            
            armSwitchPose.wct=1
            armSwitchPose.x=calcuPose4.x
            armSwitchPose.y=calcuPose4.y
            armSwitchPose.z=calcuPose4.z
            armSwitchPose.a=calcuPose4.a
            armSwitchPose.b=calcuPose4.b
            armSwitchPose.c=calcuPose4.c
            rospy.loginfo(armSwitchPose)
            rospy.sleep(0.5)
            self.pubArm.publish(armSwitchPose)
            
        elif calcuPose4.state=='MoveInTool':
            
            armSwitchPose=pose4()
            armSwitchPose.state=calcuPose4.state
            
            
            armSwitchPose.wct=1
            armSwitchPose.x=calcuPose4.x
            armSwitchPose.y=calcuPose4.y
            armSwitchPose.z=calcuPose4.z
            armSwitchPose.a=calcuPose4.a
            armSwitchPose.b=calcuPose4.b
            armSwitchPose.c=calcuPose4.c
            rospy.loginfo(armSwitchPose)
            rospy.sleep(0.5)
            self.pubArm.publish(armSwitchPose)    
        else:
            rospy.loginfo('switchPosition is failed')
            sys.exit(0)
        
        
        
    #定义视觉定位子函数
    def mm_visual_position(self):
        #定义要发布的消息类型
        self.pubCalcuPose=rospy.Publisher('/mm_visual/calcuPose',String,queue_size=20)
        
        #读取需要定位的开关类型
        calcuSwitchOrder=self.taskList[1]
        rospy.loginfo("I try to publish")
        rospy.sleep(1)
        self.pubCalcuPose.publish(calcuSwitchOrder)
        rospy.loginfo(calcuSwitchOrder)
        
        
        
    #定义电机的回调函数
    def endMotorCallback(self,data):
        motorIsArirved=data.data
        if motorIsArirved==1:
            rospy.loginfo('Motor is arrived')
            self.readMission()
            
                
        else:
            rospy.loginfo('Arm is failed')
            sys.exit(0)
        
    
    def mm_end_motor(self):
        #定义电机发布的消息类型
        self.pubMotorCommand=rospy.Publisher('/mm_motor/motor_command',motorCommand,queue_size=1000)
        
        #读取需要转动的电机角度
        motorTheta=eval(self.taskList[1])
        
        rospy.loginfo("I try to publish motor theta")
        rospy.sleep(1)
        self.pubMotorCommand.publish(motorTheta)
        rospy.loginfo(motorTheta)

    def recognizationCallback(self,LightState):
	if LightState.x>0 and LightState.y<1:
            rospy.loginfo('开关处于远方状态')
            self.readMission()		
        elif LightState.x<1 and LightState.y>0:
            rospy.loginfo('开关处于就地状态')
            self.readMission()	
        else:
            rospy.loginfo('LightRecognization is failed')
            sys.exit(0)
        
    def mm_recognization(self):
        self.pubRecogOrder=rospy.Publisher('/mm_recognization/lightStateOrder',String,queue_size=20)
        
        #读取需要定位的开关类型
        lightRecogOrder=self.taskList[1]
        rospy.loginfo("I try to publish")
        rospy.sleep(1)
        self.pubRecogOrder.publish(lightRecogOrder)
        rospy.loginfo(lightRecogOrder)
     

if __name__=='__main__':
    try:
        #Decision=mm_decision()
        #Decision.sendTransGoal()
        rospy.sleep(5)
        mm_decision()
        
    except rospy.ROSInterruptException:
        rospy.logwarn("mm_decision is finished.")
   
