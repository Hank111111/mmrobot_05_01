#!/usr/bin/env python
# -*- coding: UTF-8 -*-
import rospy
from std_msgs.msg import Int8, String, Int8MultiArray, UInt8
import actionlib
import os
import threading
import MySQLdb
import Queue
import tf
from socket import *
from mm_robot_decision.msg import pose4, VisualAppRequest, VisualAppResponse
from mm_endmotor_control.msg import motorCommand
from geometry_msgs.msg import PoseStamped,Pose, Point, Quaternion
from mm_robot_decision.msg import robot_executionAction, robot_executionGoal, robot_executionResult, robot_executionFeedback
from youibot_msgs.msg import Battery
from youibot_move_action.srv import move_action


class Robot_decision():

    def __init__(self):
        rospy.init_node('mm_decision',anonymous=True) 
        self.transqueue = Queue.Queue()
        self.armqueue = Queue.Queue()
        self.visualqueue = Queue.Queue()
        self.qrqueue = Queue.Queue()
        self.motorqueue = Queue.Queue()
        self.threadLock = threading.Lock()

#-------------------------------------------------------
        self.equ_sqlid = 1
        self.equ_num = 3
        self.check_mode = False
#-------------------------------------------------------


        self.zerolize()
 	

        self.server = actionlib.SimpleActionServer('robot_execute', robot_executionAction, self.execute, False)
        self.server.start()
        self.result = robot_executionResult()

        self.pubTrans=rospy.Publisher('/mm_trans/goal',PoseStamped,queue_size=10)
        self.pubArm=rospy.Publisher('/mm_arm/goal',pose4,queue_size=10)
        self.pubMotorCommand=rospy.Publisher('/mm_motor/motor_command',motorCommand,queue_size=10)
        self.pubVisualApp=rospy.Publisher('/mm_visual/wrapper/request', VisualAppRequest ,queue_size=10)
        self.power_pub = rospy.Publisher('/mm_powerup/switch_command', UInt8, queue_size=10) 
        self.pubChargeMsg=rospy.Publisher('/is_charge', Int8, queue_size=10)

         
        self.subTrans=rospy.Subscriber('/mm_trans/isArrived',Int8,self.transCallback)
        self.subArm=rospy.Subscriber('/mm_arm/isArrived',Int8,self.armCallback)

        self.subVisualApp=rospy.Subscriber('/mm_visual/wrapper/response',VisualAppResponse,self.visualCallback)

        self.subEndMotor=rospy.Subscriber('/mm_motor/isArrived',Int8,self.endMotorCallback)
        self.subProReset=rospy.Subscriber('/mm_arm/protected',Int8,self.resetProCallback)
        self.subcancel=rospy.Subscriber('/mm_decision/cancel',Int8,self.cancelCallback)

#        rospy.wait_for_service("/youibot_move_action_Node/youibot_move_action",10)
#        self.charge_client = rospy.ServiceProxy("/youibot_move_action_Node/youibot_move_action", move_action)


        print("active thread count = {}".format(threading.active_count()))
        rospy.sleep(1)
        rospy.spin()

    def zerolize(self):
        self.transqueue.queue.clear()
        self.armqueue.queue.clear()
        self.visualqueue.queue.clear()
        self.motorqueue.queue.clear()


    def execute(self, command):
        self.db = MySQLdb.connect(host="localhost", port=3306, user="mrobot",passwd="123456",db="Power_distribution_room",charset="utf8" )
        self.cursor = self.db.cursor()
        sql = """SELECT toolid FROM Power_distribution_room.robot_state WHERE id = 1"""
        self.cursor.execute(sql)
        results = self.cursor.fetchone()
        self.toolid=int(results[0])
        print(self.toolid)  


        self.unique_id_object_type_map = {0: "QRCode",
                                 1: "QRCode",
                                 2: "knifeSwitch",
                                 3: "remoteSwitch",
                                 4: "handcartSwitch",
                                 5: "pointMeter",
                                 6: "light",
                                 7: "digitalMeter"}
        # create object_type_unique_id_map
        self.object_type_unique_id_map ={}
        for unique_id in self.unique_id_object_type_map:
            if unique_id not in self.object_type_unique_id_map.keys():
                self.object_type_unique_id_map[self.unique_id_object_type_map[unique_id]] = []
            self.object_type_unique_id_map[self.unique_id_object_type_map[unique_id]].append(unique_id)


  
        mission = command.total_missions
        process = robot_executionFeedback()
        



        for i in range(0,len(mission)):
            spe_mission = mission[i].split('//')
            print(spe_mission)
            self.current_cabname = spe_mission[len(spe_mission)-1] 
            process.current_mission = (spe_mission[0]+'//'+self.current_cabname)
            process.mission_state = 0
            self.zerolize()



            if spe_mission[0]=='moveToEquipment':
                
 #               try:
 #                   bms_msg = rospy.wait_for_message('/bms_data', Battery, 10)
 #                   if bms_msg.rsoc < 40:
 #                       self.server.set_aborted()
 #                       break
 #                   elif bms_msg.chargingcur > 200:
 #                       resp = self.charge_client(-1,0.0)
 #                       rospy.sleep(10)
 #                       if resp.result_flag == 1:
 #                           print 'discharge success'
 #               except rospy.ServiceException,e:
 #                   print "Service call failed: %s"%e

                self.server.publish_feedback(process)               
                process.mission_state = self.mm_moveToEquipment(spe_mission[len(spe_mission)-1])
            elif spe_mission[0]=='checkDigitalMeter':
                self.server.publish_feedback(process)
                process.mission_state = self.mm_checkDigitalMeter()
            elif spe_mission[0]=='checkPointMeter':
                self.server.publish_feedback(process)
                process.mission_state = self.mm_checkPointMeter(int(spe_mission[1]))

            elif spe_mission[0]=='checkSwitchStatus':
                self.server.publish_feedback(process)
                process.mission_state = self.mm_checkSwitchStatus(int(spe_mission[1]),int(spe_mission[2]))

            elif spe_mission[0]=='operateRemoteSwitch':
                process.current_mission = (spe_mission[0]+','+spe_mission[1]+'//'+self.current_cabname)
                self.server.publish_feedback(process)
                process.mission_state = self.mm_operateRemoteSwitch(spe_mission[1],int(spe_mission[2]))
            elif spe_mission[0]=='operateHandcartSwitch':
                process.current_mission = (spe_mission[0]+','+spe_mission[1]+'//'+self.current_cabname)
                self.server.publish_feedback(process)
                process.mission_state = self.mm_operateHandcartSwitch(spe_mission[1])
            elif spe_mission[0]=='operateKnifeSwitch':
                process.current_mission = (spe_mission[0]+','+spe_mission[1]+'//'+self.current_cabname)
                self.server.publish_feedback(process)
                process.mission_state = self.mm_operateKnifeSwitch(spe_mission[1])
				
            elif spe_mission[0] == 'demo':
                process.mission_state = self.demo(process)  
                 
            elif spe_mission[0] == 'recover':
                process.current_mission = 'recover'              
                self.server.publish_feedback(process)
                process.mission_state = self.recover()   
		
            elif spe_mission[0] == 'shutdown':
                process.current_mission = 'shutdown'              
                self.server.publish_feedback(process)
                process.mission_state = self._shutdown() 

            elif spe_mission[0] == 'goCharge':     
                process.current_mission = 'charge'              
                self.server.publish_feedback(process)
		

            if process.mission_state != 1 and spe_mission[0] != 'goCharge':
                self.server.set_aborted()
                break
            self.server.publish_feedback(process)


            if (i == len(mission)-1):
                if spe_mission[0] in ['moveToEquipment','checkDigitalMeter','checkPointMeter','checkSwitchStatus','operateRemoteSwitch','operateHandcartSwitch','operateKnifeSwitch']:
         
                    test = self.mm_changetool(self.toolid,0)
                    if test < 0:
                        self.server.set_aborted()
                        break

                    test = self.mm_standby()
                    if test < 0:
                        self.server.set_aborted()
                        break

                    test = self.mm_home()
                    if test < 0:
                        self.server.set_aborted()
                        break

                rospy.loginfo('Mission is over, go back')
                self.result.missions_done = len(mission)
                self.server.set_succeeded(self.result)
                self.db.close()


                transGoal=PoseStamped()
                transGoal.header.stamp=rospy.Time.now()
                transGoal.header.frame_id='trans'
                transGoal.pose=Pose(
                            Point(0.60,-0.45,0.0),
                            Quaternion(0.0,0.0,0.0,1.0))
#                self.pubTrans.publish(transGoal)
#                rospy.loginfo(transGoal)
#                trans_result = self.mission_check(self.transqueue,60)	
 
                if spe_mission[0] == 'goCharge': 
                    try:
                        bms_msg = rospy.wait_for_message('/bms_data', Battery, 10)
                        if bms_msg.chargingcur > 200:
                            pass
                        else:
                            resp = self.charge_client(1,0.0)
                            if resp.result_flag == 1:
                                print 'charge success'
                    except rospy.ServiceException,e:
                        print "Service call failed: %s"%e


            elif mission[i+1].split('//')[0] == 'moveToEquipment':
                test = self.mm_standby()
                if test < 0:
                    break
                test = self.mm_home()
                if test < 0:
                    break
                


    def mm_moveToEquipment(self,equip_id):
        print('now executing moveToEquipment...')
        sql = """SELECT type,id,poi_x, poi_y, poi_c, poi_w FROM Power_distribution_room.equipment WHERE name = '%s'""" % (equip_id)
        self.cursor.execute(sql)
        # ��ȡ��¼�б�
        results = self.cursor.fetchone()
        self.equ_sqlid=results[0]
        self.equ_num=results[1]

        transGoal=PoseStamped()
        transGoal.header.stamp=rospy.Time.now()
        transGoal.header.frame_id='trans'
        transGoal.pose=Pose(
                    Point(results[2],results[3],0.0),
                    Quaternion(0.0,0.0,results[4],results[5]))
        
        

        #����Trans�����˶�Ŀ��
#        self.pubTrans.publish(transGoal)
#        rospy.loginfo(transGoal)
#        trans_result = self.mission_check(self.transqueue,60)
#        if trans_result < 0:
#            return -1
 
        test = self.mm_standby()
        if test < 0:
            return -1
        print('standby, continue...')

        
        results = self.mm_sql(0)
        self.mm_arm('jointSpacePlanning',results)
        arm_result = self.mission_check(self.armqueue,20)
        if arm_result < 0:
            return -1
        print('arm arrived, continue...')
        self.pubVisualApp.publish(VisualAppRequest("QRCode",0,self.equ_num,self.equ_sqlid))
        vr = self.mission_check(self.visualqueue,60)
        if vr[0] == "QRCode" and vr[-1] == True:
            pass
        else:
            return -1



        results = self.mm_sql(1)
        self.mm_arm('jointSpacePlanning',results)
        arm_result = self.mission_check(self.armqueue,20)
        if arm_result < 0:
            return -1
        print('arm arrived, continue...')
        self.pubVisualApp.publish(VisualAppRequest("QRCode",1,self.equ_num,self.equ_sqlid))
        vr = self.mission_check(self.visualqueue,60)
        if vr[0] == "QRCode" and vr[-1] == True:
            return 1
        else:
            return -1
 


    def mm_checkDigitalMeter(self):
        print('now executing checkDigitalMeter...')
        test = self.move_and_capture("digitalMeter")
        if test < 0:
            return -1
        vr = self.mission_check(self.visualqueue,60)
        if vr[0] == "digitalMeter" and vr[-1] == True:
            print (vr[-3],len(vr[-3]))
            return 1
        else:
            return -1

    def mm_checkPointMeter(self,flag1):
        print('now executing checkPointMeter...')
        test = self.move_and_capture("pointMeter")
        if test < 0:
            return -1
        vr = self.mission_check(self.visualqueue,60)
        if vr[0] == "pointMeter" and vr[-1] == True:
            print (vr[-2],len(vr[-2]))
            return 1
        else:
            return -1


    def mm_checkSwitchStatus(self,flag1,flag2):
        print('now executing checkSwitchStatus...')
        test = self.move_and_capture("light")
        if test < 0:
            return -1
        vr = self.mission_check(self.visualqueue,60)
        if vr[0] == "light" and vr[-1] == True:
            print (vr[-2],len(vr[-2]))
            if vr[-2][0] == flag1 and vr[-2][1] == flag2:
                return 1
            elif self.check_mode == True:
                rospy.logwarn('please check if state is right')
                msg = rospy.wait_for_message("/mm_state_check", Int8, 120)
                if msg.data == 1:
                    return 1
                else:
                    return -1
            else:
	            return -1
        else:
            return -1


    def mm_operateRemoteSwitch(self,order,current_flag):  
        print('now executing operateRemoteSwitch...')
    
        sql = """SELECT orientation_status_0,orientation_status_1 FROM Power_distribution_room.id_name_convert WHERE (object_unique_id_on_equipment = %d and equipment_type = %d);""" % (self.object_type_unique_id_map["remoteSwitch"][0],self.equ_sqlid)
        self.cursor.execute(sql)
        status_deg = self.cursor.fetchone()
        print(order)
        if order == 'open':
            bidegree1 = status_deg[0]
            bidegree2 = status_deg[1]
        elif order == 'close':
            bidegree1 = status_deg[1]
            bidegree2 = status_deg[0]
        else:
            return -1



        test = self.move_and_capture("remoteSwitch")

#---------test_svm-------
	    #return -1
#------------------------


        if test < 0:
            return -1
        vr = self.mission_check(self.visualqueue,60)
        if vr[0] == "remoteSwitch" and vr[-1] == True: 
            if current_flag == (vr[-2])[0]:
                # only for test of vision, not operate
                # ------------------------------------
                print ('right state')
                return 1
                # ------------------------------------
                pass
            elif self.check_mode == True:
                rospy.logwarn('please check if state is right')
                msg = rospy.wait_for_message("/mm_state_check", Int8, 120)
                if msg.data == 1:
                    current_flag = (vr[-2])[0]
#                    if msg == -1:
#                        return -1
            else:
                return -1
        else:
            return -1   

        if order == 'open' and current_flag == 1:
            return 1
        elif order == 'close' and current_flag == 0:
            return 1
        else:
            pass


        if self.toolid == 1:
            pass
        else:
            test = self.mm_changetool(self.toolid,1)
            if test < 0:
                return -1

          
            test = self.move_and_capture("remoteSwitch")
            if test < 0:
                return -1
            vr = self.mission_check(self.visualqueue,60)
            if vr[0] == "remoteSwitch" and vr[-1] == True:
                print(vr[-2])
                
            else:
                return -1
            print('camera arrived, continue...')




        self.mm_motor('move',bidegree1)
        motor_result = self.mission_check(self.motorqueue,15)
        if motor_result < 0:
            return -1
        print('motor arrived, continue...')	

        self.mm_arm(vr[1],vr[2:9])
        arm_result = self.mission_check(self.armqueue,30)
        if arm_result < 0:
            return -1
        print('arm arrived, continue...')

        self.mm_motor('move',bidegree2)
        motor_result = self.mission_check(self.motorqueue,15)
        if motor_result < 0:
            return -1
        print('motor arrived, continue...')	

        self.mm_arm('MoveInTool',[0.0,0.0,-0.1,0.0,0.0,0.0,1.0])
        arm_result = self.mission_check(self.armqueue,20)
        if arm_result < 0:
            return -1
        print('arm arrived, continue...')


        test = self.move_and_capture("remoteSwitch")
        if test < 0:
            return -1
        vr = self.mission_check(self.visualqueue,60)
        if vr[0] == "remoteSwitch" and vr[-1] == True: 
            if self.check_mode == True:
                if order == 'open' and (vr[-2])[0] == 1:
                    return 1
                elif order == 'close' and (vr[-2])[0] == 0:
                    return 1
                else:
                    rospy.logwarn('please check if state is right')
                    msg = rospy.wait_for_message("/mm_state_check", Int8, 120)
                    if msg.data == 1:
                        return 1
                    else:
                        return -1
            else:
                return 1

        else:
            return -1  
        


    def mm_operateHandcartSwitch(self,temp):
        print('now executing operateHandcartSwitch...')
        test = self.mm_changetool(self.toolid,2)
        if test < 0:
            return -1

        test = self.move_and_capture("handcartSwitch")
        if test < 0:
            return -1
        vr = self.mission_check(self.visualqueue,60)
        if vr[0] == "handcartSwitch" and vr[-1] == True:
            print(vr[-2])
        else:
            return -1	
        print('camera arrived, continue...')
#        '''
#        self.mm_motor('move',0)
#        motor_result = self.mission_check(self.motorqueue,15)
#        if motor_result < 0:
#            return -1
#        print('motor arrived, continue...')	
#        '''
        self.mm_arm(vr[1],vr[2:9])
        arm_result = self.mission_check(self.armqueue,30)
        if arm_result < 0:
            return -1
        print('arm arrived, continue...')

        if temp == "testToWork":
            direction = 1
        elif temp == "workToTest":
            direction = -1

        self.mm_motor('jog',direction)
        motor_result = self.mission_check(self.motorqueue,40)
        if motor_result < 0:
            return -1
        print('motor arrived, continue...')	

        self.mm_arm('MoveInTool',[0.0,0.0,-0.25,0.0,0.0,0.0,1.0])
        arm_result = self.mission_check(self.armqueue,30)
        if arm_result < 0:
            return -1
        print('arm arrived, continue...')

        self.mm_motor('home',0)
#        motor_result = self.mission_check(self.motorqueue,20)
#        if motor_result < 0:
#            return -1
#        print('motor arrived, continue...')

        results = self.mm_sql(0)
        self.mm_arm('jointSpacePlanning',results)
        arm_result = self.mission_check(self.armqueue,20)
        if arm_result < 0:
            return -1
        print('arm arrived, continue...')
        self.pubVisualApp.publish(VisualAppRequest("QRCode",0,self.equ_num,self.equ_sqlid))
        vr = self.mission_check(self.visualqueue,60)
        if vr[0] == "QRCode" and vr[-1] == True:
            pass
        else:
            return -1



        results = self.mm_sql(1)
        self.mm_arm('jointSpacePlanning',results)
        arm_result = self.mission_check(self.armqueue,20)
        if arm_result < 0:
            return -1
        print('arm arrived, continue...')
        self.pubVisualApp.publish(VisualAppRequest("QRCode",1,self.equ_num,self.equ_sqlid))
        vr = self.mission_check(self.visualqueue,60)
        if vr[0] == "QRCode" and vr[-1] == True:
            return 1
        else:
            return -1



        return 1


    def mm_operateKnifeSwitch(self,order):
        print('now executing operateKnifeSwitch...')
        test = self.mm_changetool(self.toolid,1)
        if test < 0:
            return -1
        print(order)

        sql = """SELECT orientation_status_0,orientation_status_1,orientation_status_2 FROM Power_distribution_room.id_name_convert WHERE (object_unique_id_on_equipment = %d and equipment_type = %d);""" % (self.object_type_unique_id_map["knifeSwitch"][0],self.equ_sqlid)
        self.cursor.execute(sql)
        status_deg = self.cursor.fetchone()

        fixdegree = status_deg[0]
    
        if order == 'open':
            bidegree = status_deg[1]
        elif order == 'close':
            bidegree = status_deg[2]
        else:
            return -1
        test = self.move_and_capture("knifeSwitch")
        if test < 0:
            return -1
        vr = self.mission_check(self.visualqueue,60)
        if vr[0] == "knifeSwitch" and vr[-1] == True:
            print(vr[-2])
        else:
            return -1


        print('camera arrived, continue...')

        self.mm_motor('move',fixdegree)
        motor_result = self.mission_check(self.motorqueue,20)
        if motor_result < 0:
            return -1
        print('motor arrived, continue...')	

        self.mm_arm(vr[1],vr[2:9])
        arm_result = self.mission_check(self.armqueue,30)
        if arm_result < 0:
            return -1
        print('arm arrived, continue...')

        self.mm_motor('move',bidegree)
        motor_result = self.mission_check(self.motorqueue,20)
        if motor_result < 0:
            return -1
        print('motor arrived, continue...')	

        self.mm_motor('move',fixdegree)
        motor_result = self.mission_check(self.motorqueue,20)
        if motor_result < 0:
            return -1
        print('motor arrived, continue...')


        self.mm_arm('MoveInTool',[0.0,0.0,-0.1,0.0,0.0,0.0,1.0])
        arm_result = self.mission_check(self.armqueue,20)
        if arm_result < 0:
            return -1
        print('arm arrived, continue...')

        return 1

    def mm_changetool(self,old,new):
        state = []
        state.append([-4.5068,-1.6159,-2.3619,-0.7266,1.5689,0.2081,0]) #above 150 mm, remoteOperator
        state.append([-4.5960,-1.6402,-2.3589,-0.7051,1.5641,0.1185,0]) #above 150 mm, handcartOperator
        #state.append(x: -1.9684,y: -1.5703,z: 2.3462,a: 3.9251,b: -1.5758,c: -0.3974,w: 0])
	#"{state: 'jointSpacePlanning', x: -4.50585, y: -1.61455, z: -2.36260, a: -0.72770, b: 1.56347, c: 0.20826, w: 0.0}"
	#"{state: 'jointSpacePlanning', x: -4.5993, y: -1.6443, z: -2.3562, a: -0.7040, b: 1.5642, c: 0.1154, w: 0.0}"
	#"{state: 'MoveInTool', x: 0.0, y: 0.0, z: 0.15, a: 0.0, b: 0.0, c: 0.0, w: 1.0}"
	#  handcart  "{state: 'jointSpacePlanning', x: -3.195, y: -1.7106, z: -2.2335, a: -2.4109, b: -1.5743, c: -0.0262, w: 0.0}"
	#  standby   "{state: 'jointSpacePlanning', x: -2.7532, y: -0.7891, z: -2.2107, a: -0.1319, b: -0.6042, c: -0.0016, w: 0.0}"
	#  home      "{state: 'jointSpacePlanning', x: -1.5579, y: 0.1263, z: -2.7121, a: -0.4735, b: -1.5758, c: -0.0493, w: 0.0}"
    #[-4.596462194119589, -1.6450002829181116, -2.357105557118551, -0.7020419279681605, 1.5641746520996094, 0.11819741874933243]


        #����������д��������
        if old == new:
            return 0

        if old == 0: 
            self.mm_motor('move',0)
            motor_result = self.mission_check(self.motorqueue,20)
            if motor_result < 0:
                return -1
        elif old in range(1,4):
            self.mm_arm('jointSpacePlanning',state[old-1])
            arm_result = self.mission_check(self.armqueue,20)
            if arm_result < 0:
                return -1

            self.mm_motor('move',-45)
            motor_result = self.mission_check(self.motorqueue,20)
            if motor_result < 0:
                return -1
		
            self.mm_arm('MoveInTool',[0.0,0.0,0.15,0.0,0.0,0.0,1.0])
            arm_result = self.mission_check(self.armqueue,20)
            if arm_result < 0:
                return -1

            self.mm_motor('move',0)
            motor_result = self.mission_check(self.motorqueue,20)
            if motor_result < 0:
                return -1

            self.mm_arm('MoveInTool',[0.0,0.0,-0.15,0.0,0.0,0.0,1.0])
            arm_result = self.mission_check(self.armqueue,20)
            if arm_result < 0:
                return -1
        else:
            return -1

        if new == 0:
            pass           
        elif new in range(1,4):
            self.mm_arm('jointSpacePlanning',state[new-1])
            arm_result = self.mission_check(self.armqueue,20)
            if arm_result < 0:
                return -1

            self.mm_arm('MoveInTool',[0.0,0.0,0.15,0.0,0.0,0.0,1.0])
            arm_result = self.mission_check(self.armqueue,20)
            if arm_result < 0:
                return -1

            self.mm_motor('move',-45)
            motor_result = self.mission_check(self.motorqueue,20)
            if motor_result < 0:
                return -1

            self.mm_arm('MoveInTool',[0.0,0.0,-0.15,0.0,0.0,0.0,1.0])
            arm_result = self.mission_check(self.armqueue,20)
            if arm_result < 0:
                return -1
        else:
            return -1

        self.toolid = new
        sql = """UPDATE `Power_distribution_room`.`robot_state` SET `toolid`='%s' WHERE `id`='1';""" % (new)
        self.cursor.execute(sql)  
        self.threadLock.acquire()
        self.db.commit()
        self.threadLock.release()       
        
        return 1    


    def recover(self): 
        self.resetProCallback(1)
        try: 
            resp = self.charge_client(-1,0.0)
        except:
            pass
#        rospy.sleep(0.5) 
#     	self.power_pub.publish(UInt8(5)) 
#        rospy.sleep(1) 
#     	self.power_pub.publish(UInt8(2))  

        self.mm_motor('stop',0.0)
        motor_result = self.mission_check(self.motorqueue,20)
        if motor_result < 0:
            return -1

        try:
            listener = tf.TransformListener()
            listener.waitForTransform(
                "/base", "/tool0_controller", rospy.Time(0), rospy.Duration(4.0))
            (rob_trans, rob_rot) = listener.lookupTransform(
                    '/base', '/tool0_controller', rospy.Time(0))
            if abs(rob_trans[0]) < 0.3  and abs(rob_trans[1]) < 0.3:
                pass
            else:
                self.mm_arm('MoveInTool',[0.0,0.0,-0.1,0.0,0.0,0.0,1.0])
                arm_result = self.mission_check(self.armqueue,20)
                if arm_result < 0:
                    return -1
        except Exception as e:
            print e
            return -1

        self.mm_motor('home',0.0)
        motor_result = self.mission_check(self.motorqueue,20)
        if motor_result < 0:
            return -1   

        test = self.mm_standby()
        if test < 0:
            return -1
        if self.toolid != 0:
            test = self.mm_changetool(self.toolid,0)
            if test < 0:
                return -1
            test = self.mm_standby()
            if test < 0:
                return -1
        test = self.mm_home()
        if test < 0:
            return -1
    
        return 1   


    def demo(self,process):
        process.current_mission = 'demo'              
        self.server.publish_feedback(process)
        test = self.mm_standby()
        if test < 0:
            return -1
        print('standby, continue...')
        self.equ_sqlid = 8

        results = self.mm_sql(0)
        self.mm_arm('jointSpacePlanning',results)
        arm_result = self.mission_check(self.armqueue,20)
        if arm_result < 0:
            return -1
        print('arm arrived, continue...')
        rospy.sleep(2)

        results = self.mm_sql(1)
        self.mm_arm('jointSpacePlanning',results)
        arm_result = self.mission_check(self.armqueue,20)
        if arm_result < 0:
            return -1
        print('arm arrived, continue...')

        process.current_mission = "pointMeter"              
        self.server.publish_feedback(process)
        test = self.move_and_capture("pointMeter")
        if test < 0:
            return -1

        test = self.mm_changetool(self.toolid,1)
        if test < 0:
            return -1

        process.current_mission = "remoteSwitch" 
        self.server.publish_feedback(process)
        test = self.move_and_capture("remoteSwitch")
        if test < 0:
            return -1

        self.mm_motor('move',30)
        motor_result = self.mission_check(self.motorqueue,20)
        if motor_result < 0:
            return -1

        self.mm_arm('MoveInTool',[0.0,0.0,0.15,0.0,0.0,0.0,1.0])
        arm_result = self.mission_check(self.armqueue,20)
        if arm_result < 0:
            return -1

        self.mm_motor('move',-30)
        motor_result = self.mission_check(self.motorqueue,20)
        if motor_result < 0:
            return -1

        self.mm_arm('MoveInTool',[0.0,0.0,-0.1,0.0,0.0,0.0,1.0])
        arm_result = self.mission_check(self.armqueue,20)
        if arm_result < 0:
            return -1

        test = self.move_and_capture("light")
        if test < 0:
            return -1

        process.current_mission = "handcartSwitch" 
        self.server.publish_feedback(process)
        test = self.mm_changetool(self.toolid,2)
        if test < 0:
            return -1

        test = self.move_and_capture("handcartSwitch")
        if test < 0:
            return -1

        self.mm_motor('move',-10)

        self.mm_arm('MoveInTool',[0.0,0.0,0.15,0.0,0.0,0.0,1.0])
        arm_result = self.mission_check(self.armqueue,20)
        if arm_result < 0:
            return -1

        self.mm_motor('jog',1)
        motor_result = self.mission_check(self.motorqueue,40)
        if motor_result < 0:
            return -1

        self.mm_arm('MoveInTool',[0.0,0.0,-0.15,0.0,0.0,0.0,1.0])
        arm_result = self.mission_check(self.armqueue,20)
        if arm_result < 0:
            return -1

        self.mm_motor('home',0)

        test = self.move_and_capture("light")
        if test < 0:
            return -1

        test = self.mm_changetool(self.toolid,1)
        if test < 0:
            return -1

        process.current_mission = "knifeSwitch" 
        self.server.publish_feedback(process)
        test = self.move_and_capture("knifeSwitch")
        if test < 0:
            return -1

        self.mm_motor('move',0)
        motor_result = self.mission_check(self.motorqueue,20)
        if motor_result < 0:
            return -1

        self.mm_arm('MoveInTool',[0.0,0.0,0.15,0.0,0.0,0.0,1.0])
        arm_result = self.mission_check(self.armqueue,20)
        if arm_result < 0:
            return -1

        self.mm_motor('move',-50)
        motor_result = self.mission_check(self.motorqueue,20)
        if motor_result < 0:
            return -1

        self.mm_motor('move',0)
        motor_result = self.mission_check(self.motorqueue,20)
        if motor_result < 0:
            return -1


        self.mm_arm('MoveInTool',[0.0,0.0,-0.1,0.0,0.0,0.0,1.0])
        arm_result = self.mission_check(self.armqueue,20)
        if arm_result < 0:
            return -1

        test = self.move_and_capture("light")
        if test < 0:
            return -1

        process.current_mission = "remoteSwitch" 
        self.server.publish_feedback(process)
        test = self.move_and_capture("remoteSwitch")
        if test < 0:
            return -1

        self.mm_motor('move',-30)
        motor_result = self.mission_check(self.motorqueue,20)
        if motor_result < 0:
            return -1

        self.mm_arm('MoveInTool',[0.0,0.0,0.15,0.0,0.0,0.0,1.0])
        arm_result = self.mission_check(self.armqueue,20)
        if arm_result < 0:
            return -1

        self.mm_motor('move',30)
        motor_result = self.mission_check(self.motorqueue,20)
        if motor_result < 0:
            return -1

        self.mm_arm('MoveInTool',[0.0,0.0,-0.1,0.0,0.0,0.0,1.0])
        arm_result = self.mission_check(self.armqueue,20)
        if arm_result < 0:
            return -1

        test = self.mm_changetool(self.toolid,0)
        if test < 0:
            return -1

        test = self.mm_standby()
        if test < 0:
            return -1

        test = self.mm_home()
        if test < 0:
            return -1

        rospy.sleep(60)
        return 1


    def _shutdown(self): 
        rospy.sleep(0.5) 
        self.power_pub.publish(UInt8(3))
        try: 
            resp = self.charge_client(-1,0.0)
        except:
            pass
        rospy.sleep(5)
#        os.system("pkill ros")    
        return 1  




    def mm_home(self):
        self.mm_arm('jointSpacePlanning',[-1.5579,0.1263,-2.7121,-0.4735,-1.5758,-0.0493,0])
        arm_result = self.mission_check(self.armqueue,30)
        if arm_result < 0:
            return -1
        else:
            return 1
       

    def mm_standby(self):
        self.mm_arm('jointSpacePlanning',[-2.7532,-0.7891,-2.2107,-0.1319,-0.6042,-0.0016,0])
        arm_result = self.mission_check(self.armqueue,30)
        if arm_result < 0:
            return -1
        else:
            return 1

    
    def mm_sql(self,unique_id):
        sql = """SELECT joint1,joint2,joint3,joint4,joint5,joint6 FROM Power_distribution_room.capture_position WHERE (equipment_type = %d and element_unique_id = %d)""" % (self.equ_sqlid,unique_id)
        self.cursor.execute(sql)
        results = self.cursor.fetchone()[0:6]
        results += (0.0,)
        return results     


    def move_and_capture(self,name):
        results = self.mm_sql(self.object_type_unique_id_map[name][0])
        self.mm_arm('jointSpacePlanning',results)
        arm_result = self.mission_check(self.armqueue,20)
        if arm_result < 0:
            return -1
        print('arm arrived, continue...')
        self.pubVisualApp.publish(VisualAppRequest(name,self.object_type_unique_id_map[name][0],self.equ_num,self.equ_sqlid))  
        return 1     
  

    def mm_arm(self,armorder,position):
        armGoal=pose4()
        armGoal.state=armorder
        armGoal.x=position[0]
        armGoal.y=position[1]
        armGoal.z=position[2]       
        armGoal.a=position[3]
        armGoal.b=position[4]
        armGoal.c=position[5]
        armGoal.w=position[6]


        print(armGoal)
        self.pubArm.publish(armGoal)

        


    def mm_motor(self,motororder,degree):
        motorGoal=motorCommand()
        motorGoal.workstate=motororder
        motorGoal.degree=degree  
        self.pubMotorCommand.publish(motorGoal)
        rospy.loginfo(motorGoal)              
            
   

	    

    def transCallback(self,flag):
        if flag.data == 1:
            print('trans is arrived')
            self.threadLock.acquire()
            self.transqueue.put(flag.data)
            self.threadLock.release()


    def armCallback(self,flag):
        if flag.data == 1:
            print('arm is arrived')
            self.threadLock.acquire()
            self.armqueue.put(flag.data)
            self.threadLock.release()


    def visualCallback(self,flag):
        self.threadLock.acquire()
        self.visualqueue.put([flag.object_name,flag.pose.state,flag.pose.x,flag.pose.y,flag.pose.z,flag.pose.a,flag.pose.b,flag.pose.c,flag.pose.w,flag.additional_text_info,flag.object_status,flag.success])
        self.threadLock.release()


    def endMotorCallback(self,flag):
        if flag.data == 1:
            print('motor is arrived')
            self.threadLock.acquire()
            self.motorqueue.put(flag.data)
            self.threadLock.release()

    def resetProCallback(self,flag):
        #if flag.data==1:      
            HOST = '192.168.0.11'  # ������������������ IP ��ַ
            PORT = 29999        # ������ʹ�õĶ˿�
             
            s = socket(AF_INET, SOCK_STREAM)
            s.connect((HOST, PORT))
            s.sendall('unlock protective stop\n')
            data = s.recv(1024)

            print('Received', repr(data)) 
            s.close()  

    def cancelCallback(self,flag):
        if flag.data==1:
            self.threadLock.acquire()
            self.armqueue.put(-1)
            self.transqueue.put(-1)
            self.motorqueue.put(-1)
            self.visualqueue.put([-1,-1])
            self.threadLock.release()                          



    def mission_check(self,name,time_out):
        self.zerolize()         
        try:
            check_result = name.get(timeout = time_out) 
        except Queue.Empty:
            return -1
        return check_result
        




if __name__ == '__main__':
    try:
        Execute=Robot_decision()
    except rospy.ROSInterruptException:
        pass
