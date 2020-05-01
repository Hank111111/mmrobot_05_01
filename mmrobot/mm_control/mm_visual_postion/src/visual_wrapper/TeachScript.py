#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import division, print_function
import roslib
import rospy
from mm_robot_decision.msg import VisualAppResponse, VisualAppRequest
from mm_visual_postion.msg import AppInnerRequest

import mysql.connector

# This is a teach script that guides you to do the things correctly.
unique_id_object_type_map = {0: "QRCode",
                        1: "QRCode",
                        2: "knifeSwitch",
                        3: "remoteSwitch",
                        4: "handcartSwitch",
                        5: "pointMeter",
                        6: "light"}

class DataBaseInterface:
    def __init__(self, user, password, host, database_name):
        self.user = user
        self.password = password
        self.host = host
        self.database_name = database_name
        self.src_id = None
        self.src_type_id = None
        self.data_of_src_equipment = []

    def readEquipment(self, type_id, equipment_id):
        '''
        cnx = mysql.connector.connect(user='mrobot', password='123456',
                                host='127.0.0.1',
                                database='Power_distribution_room')
        '''
        cnx = mysql.connector.connect(user=self.user, password=self.password,
                                host=self.host,
                                database=self.database_name)
        cursor = cnx.cursor(dictionary=True)
        table_name = "equipment_type_" + str(type_id) + "_details"
        
        query = "SELECT * FROM %s WHERE %s.equipment_id = %d;"

        print(query % (table_name, table_name, equipment_id))


        cursor.execute(query % (table_name, table_name, equipment_id))
        print("read equipment info of equipment id = {}, type id = {}.".format(type_id, equipment_id))
        self.data_of_src_equipment = []
        for row in cursor:
            self.data_of_src_equipment.append(row)
           
        cursor.close()
        cnx.close()
        self.src_id = equipment_id
        self.src_type_id = type_id
        print("************************")
    
    def copyToEquipment(self, type_id, equipment_id, qr_code_type_id, qr_code_content_col_name, qr_code_0_content, qr_code_1_content):
        assert self.src_type_id is not None and self.src_id is not None, "you should firstly read the equipment of same type from database"
        assert type_id == self.src_type_id
        cnx = mysql.connector.connect(user=self.user, password=self.password,
                                host=self.host,
                                database=self.database_name)
        cursor = cnx.cursor()
        
        # prepare the prefix "INSERT INTO XXXX"
        add_line_cmd_prefix = "INSERT INTO equipment_type_%s_details " % type_id 
        col_names_list = self.data_of_src_equipment[0].keys()
        print( self.data_of_src_equipment[0].keys())
        add_line_cmd_prefix += "("
        for col_i, col_name in enumerate(col_names_list):
            if col_name == 'id':
                 continue
            print(col_name)
            add_line_cmd_prefix += col_name + ", "
        add_line_cmd_prefix = add_line_cmd_prefix[:-2] # remove last ", "
        add_line_cmd_prefix += ") "
        
        # prepare the values
        for row_of_src_data in self.data_of_src_equipment:
            add_line_cmd = add_line_cmd_prefix + "VALUES ("
            for col_i, col_name in enumerate(col_names_list):
                if col_name == 'id': 
                    continue
                # change qr code content, the qrcode should be always the first element of the equipment
                if type(row_of_src_data[col_name]) is str:
                    value = "'"+row_of_src_data[col_name]+"'"
                else:
                    value = str(row_of_src_data[col_name])
                
                if col_name == 'type_id' :
                    value = str(type_id)
                elif col_name == "equipment_id":
                    value = str(equipment_id)
                elif(col_name == qr_code_content_col_name and row_of_src_data["object_unique_id_on_equipment"] == 0):
                    assert row_of_src_data["object_type_id"] == qr_code_type_id, "qr code should be always the first element of the equipment"
                    value = "'" + qr_code_0_content + "'"
                elif (col_name == qr_code_content_col_name and row_of_src_data["object_unique_id_on_equipment"] == 1):
                    assert row_of_src_data["object_type_id"] == qr_code_type_id, "qr code should be always the first element of the equipment"
                    value = "'" + qr_code_1_content + "'"
                elif value == "None" or value == "'None'":
                    value = "NULL"
                add_line_cmd += value + ", "
            add_line_cmd = add_line_cmd[:-2] # remove last ", "
            add_line_cmd += ")"
            print(add_line_cmd)
            cursor.execute(add_line_cmd)

        cnx.commit()
        cursor.close()
        cnx.close()
        print("************************")

                    
class TeachProcess():
    def __init__(self):

        self.unique_id_object_type_map = unique_id_object_type_map
        rospy.init_node('teach_script')
        rospy.loginfo('started')
        rospy.Subscriber('/mm_visual/wrapper/result', VisualAppResponse, self.positionningCallback)
        self.positionningRequestPub = rospy.Publisher('/mm_visual/wrapper/request',VisualAppRequest,queue_size=1) 
        self.refineRequestPub = rospy.Publisher('/mm_visual/wrapper/refine_cabinet', VisualAppRequest, queue_size=1)
        self.grabTemplatePub = rospy.Publisher('mm_visual/wrapper/grab_template', AppInnerRequest, queue_size=1)
        self.grabDataPub = rospy.Publisher('mm_visual/wrapper/grab_data', VisualAppRequest, queue_size=1)
        self.grabEquipmentPosePub = rospy.Publisher("mm_visual/wrapper/request_cabinet_pose", VisualAppRequest, queue_size=1)
        self.goal_success = False
        self.current_equipment_id = None
        self.current_equipment_type_id = None
    def positionningCallback(self, msg):
        self.goal_success = msg.success

    def waitForKey(self, msg, key='y'):
        while True:
            input_content = raw_input(msg)
            if(key == input_content):
                return
    def createDataBase(self):
        print("Please run the 'create_cabinet_equipment_database' as root to create the database, if you already create the database, ignore this step")
        self.waitForKey("If you finished, press 'y' to continue: ")
        print("************************")

        return
    def mesureEquipment(self):
        print("Please mesure the equipment and fill in the database manually")
        print("Attention! the QRCode0 can only have object_unique_id_on_equipment=0, the QRCode1 can only have object_unique_id_on_equipment=1")
        print("Each QRCode's content (string) should be written into colomn 'additional_text_info'")
        print("'object_unique_id_on_equipment' should be unique for each object on same equipment")
        self.waitForKey("If you finished, press 'y' to continue: ")
        print("************************")

    def moveRobotBaseToEquipment(self, equipment_id, equipment_type_id):
        print("Please move robot base to equipment (equipment_id={:d}, equipment_type_id={:d}".format(equipment_id, equipment_type_id))
        self.current_equipment_id = equipment_id
        self.current_equipment_type_id = equipment_type_id
        self.waitForKey("If you finished, press 'y' to continue: ")
        print("************************")

    def moveRobotArmToCaputreQRCode(self, qr_code_id):
        print("Please move robot arm to capture the QR code {:d} of this equipment".format(qr_code_id))
        self.waitForKey("If you finished, press 'y' to continue: ")
        print("************************")
        
    def sendEquipmentPoseToServer(self):
        print("Send equipment pose to server, please make sure you have run SocketSendEquipmentPose.py on robot, and SocketReceiveEquipmentPose.py on server")
        request = VisualAppRequest()
        request.equipment_id = self.current_equipment_id
        request.equipment_type = self.current_equipment_type_id
        self.grabEquipmentPosePub.publish(request)
        self.waitForKey("If you finished, press 'y' to continue: ")
        print("************************")
    def captureQRCode(self, qr_code_id):
        request = VisualAppRequest()
        request.equipment_id = self.current_equipment_id
        request.equipment_type = self.current_equipment_type_id
        request.object_name = self.unique_id_object_type_map[qr_code_id]
        request.object_unique_id_on_equipment = qr_code_id
        self.positionningRequestPub.publish(request)
        print("published msg \n", request)
        print("wait for visual_safe_wrapper to positionning the qrcode {:d}".format(qr_code_id))
        self.waitForKey("If visual_safe_wrapper finished, press 'y' to continue")
        print("************************")

    def refineObject(self, unique_id):
        request = VisualAppRequest()
        request.equipment_id = self.current_equipment_id
        request.equipment_type = self.current_equipment_type_id
        request.object_name = self.unique_id_object_type_map[unique_id]
        request.object_unique_id_on_equipment = unique_id
        print("published msg \n", request)
        self.refineRequestPub.publish(request)
        print("wait for visual_safe_wrapper to refine the object with unique_id={:d}".format(unique_id))
        self.waitForKey("If visual_safe_wrapper finished, press 'y' to continue")
        print("************************")
    
    def grabRemoteSwitchTemplate(self, remote_switch_unique_id):
        request = AppInnerRequest()
        request.object_unique_id_on_equipment = remote_switch_unique_id
        request.object_name = self.unique_id_object_type_map[remote_switch_unique_id]

        current_status = -1
        while current_status == -1:
            raw_input_content = raw_input("Please enter the current status of the remote switch, only '0' or '1' is acceptable:  ")
            if raw_input_content == '0':
                current_status = 0
            elif raw_input_content == '1':
                current_status = 1

        if current_status == 0:
            request.additional_text_info = "status_0"
        else:
            request.additional_text_info = "status_1"
        print("published msg \n", request)
        self.grabTemplatePub.publish(request)
        self.waitForKey("If visual_safe_wrapper finished, press 'y' to continue")
        print("************************")

    def grabTrainableData(self, object_unique_id):
        request = VisualAppRequest()
        request.equipment_id = self.current_equipment_id
        request.equipment_type = self.current_equipment_type_id
        request.object_name = self.unique_id_object_type_map[object_unique_id]
        request.object_unique_id_on_equipment = unique_id

        print("published msg \n", request)
        self.grabDataPub.publish(request)
        print("wait for visual_safe_wrapper to grab trainable data of the object with unique_id={:d}".format(object_unique_id))
        self.waitForKey("If visual_safe_wrapper finished, press 'y' to continue")
        print("************************")
    
    def copyToEquipment(self, src_equipment_id, src_type_id, dst_equipment_id, qr_code_type_id):
        db_interface = DataBaseInterface(user='mrobot', password='123456',
                                host='127.0.0.1',
                                database_name='Power_distribution_room')
        db_interface.readEquipment(src_type_id, src_equipment_id)
        qr_code_0_content = raw_input("Please enter the qrcode 0's content for this equipment:  ")
        qr_code_1_content = raw_input("Please enter the qrcode 1's content for this equipment:  ")

        db_interface.copyToEquipment(type_id=src_type_id, equipment_id=dst_equipment_id,
                                        qr_code_0_content=qr_code_0_content, qr_code_1_content=qr_code_1_content, 
                                        qr_code_content_col_name='additional_text_info', qr_code_type_id = qr_code_type_id)
        print("succesfully copy the data of equipment {:d} to equipment {:d} ".format(src_equipment_id, dst_equipment_id))
        print("************************")

if __name__ == "__main__":

    qr_code_type_id = 5
    print("please make sure this unique_id_object_type_map is coherent with the database")
    print(unique_id_object_type_map)
    print("please make sure the qrcode_type_id={:d} in database".format(qr_code_type_id))
    # create object_type_unique_id_map
    object_type_unique_id_map ={}
    for unique_id in unique_id_object_type_map:
        if unique_id not in object_type_unique_id_map.keys():
            object_type_unique_id_map[unique_id_object_type_map[unique_id]] = []
        object_type_unique_id_map[unique_id_object_type_map[unique_id]].append(unique_id)
    
    print(object_type_unique_id_map)

    # start to teach
    teach_process = TeachProcess()

    while not rospy.is_shutdown():
        first_equipment_type_id = input("Please enter the type id of this kind of equipment:  " )
        first_equipment_id = input("Please enter the id of first equipment to teach:  ")

        teach_process.createDataBase()
        teach_process.mesureEquipment()

        teach_process.moveRobotBaseToEquipment(first_equipment_id, first_equipment_type_id)
        teach_process.moveRobotArmToCaputreQRCode(0)
        teach_process.captureQRCode(0)
        teach_process.moveRobotArmToCaputreQRCode(1)
        teach_process.captureQRCode(1)
        for unique_id in unique_id_object_type_map.keys():
            teach_process.refineObject(unique_id)
        for remote_switch_unique_id in object_type_unique_id_map['remoteSwitch']:
            teach_process.grabRemoteSwitchTemplate(remote_switch_unique_id)
        for light_unique_id in object_type_unique_id_map['light']:
            teach_process.grabTrainableData(light_unique_id)
        #teach_process.sendEquipmentPoseToServer()
        while not rospy.is_shutdown():
            next_equipment_id = raw_input("Please enter the next equipment id of the same type, enter 'break' to start the teaching on other type of equipment:  ")
            if(next_equipment_id == 'break'):
                break
            next_equipment_id = int(next_equipment_id)
            teach_process.copyToEquipment(first_equipment_id, first_equipment_type_id, next_equipment_id, qr_code_type_id)
            teach_process.moveRobotBaseToEquipment(next_equipment_id, first_equipment_type_id)
            teach_process.moveRobotArmToCaputreQRCode(0)
            teach_process.captureQRCode(0)
            teach_process.moveRobotArmToCaputreQRCode(1)
            teach_process.captureQRCode(1)
            for unique_id in unique_id_object_type_map.keys():
                teach_process.refineObject(unique_id)
            for light_unique_id in object_type_unique_id_map['light']:
                teach_process.grabTrainableData(light_unique_id)
            #teach_process.sendEquipmentPoseToServer()

    rospy.spin()
