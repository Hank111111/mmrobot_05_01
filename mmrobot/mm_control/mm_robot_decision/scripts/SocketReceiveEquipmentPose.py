#!/usr/bin/env python
# -*- coding: UTF-8 -*-
import rospy
import sys
from socket import *
import json
import mysql.connector
class EquipmentPoseReceiver():
    def __init__(self):
        self.ss = socket(AF_INET, SOCK_STREAM)
        IP_PROT = ('127.0.0.1', 12397)
        
        #IP_PROT = ('10.161.8.142', 9995)
        self.ss.connect(IP_PROT)

        self.cnx = mysql.connector.connect(user='root', password='poweroot',
                        host='127.0.0.1',
                        database='wholedata')
    
        self.ss.send("Ready to receive")
        data = self.ss.recv(1024)
        while data != 'ready':
            print("received msg: ", data)
            print("wait for msg 'ready' from robot")
            data = self.ss.recv(1024)
            sleep(1.0)
        print("the connection to robot is established")
        while True:
            data = self.ss.recv(1024)
            print(data)
            data_loaded = json.loads(data) #data loaded
            self.writeToDB(data_loaded["equipment_name"], data_loaded["x"], data_loaded["y"], data_loaded["theta"])
        

    def writeToDB(self,equipment_name,x,y,theta):
        cursor = self.cnx.cursor()
        query = "UPDATE SET x=%f, y=%f * FROM equipment WHERE equipment.equipment_name = %s;"
        print(query % (x, y, equipment_name))
        cursor.execute(query % (x, y, equipment_name))
        self.cnx.commit()


if __name__ == "__main__":
    receiver = EquipmentPoseReceiver()