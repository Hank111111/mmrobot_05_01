#! /usr/bin/env python
# -*- coding: utf-8 -*-
import MySQLdb
db = MySQLdb.connect(host="localhost", port=3306, user="mrobot",passwd="123456",db="Power_distribution_room",charset="utf8" )
cursor = db.cursor()
sql = """SELECT toolid FROM Power_distribution_room.robot_state WHERE id = 1"""
cursor.execute(sql)
# 获取记录列表
results = cursor.fetchone()
toolid=int(results[0])
print(toolid)        
