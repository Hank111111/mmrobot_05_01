#!/usr/bin/env python

import rospy
import math
from sensor_msgs.msg import JointState 
import MySQLdb
import tf
from mm_robot_decision.msg import PositionRecord

db = MySQLdb.connect(host="localhost", port=3306, user="mrobot",passwd="123456",db="Power_distribution_room",charset="utf8" )

def callback(msg):
    for i in range (0,6):
        name=msg.name[i]  
        rad=msg.position[i]
        rospy.loginfo('joint: %s, degree: %.4f',name, rad)
    rospy.loginfo('************************************************')
    return msg.position

def orderCallback(msg):
    if 'cab' == msg.record_type:
        listener = tf.TransformListener()
        listener.waitForTransform("/map", "/base_footprint", rospy.Time(), rospy.Duration(100.0)) 
        equ_name = msg.equ_name
        (trans1, rot1) = listener.lookupTransform("/map", "/base_footprint", rospy.Time(0))
        if rot1[3] < 0:
            rot1[2]=-rot1[2]
            rot1[3]=-rot1[3]
        print(trans1,rot1)
        write2sql_cab(equ_name,trans1,rot1)  
 
    elif 'ele' == msg.record_type:
        equ_type = msg.equ_type
        ele_name = msg.ele_name
        ele_unique_id = msg.ele_unique_id
        subjoint = rospy.wait_for_message('/joint_states', JointState)
        jjoints = callback(subjoint)
        write2sql_ele(equ_type,ele_name,ele_unique_id,jjoints)  

    rospy.sleep(0.5)           


def write2sql_ele(equ_type,ele_name,ele_id,joints):
    cursor = db.cursor()
    sql = """INSERT INTO `Power_distribution_room`.`capture_position` (`equipment_type`,`element_name`,`element_unique_id`,`joint1`, `joint2`, `joint3`, `joint4`, `joint5`, `joint6`) VALUES ('%d', '%s', '%d', '%.4f', '%.4f', '%.4f', '%.4f', '%.4f', '%.4f');""" % (equ_type,ele_name,ele_id,joints[0],joints[1],joints[2],joints[3],joints[4],joints[5])
    cursor.execute(sql)
    db.commit()  


def write2sql_cab(name,trans,rot):
    cursor = db.cursor()
    sql = """UPDATE `Power_distribution_room`.`equipment` SET poi_x=%f, poi_y=%f, poi_c=%f, poi_w=%f  WHERE equipment.name = '%s' ;""" % (trans[0],trans[1],rot[2],rot[3],name)
    cursor.execute(sql)
    db.commit()  


if __name__ == '__main__':
    rospy.init_node('recordlistener', anonymous=True)
    subOrder=rospy.Subscriber('/mm_record/order',PositionRecord, orderCallback)    
    rospy.sleep(1)
    rospy.spin()


