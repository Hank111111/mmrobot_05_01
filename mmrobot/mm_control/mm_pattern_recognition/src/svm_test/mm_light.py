#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import division
import numpy as np
import cv2
import os
import roslib
import rospy
import rospkg
from sklearn.svm import SVC
from sklearn.externals import joblib
from mm_robot_decision.msg import VisualAppResponse
from mm_visual_postion.msg import AppInnerRequest
from train_light import preprocess

import rospkg
from os.path import join
rospack = rospkg.RosPack()
utils_path = rospack.get_path('mm_pattern_recognition')
utils_path = join(utils_path, "src/utils")
import sys 
sys.path.append(utils_path)
print(utils_path)
from stereo_image_receiver import StereoImageReceiver

class LightRecognition():

    def __init__(self):
        rospy.init_node('light_recognition')
        rospy.loginfo('started')
        rospy.Subscriber('/mm_visual/apps/light/goal', AppInnerRequest, self.callback)
        self.pub_photo_light=rospy.Publisher('/mm_visual/apps/light/result',VisualAppResponse,queue_size=2) 
        self.stereo_image_receiver = StereoImageReceiver()

        rospack = rospkg.RosPack()
        pkg_pth = rospack.get_path('mm_pattern_recognition')
        model_roi_1 = joblib.load(pkg_pth + "/src/svm_test/model/train_model_roi_1.m")
        model_roi_2 = joblib.load(pkg_pth + "/src/svm_test/model/train_model_roi_2.m")
        model_roi_3 = joblib.load(pkg_pth + "/src/svm_test/model/train_model_roi_3.m")
        model_roi_4 = joblib.load(pkg_pth + "/src/svm_test/model/train_model_roi_4.m")
        self.models = {"roi_1":model_roi_1,"roi_2":model_roi_2,"roi_3":model_roi_3,"roi_4":model_roi_4}

        rospy.sleep(1)
    

    def publishResult(self, response):
        self.pub_photo_light.publish(response)
        print(response)

#    def preprocess(self,img):
#        (h,w,d) = img.shape
#        HSV=cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
#        hist, bins = np.histogram(HSV[:, :, 0].ravel(), bins=10)  
#        return hist/(h*w)

    def callback(self,msg):
        print("received the msg for ", msg)
        if msg.object_name.find('light') >=0:
            left_image, right_image = self.stereo_image_receiver.getLatestImages()
            left_roi = msg.left_roi
            right_roi = msg.right_roi

            type_light = msg.additional_text_info

            response = VisualAppResponse()
            response.frame_id = "endeffector"
            response.object_name = msg.object_name
            response.width = msg.object_width
            response.height = msg.object_height
            response.object_unique_id_on_equipment = msg.object_unique_id_on_equipment
            # default as false
            response.success = False

            try:            
                left_result = self.predict(left_image,left_roi,type_light)
                right_result = self.predict(right_image,right_roi,type_light)
#                if left_result == right_result:
                response.object_status = left_result
                if type_light == 'single' and len(response.object_status) == 1:
                    response.success = True
                elif type_light == 'double' and len(response.object_status) == 2:
                    response.success = True 
                    

            except BaseException, e:
                    response.success = False
                    print(e)  

        self.publishResult(response)   


    def predict(self,src,roi,kind):

        # predict result, int 0-dark, 1-bright
        # light state result, int 0-远方，分闸， int 1-就地，合闸
        result_list = []

        pattern = 0
        roi_modify = [int(roi.y)-pattern, int(roi.y+roi.height)+pattern, int(roi.x)-pattern, int(roi.x+roi.width)+pattern]
        roi_img = src[roi_modify[0]:roi_modify[1],roi_modify[2]:roi_modify[3]]


        if kind == 'single':
            # actual model for single light to be checked
            h,w,d = roi_img.shape
            width_light = int(28 / 348 * w)
            width_set = int(80 / 348 * w)

            roi_1 = roi_img[0:h, 0:width_light]
            roi_2 = roi_img[0:h, width_set:(width_set + width_light)]

            result1 = int(self.models[roi_1].predict([preprocess(roi_1)]))
            result2 = int(self.models[roi_2].predict([preprocess(roi_2)]))
            if result1 + result2 == 1:
                result_list.append(result2)                       



        if kind == 'double':

            h,w,d = roi_img.shape

            width_light = int(28 / 348 * w)
            width_set = int(80 / 348 * w)

            roi_1 = roi_img[0:h, 0:width_light]
            roi_2 = roi_img[0:h, width_set:(width_set + width_light)]
            roi_3 = roi_img[0:h, 2 * width_set:(2 * width_set + width_light)]
            roi_4 = roi_img[0:h, 3 * width_set:(3 * width_set + width_light)]

            result1 = int(self.models["roi_1"].predict([preprocess(roi_1)]))
            result2 = int(self.models["roi_2"].predict([preprocess(roi_2)]))
            result3 = int(self.models["roi_3"].predict([preprocess(roi_3)]))
            result4 = int(self.models["roi_4"].predict([preprocess(roi_4)]))
            print result1,result2,result3,result4
            if result1 + result2 == 1 and result3 + result4 == 1:
                result_list.append(int(result2))
                result_list.append(int(result4))

        print result_list
        return result_list


if __name__ == '__main__':
    light_rec = LightRecognition()
    rospy.spin()
            
            


