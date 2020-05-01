#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import division, print_function
import roslib
import rospy
import rospkg
from os.path import join
import os
import pandas as pd
import cv2
import numpy as np
class DatasetProcessor:
    def __init__(self, path):
        self.raw_data_dir = join(path, "raw")
        self.processed_data_dir = join(path, "processed")
        rospy.loginfo("write processed data to %s" % self.processed_data_dir)
        self.raw_data_sub_dirs = [join(self.raw_data_dir, sub_p) for sub_p in os.listdir(self.raw_data_dir)]
        
    @classmethod
    def dataAugmentation(cls, image, roi, num):
        augmented_images = []
        for i in range(num):
            if(rospy.is_shutdown()):
                return
            
            roi_random = np.array(roi)
            # add pattern
            pattern = 0
            roi_random[0] -= pattern
            roi_random[1] -= pattern
            roi_random[2] += pattern * 2
            roi_random[3] += pattern * 2
            
            roi_random[0] += (2*np.random.rand() -1 ) * 10
            roi_random[1] += (2*np.random.rand() -1 ) * 10
            roi_random[2] += (2*np.random.rand() -1 ) * 10
            roi_random[3] += (2*np.random.rand() - 1 ) * 10
            
            roi_random = roi_random.astype(np.int)
            rotate_random = (2*np.random.rand()-1) * 5 #+-5 degrees

            M = cv2.getRotationMatrix2D((roi[0] + roi[2]//2, roi[1] + roi[3]//2), rotate_random, 1.0)
            rotated_image = cv2.warpAffine(image, M, (image.shape[1], image.shape[0]))
            augmented_images.append(rotated_image[roi_random[1]:roi_random[1]+roi_random[3], roi_random[0]:roi_random[0]+roi_random[2]])
            #cv2.imshow("augmented_images", augmented_images[len(augmented_images)-1])
            #cv2.waitKey(0)
        return augmented_images

    @classmethod
    def getROI(cls, df_row,roi_x_name, roi_y_name, roi_width_name, roi_height_name):
        roi_x = df_row[roi_x_name]
        roi_y = df_row[roi_y_name]
        roi_width = df_row[roi_width_name]
        roi_height = df_row[roi_height_name]
        roi = np.array([roi_x, roi_y, roi_width, roi_height]).astype(np.int)
        
        return roi
    def processSubDir(self, sub_dir, save_path, save_index, augmentation_ratio):
        df = pd.read_csv(join(sub_dir, "roi.csv"))
        for i, row in df.iterrows():
            left_image = cv2.imread(join(sub_dir, "left_"+str(row["id"])+".png"))
            right_image = cv2.imread(join(sub_dir, "right_"+str(row["id"])+".png"))

            left_roi = self.getROI(row, "left_roi_x","left_roi_y","left_roi_width","left_roi_height")
            right_roi = self.getROI(row, "right_roi_x","right_roi_y","right_roi_width","right_roi_height")
            
            left_imgs = self.dataAugmentation(left_image, left_roi, augmentation_ratio)
            for img in left_imgs:
                cv2.imwrite(join(save_path, str(save_index)+"_.png"), img)
                save_index += 1
                

            right_imgs =  self.dataAugmentation(right_image, right_roi, augmentation_ratio)
            
            for img in right_imgs:
                cv2.imwrite(join(save_path, str(save_index)+"_.png"), img)
                save_index += 1
        return save_index
    
    def processDirs(self):
        index = {}
        for sub_dir in self.raw_data_sub_dirs:
            dir_name = sub_dir.split("/")[-1]
            if len(dir_name) == 0:
                dir_name = sub_dir.split("/")[-2]
            print("process directory: ", dir_name)
            # dir_name.split("_")[-2]to distinguish
            status_name = dir_name.split("_")[-1]
            save_path = join(self.processed_data_dir, status_name)
            if not os.path.exists(save_path):
                os.makedirs(save_path)
                
            
            if status_name not in index:
                index[status_name] = 0
                
            index[status_name] = self.processSubDir(sub_dir, save_path, index[status_name], 3)
            



            


if __name__ == "__main__":

    rospy.init_node('dataset_processor')
    rospy.loginfo('started')
    rospack = rospkg.RosPack()
    data_dir =  join(rospack.get_path('mm_pattern_recognition'), "data/remoteSwitch")
    processor = DatasetProcessor(data_dir)
    processor.processDirs()
