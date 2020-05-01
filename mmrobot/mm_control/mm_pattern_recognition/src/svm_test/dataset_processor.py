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
            roi_random = np.array(roi) + (2*np.random.rand(4) -1.0) * 5
            roi_random = roi_random.astype(np.int)
            rotate_random = (2*np.random.rand() -1.0) * 5 #+-5 degrees

            M = cv2.getRotationMatrix2D((roi[0] + roi[2]//2, roi[1] + roi[3]//2), rotate_random, 1.0)
            rotated_image = cv2.warpAffine(image, M, (image.shape[1], image.shape[0]))
            augmented_images.append(rotated_image[roi_random[1]:roi_random[1]+roi_random[3], roi_random[0]:roi_random[0]+roi_random[2]])
            #cv2.imshow("augmented_images", augmented_images[len(augmented_images)-1])
            #cv2.waitKey(0)
        return augmented_images

    @classmethod
    def getSubROI(cls, df_row,roi_x_name, roi_y_name, roi_width_name, roi_height_name):
        roi_x = df_row[roi_x_name]
        roi_y = df_row[roi_y_name]
        roi_width = df_row[roi_width_name]
        roi_height = df_row[roi_height_name]

        width_light = int(28 / 348 * roi_width)
        width_set = int(80 / 348 * roi_width)

        roi_1 = np.array([roi_x, roi_y, width_light, roi_height]).astype(np.int)
        roi_2 = np.array([roi_x + width_set, roi_y, width_light, roi_height]).astype(np.int)
        roi_3 = np.array([roi_x + 2 * width_set, roi_y, width_light, roi_height]).astype(np.int)
        roi_4 = np.array([roi_x + 3 * width_set, roi_y, width_light, roi_height]).astype(np.int)
        return (roi_1, roi_2, roi_3, roi_4)

    def processSubDir(self, sub_dir, roi_1_save_path, roi_2_save_path, roi_3_save_path, roi_4_save_path, roi_1_save_index, roi_2_save_index, roi_3_save_index, roi_4_save_index, augmentation_ratio):
        df = pd.read_csv(join(sub_dir, "roi.csv"))
        for i, row in df.iterrows():
            left_image = cv2.imread(join(sub_dir, "left_"+str(row["id"])+".png"))
            right_image = cv2.imread(join(sub_dir, "right_"+str(row["id"])+".png"))
            left_roi_1, left_roi_2, left_roi_3, left_roi_4 = self.getSubROI(row, "left_roi_x","left_roi_y","left_roi_width","left_roi_height")
            right_roi_1, right_roi_2, right_roi_3, right_roi_4 = self.getSubROI(row, "right_roi_x","right_roi_y","right_roi_width","right_roi_height")
            
            roi_1_imgs = self.dataAugmentation(left_image, left_roi_1, augmentation_ratio)
            roi_2_imgs = self.dataAugmentation(left_image, left_roi_2, augmentation_ratio)
            roi_3_imgs = self.dataAugmentation(left_image, left_roi_3, augmentation_ratio)
            roi_4_imgs = self.dataAugmentation(left_image, left_roi_4, augmentation_ratio)

            for img in roi_1_imgs:
                cv2.imwrite(join(roi_1_save_path, str(roi_1_save_index)+"_.png"), img)
                roi_1_save_index += 1
            for img in roi_2_imgs:
                cv2.imwrite(join(roi_2_save_path, str(roi_2_save_index)+"_.png"), img)
                roi_2_save_index += 1
            for img in roi_3_imgs:
                cv2.imwrite(join(roi_3_save_path, str(roi_3_save_index)+"_.png"), img)
                roi_3_save_index += 1
            for img in roi_4_imgs:
                cv2.imwrite(join(roi_4_save_path, str(roi_4_save_index)+"_.png"), img)
                roi_4_save_index += 1

            roi_1_imgs = self.dataAugmentation(right_image, right_roi_1, augmentation_ratio)
            roi_2_imgs = self.dataAugmentation(right_image, right_roi_2, augmentation_ratio)
            roi_3_imgs = self.dataAugmentation(right_image, right_roi_3, augmentation_ratio)
            roi_4_imgs = self.dataAugmentation(right_image, right_roi_4, augmentation_ratio)

            for img in roi_1_imgs:
                cv2.imwrite(join(roi_1_save_path, str(roi_1_save_index)+"_.png"), img)
                roi_1_save_index += 1
            for img in roi_2_imgs:
                cv2.imwrite(join(roi_2_save_path, str(roi_2_save_index)+"_.png"), img)
                roi_2_save_index += 1
            for img in roi_3_imgs:
                cv2.imwrite(join(roi_3_save_path, str(roi_3_save_index)+"_.png"), img)
                roi_3_save_index += 1
            for img in roi_4_imgs:
                cv2.imwrite(join(roi_4_save_path, str(roi_4_save_index)+"_.png"), img)
                roi_4_save_index += 1

        return roi_1_save_index, roi_2_save_index, roi_3_save_index, roi_4_save_index
    
    def processDirs(self):
        index_roi_1 = {}
        index_roi_2 = {}
        index_roi_3 = {}
        index_roi_4 = {}
        for sub_dir in self.raw_data_sub_dirs:
            dir_name = sub_dir.split("/")[-1]
            if len(dir_name) == 0:
                dir_name = sub_dir.split("/")[-2]
            print("process directory: ", dir_name)
            # dir_name.split("_")[7]to distinguish
            roi_1_color_name = dir_name.split("_")[3]
            roi_2_color_name = dir_name.split("_")[4]
            roi_3_color_name = dir_name.split("_")[5]
            roi_4_color_name = dir_name.split("_")[6]
            
            roi_1_save_path = join(self.processed_data_dir, "roi_1/"+roi_1_color_name)
            roi_2_save_path = join(self.processed_data_dir, "roi_2/"+roi_2_color_name)
            roi_3_save_path = join(self.processed_data_dir, "roi_3/"+roi_3_color_name)
            roi_4_save_path = join(self.processed_data_dir, "roi_4/"+roi_4_color_name)

            if not os.path.exists(roi_1_save_path):
                os.makedirs(roi_1_save_path)
            if not os.path.exists(roi_2_save_path):
                os.makedirs(roi_2_save_path)
            if not os.path.exists(roi_3_save_path):
                os.makedirs(roi_3_save_path)
            if not os.path.exists(roi_4_save_path):
                os.makedirs(roi_4_save_path)
            
            if roi_1_color_name not in index_roi_1:
                index_roi_1[roi_1_color_name] = 0
            if roi_2_color_name not in index_roi_2:
                index_roi_2[roi_2_color_name] = 0
            if roi_3_color_name not in index_roi_3:
                index_roi_3[roi_3_color_name] = 0
            if roi_4_color_name not in index_roi_4:
                index_roi_4[roi_4_color_name] = 0

            index_roi_1[roi_1_color_name], index_roi_2[roi_2_color_name], index_roi_3[roi_3_color_name], index_roi_4[roi_4_color_name] = self.processSubDir(sub_dir, roi_1_save_path, roi_2_save_path, roi_3_save_path, roi_4_save_path, index_roi_1[roi_1_color_name], index_roi_2[roi_2_color_name], index_roi_3[roi_3_color_name], index_roi_4[roi_4_color_name], 3)
            



            


if __name__ == "__main__":

    rospy.init_node('dataset_processor')
    rospy.loginfo('started')
    rospack = rospkg.RosPack()
    data_dir =  join(rospack.get_path('mm_pattern_recognition'), "data/light")
    processor = DatasetProcessor(data_dir)
    processor.processDirs()
