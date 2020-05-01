#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import division
import roslib
import rospy
import rospkg
import numpy as np
import cv2
import matplotlib.pyplot as plt
import os
from sklearn.svm import SVC
from sklearn.preprocessing import MinMaxScaler
from sklearn.externals import joblib
from os.path import join
#import torch
class FeatureExtractor:
    @classmethod
    def fd_hu_moments(cls, image):
        image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        feature = cv2.HuMoments(cv2.moments(image)).flatten()
        #print("hu", feature.shape)
        return feature
    '''
    @classmethod
    def fd_haralick(cls, image):    # convert the image to grayscale
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        # compute the haralick texture feature vector
        haralick = mahotas.features.haralick(gray).mean(axis=0)
        return haralick
    '''
    @classmethod
    def fd_histogram(cls, image, mask=None, bins=10):
        # convert the image to HSV color-space
        image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        # compute the color histogram
        hist_0  = cv2.calcHist([image], [0], None, [bins], [0, 256])
        hist_1  = cv2.calcHist([image], [1], None, [bins], [0, 256])
        hist_2  = cv2.calcHist([image], [2], None, [bins], [0, 256])
        # normalize the histogram
        cv2.normalize(hist_0, hist_0)
        cv2.normalize(hist_1, hist_1)
        cv2.normalize(hist_2, hist_2)
        hist = np.concatenate([hist_0, hist_1, hist_2], axis=0).flatten()
        #print("hist", hist.shape)
        return hist

    @classmethod
    def getFeatures(cls, image_origin):
        image = cv2.resize(image_origin, (200,200))
        global_features = np.hstack([cls.fd_histogram(image), cls.fd_hu_moments(image)]).reshape(-1,1)
        scaler = MinMaxScaler(feature_range=(0, 1))
        #Normalize The feature vectors...
        rescaled_features = scaler.fit_transform(global_features)
        return rescaled_features.reshape(-1)

'''
def preprocess(img):
    return FeatureExtractor.getFeatures(img)
'''
def preprocess(image_origin):
    image = cv2.resize(image_origin, (64,64))
    image = image / 255.0
    return image.reshape(-1)
def loadSubDataset(sub_dir_name):
    print(sub_dir_name)
    if not os.path.exists(sub_dir_name):
        return np.zeros((0,10))
    img_list = os.listdir(sub_dir_name)
    

    for i, img_name in enumerate(img_list):    
        img = cv2.imread(join(sub_dir_name, img_name))
        if i == 0:
            x_data_0 = preprocess(img)
            x_data = np.zeros((len(img_list), x_data_0.shape[0]))
            x_data[0, :] = x_data_0
        else:
            x_data[i, :] = preprocess(img)
    return x_data

def loadDataset(dir_name):
    labeled_data = {}
    label_names = ["remote","local"]
    for label in label_names:
        labeled_data[label] = loadSubDataset(join(dir_name, label))
        print("load %s data : %d" %(label, labeled_data[label].shape[0]) )
    
    x_data = np.concatenate([labeled_data[label] for label in label_names], axis=0)
    
    y_data = np.zeros(x_data.shape[0], dtype=np.int)
    last_num = 0
    for i,label in enumerate(label_names):
        num = labeled_data[label].shape[0]
        y_data[last_num:last_num+num] = i
        last_num = last_num + num
    
    return x_data, y_data    

def shuffleDataAndLabelsInPlace(arr1, arr2):
    seed = np.random.randint(0, 1000000)
    prng = np.random.RandomState(seed)
    prng.shuffle(arr1)
    prng = np.random.RandomState(seed)
    prng.shuffle(arr2)

def train(dir_name, save_dir, save_name, val_ratio):
    x_data, y_data = loadDataset(dir_name)
    shuffleDataAndLabelsInPlace(x_data, y_data)

    train_num = int((1-val_ratio) * x_data.shape[0])
    val_num = int(val_ratio * x_data.shape[0])
    
    train_x_data = x_data[:train_num]
    train_y_data = y_data[:train_num]
    val_x_data = x_data[train_num:]
    val_y_data = y_data[train_num:]

    clf = SVC(C=1.0, cache_size=200, class_weight=None, coef0=0.0,
            decision_function_shape='ovr', degree=3, gamma='scale', kernel='rbf',
            max_iter=-1, probability=False, random_state=None, shrinking=True,
            tol=0.001, verbose=False) 
    clf.fit(train_x_data,train_y_data)
    if not os.path.exists(save_dir):
        os.makedirs(save_dir)
    joblib.dump(clf, join(save_dir, save_name))

    train_score = clf.score(train_x_data,train_y_data) 
    val_score = clf.score(val_x_data, val_y_data)
    print("for save_name: ", save_name)
    print("train_score: ", train_score)
    print("val score: ", val_score)

def trainNN(dir_name, save_dir, save_name, val_ratio):
    from sklearn.neural_network import MLPClassifier
    x_data, y_data = loadDataset(dir_name)
    shuffleDataAndLabelsInPlace(x_data, y_data)

    train_num = int((1-val_ratio) * x_data.shape[0])
    val_num = int(val_ratio * x_data.shape[0])
    
    train_x_data = x_data[:train_num]
    train_y_data = y_data[:train_num]
    val_x_data = x_data[train_num:]
    val_y_data = y_data[train_num:]

    clf = MLPClassifier(solver='lbfgs', alpha=1e-5, 
                        hidden_layer_sizes=(10, 10, 10), random_state=1)
    clf.fit(train_x_data,train_y_data)
    if not os.path.exists(save_dir):
        os.makedirs(save_dir)
    joblib.dump(clf, join(save_dir, save_name))

    train_score = clf.score(train_x_data,train_y_data) 
    val_score = clf.score(val_x_data, val_y_data)
    print("for save_name: ", save_name)
    print("train_score: ", train_score)
    print("val score: ", val_score)

    clf.predict(x_data[1,:].reshape(1,-1))


if __name__ == "__main__":
    rospy.init_node('svm_trainer')
    rospy.loginfo('started')
    rospack = rospkg.RosPack()
    data_dir = join(rospack.get_path('mm_pattern_recognition'), "data/remoteSwitch/processed")
    save_dir = join(rospack.get_path('mm_pattern_recognition'), "src/switch/model")
    trainNN(dir_name=data_dir,save_dir=save_dir, save_name="train_model.m", val_ratio=0.3)   


