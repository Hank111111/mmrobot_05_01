#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import division

import rospy
import os
import shutil
import sys
import time

import cv2
import numpy as np
import tensorflow as tf
from cv_bridge import CvBridge, CvBridgeError
from mm_pattern_recognition.srv import ocr_rec
from math import *

sys.path.append(os.getcwd())
from nets import model_train as model
from utils.rpn_msr.proposal_layer import proposal_layer
from utils.text_connector.detectors import TextDetector
#from usr_ocr import seperation as sp
#from usr_ocr import g_graph as gl
from crnn import test as crnn
#print(ocr.__file__)

from std_msgs.msg import String,Int8



#over_pub=rospy.Publisher('/mm_char/check',Int8 , queue_size=10)


tf.app.flags.DEFINE_string('test_data_path', '/home/youibot/mrobot/src/mmrobot/mm_control/mm_pattern_recognition/src/text-detection-ctpn/data/demo/', '')
tf.app.flags.DEFINE_string('output_path', '/home/youibot/mrobot/src/mmrobot/mm_control/mm_pattern_recognition/src/text-detection-ctpn/data/res/', '')
tf.app.flags.DEFINE_string('singular_path', '/home/youibot/mrobot/src/mmrobot/mm_control/mm_pattern_recognition/src/text-detection-ctpn/crnn/img/', '')
tf.app.flags.DEFINE_string('gpu', '0', '')
tf.app.flags.DEFINE_string('checkpoint_path', '/home/youibot/mrobot/src/mmrobot/mm_control/mm_pattern_recognition/src/text-detection-ctpn/checkpoints_mlt/', '')
FLAGS = tf.app.flags.FLAGS


class Recognizer():
    def __init__(self):

        #with tf.get_default_graph().as_default():
        self.input_image = tf.placeholder(tf.float32, shape=[None, None, None, 3], name='input_image')
        self.input_im_info = tf.placeholder(tf.float32, shape=[None, 3], name='input_im_info')

        global_step = tf.get_variable('global_step', [], initializer=tf.constant_initializer(0), trainable=False)

        self.bbox_pred, self.cls_pred, self.cls_prob = model.model(self.input_image)

        variable_averages = tf.train.ExponentialMovingAverage(0.997, global_step)
        saver = tf.train.Saver(variable_averages.variables_to_restore())


        self.sess = tf.Session()
        #with tf.Session(config=tf.ConfigProto(allow_soft_placement=True)) as sess:
        ckpt_state = tf.train.get_checkpoint_state(FLAGS.checkpoint_path)
        model_path = os.path.join(FLAGS.checkpoint_path, os.path.basename(ckpt_state.model_checkpoint_path))
        print('Restore from {}'.format(model_path))
        saver.restore(self.sess, model_path)
        try:
            im = cv2.imread("/home/youibot/mrobot/src/mmrobot/mm_control/mm_pattern_recognition/src/text-detection-ctpn/Error.png")[:, :, ::-1]
        except:
            print("Error reading image {}!".format(im_fn))
        img, (rh, rw) = resize_image(im)
        h, w, c = img.shape
        im_info = np.array([h, w, c]).reshape([1, 3])
        bbox_pred_val, cls_prob_val = self.sess.run([self.bbox_pred, self.cls_prob],
                                               feed_dict={self.input_image: [img],
                                                          self.input_im_info: im_info})

        rospy.init_node('char_recognition')
        s = rospy.Service("/mm_char/img_roi", ocr_rec, self.handle_img_func)
        rospy.loginfo('started')
        #rospy.Subscriber('/mm_char/goal', Int8, self.callback)
        rospy.spin()    

    def handle_img_func(self,data):

        try:
            im = self.bridge.imgmsg_to_cv2(data.image_message, "bgr8")
        #cv_image = bridge.imgmsg_to_cv2(image_message, desired_encoding="passthrough")
        except CvBridgeError as e:
            print(e)

        start = time.time()

        if os.path.exists(FLAGS.output_path):
            shutil.rmtree(FLAGS.output_path)
        os.makedirs(FLAGS.output_path)
        if os.path.exists(FLAGS.singular_path):
            shutil.rmtree(FLAGS.singular_path)
        os.makedirs(FLAGS.singular_path)
        os.environ['CUDA_VISIBLE_DEVICES'] = FLAGS.gpu

        print('===============')

        img, (rh, rw) = resize_image(im)
        h, w, c = img.shape
        im_info = np.array([h, w, c]).reshape([1, 3])

        cost_time = (time.time() - start)
        print("cost time: {:.2f}s".format(cost_time))

        bbox_pred_val, cls_prob_val = self.sess.run([self.bbox_pred, self.cls_prob],
                                               feed_dict={self.input_image: [img],
                                                          self.input_im_info: im_info})
       
        cost_time = (time.time() - start)
        print("cost time: {:.2f}s".format(cost_time))

        textsegs, _ = proposal_layer(cls_prob_val, bbox_pred_val, im_info)
        scores = textsegs[:, 0]
        textsegs = textsegs[:, 1:5]

        textdetector = TextDetector(DETECT_MODE='O')
        boxes = textdetector.detect(textsegs, scores[:, np.newaxis], img.shape[:2])
        boxes = np.array(boxes, dtype=np.int)

        cost_time = (time.time() - start)
        print("cost time: {:.2f}s".format(cost_time)) 
        print len(boxes)

        for i, box in enumerate(boxes):
                #cv2.polylines(img, [box[:8].astype(np.int32).reshape((-1, 1, 2))], True, color=(0, 255, 0), thickness=2)
                #cropImg = im[int(box[1]/rw-10):int(box[5]/rw+10),int(box[0]/rh-10):int(box[4]/rh+10)]
                pt1=[box[0]/rh,box[1]/rw]
                pt2=[box[2]/rh,box[3]/rw]
                pt3=[box[4]/rh,box[5]/rw]
                pt4=[box[6]/rh,box[7]/rw]
                cropImg=dumpRotateImage(im,pt1,pt2,pt3,pt4)
                height,width=cropImg.shape[:2]

                if height == 0 or width == 0:
                    default_img=cv2.imread("/home/youibot/mrobot/src/mmrobot/mm_control/mm_pattern_recognition/src/text-detection-ctpn/Error.png")
                    cv2.imwrite(os.path.join(FLAGS.singular_path, os.path.basename(str(i)+'.png')), default_img)
                    continue

                resize_img=cv2.resize(cropImg, None, None, fx=64/height, fy=64/height, interpolation=cv2.INTER_LINEAR)

                #cropImg = cv2.equalizeHist(cropImg)
                #cv2.imshow("test",cropImg)
                #cv2.waitKey(0)
                #print(FLAGS.singular_path)
                cv2.imwrite(os.path.join(FLAGS.singular_path, os.path.basename(str(i)+'.png')), resize_img)

        ocr_string=crnn.crnn_process(len(boxes))
                #ocr_result=sp.divide(len(boxes)-1)
                #print(box[:8])
                #print (len(boxes)+ocr_result[0])

        for i, box in enumerate(boxes): 
                #print "原图中位置",int(np.mat(box[:4]).mean(1)/rh), int(np.mat(box[5:8]).mean(1)/rw), "字数",ocr_result[i],"字符", ocr_result[len(boxes)+i],'\n'

        	    cv2.polylines(img, [box[:8].astype(np.int32).reshape((-1, 1, 2))], True, color=(0, 255, 0), thickness=2)

            #img = cv2.resize(img, None, None, fx=1.0 / rh, fy=1.0 / rw, interpolation=cv2.INTER_LINEAR)
        cv2.imwrite(os.path.join(FLAGS.output_path, os.path.basename(im_fn)), img[:, :, ::-1])


        with open(os.path.join(FLAGS.output_path, os.path.splitext(os.path.basename(im_fn))[0]) + ".txt",
                  "w") as f:
            for i, box in enumerate(boxes):
                line = ",".join(str(box[k]) for k in range(8))
                line += "," + str(scores[i]) + "\r\n"
                f.writelines(line)


        cost_time = (time.time() - start)
        print("cost time: {:.2f}s".format(cost_time)) 
        return ocr_recResponse(ocr_string)
	


def resize_image(img):
    img_size = img.shape
    im_size_min = np.min(img_size[0:2])
    im_size_max = np.max(img_size[0:2])
    im_scale = float(600) / float(im_size_min)
    if np.round(im_scale * im_size_max) > 1200:
        im_scale = float(1200) / float(im_size_max)
    new_h = int(img_size[0] * im_scale)
    new_w = int(img_size[1] * im_scale)
    if im_size_max < 600:
        new_h = new_h // 2
        new_w = new_w // 2
    new_h = new_h if new_h // 16 == 0 else (new_h // 16 + 1) * 16
    new_w = new_w if new_w // 16 == 0 else (new_w // 16 + 1) * 16
    re_im = cv2.resize(img, (new_w, new_h), interpolation=cv2.INTER_LINEAR)
    return re_im, (float(new_h) / img_size[0], float(new_w) / img_size[1])


def dumpRotateImage(img,pt1,pt2,pt3,pt4):
    copy_img=cv2.cvtColor(img, cv2.COLOR_RGB2GRAY) 
    degree=np.arctan((pt2[1]-pt1[1])/(pt2[0]-pt1[0]))
    height,width=copy_img.shape[:2]
    heightNew = int(width * fabs(sin(degree)) + height * fabs(cos(degree)))
    widthNew = int(height * fabs(sin(degree)) + width * fabs(cos(degree)))
    matRotation=cv2.getRotationMatrix2D((width/2,height/2),degrees(degree),1)
    matRotation[0, 2] += (widthNew - width) / 2
    matRotation[1, 2] += (heightNew - height) / 2
    imgRotation = cv2.warpAffine(copy_img, matRotation, (widthNew, heightNew), borderValue=(255, 255, 255))
    pt1 = list(pt1)
    pt3 = list(pt3)
    
    
    [[pt1[0]], [pt1[1]]] = np.dot(matRotation, np.array([[pt1[0]], [pt1[1]], [1]]))
    [[pt3[0]], [pt3[1]]] = np.dot(matRotation, np.array([[pt3[0]], [pt3[1]], [1]]))
    imgOut=imgRotation[int(pt1[1]-15):int(pt3[1]+15),int(pt1[0]-10):int(pt3[0]+10)]


    #对比度增强
    Imin, Imax = cv2.minMaxLoc(imgOut)[:2]
    if Imax == Imin:
	return imgOut
    else:
        Omin, Omax = 0, 255
        a = float(Omax - Omin) / (Imax - Imin)
        b = Omin - a * Imin
        out = a * imgOut + b
        out = out.astype(np.uint8)


    #边缘逼近
    retval, imgOut = cv2.threshold(out, 0, 255, cv2.THRESH_OTSU)
    mean_x=np.mean(imgOut,axis=1)
    mean_y=np.mean(imgOut,axis=0)
    height,width=imgOut.shape[:2]
    countf=0
    countr=width-1
    countt=0
    countb=height-1
    for i in range (0,width):
	if mean_y[i]>100:
		countf=i
		break
    for i in range (width-1,-1,-1):
	if mean_y[i]>100:
		countr=i
		break
    for i in range (0,height):
	if mean_x[i]>100:
		countt=i
		break
    for i in range (height-1,-1,-1):
	if mean_x[i]>100:
		countb=i
		break
    imgprint=out[(countt+2):(countb-2),(countf+2):(countr-2)]
    #print height, width
    return imgprint


def main(argv):
    temp = Recognizer()




if __name__ == '__main__':
    tf.app.run()
