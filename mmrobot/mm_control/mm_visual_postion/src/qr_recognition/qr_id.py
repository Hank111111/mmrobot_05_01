#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import division, print_function
import roslib
import rospy
import numpy as np
import cv2
import zbar
from PIL import Image as PIL_Image
import cv2
import numpy as np
from stereo_image_receiver import StereoImageReceiver
from rigid_transform_solver import solveRigidTransform
from mm_robot_decision.msg import VisualAppResponse
from mm_visual_postion.msg import AppInnerRequest
from scipy.spatial.transform import Rotation
from stereo_camera_arm_model import StereoCameraArmModel

from arm_transform_receiver import ArmTransformReceiver
from roi_extract_utils import ROIExtractor

class QR_Recognizer():
    def __init__(self, qr_length):
        # find all the picture file in 'source' folder
        ##    filename = "source_06"
        ##    filetype = "jpg"
        rospy.init_node('qr_recognition')
        rospy.loginfo('started')
        rospy.Subscriber('/mm_visual/apps/qr_code/goal', AppInnerRequest, self.callback)
        self.pub_photo_QR=rospy.Publisher('/mm_visual/apps/qr_code/result',VisualAppResponse,queue_size=1) 

        self.scanner = zbar.ImageScanner()
        #self.scanner.parse_config('enable')
        self.scanner.set_config(symbology=64, config=0)
        
        self.stereo_image_receiver = StereoImageReceiver()
        self.qr_length = qr_length

        self.roi_extractor = ROIExtractor()
        # debug use
        #self.arm_transform_receiver = ArmTransformReceiver()


    def publishResult(self, response):
        self.pub_photo_QR.publish(response)
        print(response)

    def callback(self, msg):
        print("received the msg for ", msg.object_name)
        print(msg)
        if msg.object_name.find('QRCode') >=0:
            print("process ")
            self.qr_length = msg.object_width
            
            left_image, right_image = self.stereo_image_receiver.getLatestImages()
            left_result = self.qrdeal(left_image)
            right_result = self.qrdeal(right_image)
            response = VisualAppResponse()

            response.frame_id = "endeffector"
            response.object_name = msg.object_name
            response.object_unique_id_on_equipment = msg.object_unique_id_on_equipment
            if(len(left_result) < 1):
                response.additional_text_info = "cannot detect any QR code in the left image"
                rospy.logwarn(response.additional_text_info)
                response.success = False
                self.publishResult(response)
                return
            if(len(right_result) < 1):
                response.additional_text_info = "cannot detect any QR code in the right image"
                rospy.logwarn(response.additional_text_info)
                response.success = False
                self.publishResult(response)
                return
            if(len(msg.additional_text_info) == 0):
                response.additional_text_info = "the target qr code's content is not indicated"
                rospy.logwarn(response.additional_text_info)
                response.success = False
                self.publishResult(response)
                return
            # check for left symbol
            left_symbol_data = None
            left_symbol_loc = None
            for symbol_data, location in left_result:
                print(type(symbol_data), type(msg.additional_text_info) , len(symbol_data), symbol_data, symbol_data == msg.additional_text_info)
                print(symbol_data.split(msg.additional_text_info))
                if(symbol_data == msg.additional_text_info):
                    left_symbol_data = symbol_data
                    left_symbol_loc = location
                    break
            if(left_symbol_data == None):
                response.additional_text_info = "the target qr code is not detected in left image"
                rospy.logwarn(response.additional_text_info)
                response.success = False
                self.publishResult(response)
                return
            # check for right symbol
            right_symbol_data = None
            right_symbol_loc = None
            for symbol_data, location in right_result:
                if(symbol_data == msg.additional_text_info):
                    right_symbol_data = symbol_data
                    right_symbol_loc = location
                    break
            if(right_symbol_data == None):
                response.additional_text_info = "the target qr code is not detected in right image"
                rospy.logwarn(response.additional_text_info)
                response.success = False
                self.publishResult(response)
                return
                

            response.success = True
            response.additional_text_info = right_symbol_data
            corners_in_left_image = np.array(left_symbol_loc).astype(np.float).transpose()
            corners_in_right_image = np.array(right_symbol_loc).astype(np.float).transpose()
            T_cam_to_qrcode, corners_in_cam_homo, scale = self.computeTransfromFromCamToQRCode(corners_in_left_image, corners_in_right_image, self.qr_length)
            T_endeffector_to_qrcode = np.dot(self.stereo_image_receiver.model.endeffector_to_cam_transform, T_cam_to_qrcode)
            r = Rotation.from_dcm(T_endeffector_to_qrcode[0:3,0:3])
            quat = r.as_quat()
            response.pose.state = "workSpacePlanning"
            response.pose.x = T_endeffector_to_qrcode[0,3] / 1000.0
            response.pose.y = T_endeffector_to_qrcode[1,3] / 1000.0
            response.pose.z = T_endeffector_to_qrcode[2,3] / 1000.0
            response.pose.a = quat[0]
            response.pose.b = quat[1]
            response.pose.c = quat[2]
            response.pose.w = quat[3]
            response.width = scale * self.qr_length
            response.height = scale * self.qr_length
            #print("scale, qr_length", scale, self.qr_length)
            corners_in_endeffector_homo = self.stereo_image_receiver.model.endeffector_to_cam_transform.dot(corners_in_cam_homo)
            response.additional_numerical_info += corners_in_endeffector_homo[:3, 0].tolist() # top left
            response.additional_numerical_info += corners_in_endeffector_homo[:3, 3].tolist() # top right
            response.additional_numerical_info += corners_in_endeffector_homo[:3, 2].tolist() # bottom right
            response.additional_numerical_info += corners_in_endeffector_homo[:3, 1].tolist() # bottom left
            print(corners_in_endeffector_homo)
            print(response.additional_numerical_info)
            self.publishResult(response)
            '''
            # debug
            # reprojection
            real_points = np.zeros((4,4))
            real_points[0,:] = np.array([-self.qr_length/2.0, -self.qr_length/2.0, 0, 1])
            real_points[1,:] = np.array([-self.qr_length/2.0, self.qr_length/2.0, 0,  1])
            real_points[2,:] = np.array([self.qr_length/2.0, self.qr_length/2.0, 0, 1])
            real_points[3,:] = np.array([self.qr_length/2.0, -self.qr_length/2.0, 0, 1])
            pts = []
            for i in range(4):
                pt_in_image = np.dot( self.stereo_image_receiver.model.right_camera.projection_mat, T_cam_to_qrcode).dot(real_points[i,:].transpose())
                pt_in_image /= pt_in_image[2]
                pts.append(tuple(pt_in_image[:2].astype(np.int).tolist())) 
            self.drawLines(pts, right_image)
            #cv2.imshow("right_image", right_image)
            #cv2.waitKey(0)
            
            print("T_cam_to_qrcode :\n", T_cam_to_qrcode)
            T_base_to_endeffector = self.arm_transform_receiver.getLatestTransformInMM()
            print(self.stereo_image_receiver.model.endeffector_to_cam_transform)
            T_base_to_qrcode = np.matmul(T_base_to_endeffector, self.stereo_image_receiver.model.endeffector_to_cam_transform).dot(T_cam_to_qrcode)
            
            print("T_base_to_qrcode :\n", T_base_to_qrcode)
            '''
        
        
    def drawLines(self, corners, image):
        cv2.line(image, corners[0], corners[1], (155,0,0),2)
        cv2.line(image, corners[1], corners[2], (155,0,155),2)
        cv2.line(image, corners[2], corners[3], (155,155,0),2)
        cv2.line(image, corners[3], corners[0], (0,155,155), 2)
        
    def computeTransfromFromCamToQRCode(self, corners_in_left_image, corners_in_right_image, qr_length):
        # corners should be in the order: topLeftCorners, bottomLeftCorners, bottomRightCorners, topRightCorners

        corners_in_cam_homo = cv2.triangulatePoints(self.stereo_image_receiver.model.left_camera.projection_mat, \
                                                self.stereo_image_receiver.model.right_camera.projection_mat, \
                                                corners_in_left_image, corners_in_right_image)
        # normalize corners_in_cam

        for i in range(4):
            corners_in_cam_homo[:,i] /= corners_in_cam_homo[3,i]
                 
        
        real_points = np.zeros((4,3))

        real_points[0,:] = np.array([-qr_length/2.0, -qr_length/2.0, 0]) # order is the same as zbar's output convention
        real_points[1,:] = np.array([-qr_length/2.0, qr_length/2.0, 0])
        real_points[2,:] = np.array([qr_length/2.0, qr_length/2.0, 0])
        real_points[3,:] = np.array([qr_length/2.0, -qr_length/2.0, 0])
        #print(corners_in_cam_homo.transpose())
        T_cam_to_qrcode, scale = solveRigidTransform(corners_in_cam_homo.transpose()[:,:3], real_points)
        #print("scale, qr_length", scale, qr_length)
        #print(T_cam_to_qrcode)
        return T_cam_to_qrcode, corners_in_cam_homo, scale

    def qrdeal(self, im):
        roi_list = self.roi_extractor.extract(im)
        result_symbols_data_loc = []
        for i, roi in enumerate(roi_list):
            gray = cv2.cvtColor(im[roi[0]:roi[1],roi[2]:roi[3]], cv2.COLOR_BGR2GRAY)
            gray = np.ascontiguousarray(gray)
            pil = PIL_Image.fromarray(gray)
            width, height = pil.size
            raw = pil.tobytes()
            image = zbar.Image(width, height, 'Y800', raw)
            ret = self.scanner.scan(image)
            
            for symbol in image.symbols:
                corners_loc = []
                for i in range(4):
                    corner = (symbol.location[i][0] + roi[2], symbol.location[i][1] + roi[0])
                    corners_loc.append(corner)

                result_symbols_data_loc.append((symbol.data.strip(), corners_loc)) # in case the last character is a space (sometimes)
                
                #print(symbol.data, symbol.location, corners_loc)
        #cv2.namedWindow("gray", 0)
        #cv2.imshow("gray", im)
        #cv2.waitKey(0)
            
            #print("ret",ret)

        return result_symbols_data_loc
        
if __name__ == '__main__':
    qr = QR_Recognizer(50)
    '''
    #image = cv2.imread("right_image.png")
    image = cv2.imread("left_image.png")
    
    model = StereoCameraArmModel()
    model.loadDefaultParams()
    image = model.rectifyLeftImage(image)
    
    #image = cv2.imread("qrcode.png")
    
    #cv2.namedWindow("img", 0)
    #cv2.imshow("img",image)
    #cv2.waitKey(0)
    
    
    result_symbols_roi = qr.qrdeal(image)
    print(len(result_symbols_roi))
    for symbol_data, corners_loc in result_symbols_roi:
        print(type(symbol_data),symbol_data)
        print(corners_loc)
    '''
    rospy.spin()