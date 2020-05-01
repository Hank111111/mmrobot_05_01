#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import division, print_function
import roslib
import rospy
import numpy as np
import cv2
import warnings
import matplotlib.pyplot as plt
import math
import copy
import preprocess
import extraction
from Line import *
import os
import shutil

from std_msgs.msg import String,Int8,Int8MultiArray
from mm_robot_decision.msg import VisualAppResponse
from mm_visual_postion.msg import AppInnerRequest
from sensor_msgs.msg import Image

import rospkg
from os.path import join
rospack = rospkg.RosPack()
utils_path = rospack.get_path('mm_pattern_recognition')
utils_path = join(utils_path, "src/utils")
import sys 
sys.path.append(utils_path)
print(utils_path)
from stereo_image_receiver import StereoImageReceiver

def distance(p1,p2):
    x,y = p1
    a,b = p2
    dis = (x-a)**2+(y-b)**2
    dis = math.sqrt(dis)
    return dis

def otsu(img):
    ## define otsu([1]flat into 1 dim [2]delete 0,255)
    a = img.flatten()
    i = 0
    while i < a.size:
        if a[i] in [0,255]:
            a = np.delete(a,i)
        else:
            i += 1
    ret,th = cv2.threshold(a,0,255,cv2.THRESH_OTSU)
    return ret

def line_rec(src):
    # [1] Line Detection with iteration
    h,w = src.shape[:2]
    mul = 12
    min_length = (h+w)/mul
    yuzhi = int(min_length)
    min_len = int(min_length / 2)
    max_gap = 4
##    print("yuzhi=",yuzhi,"minlen=",min_len)
    detectimg = src.copy()
    while True:
        lines = cv2.HoughLinesP(detectimg, 1, math.pi/180, yuzhi, None, min_len, max_gap)
        lines_num = lines.shape[0]
        # if lines' num is less than 5, add it
        if lines_num <= 5:
            mul += 0.1
            min_length = (h+w)/mul
            yuzhi = int(min_length)
            if min_len >= 4:
                min_len -= 1
            print('beishu=',mul,'yuzhi=',yuzhi,'minlen',min_len)
            print('line num is too small, detect again')
        elif lines_num >= 50:
            mul -= 0.1
            min_length = (h+w)/mul
            yuzhi = int(min_length)
            min_len += 1
            print('beishu=',mul,'yuzhi=',yuzhi,'minlen',min_len)
            print('line num is too large, detect again')
        else:
            break
    print('Detect',str(lines_num),'lines')
    linesimg = np.zeros(src.shape, dtype = np.uint8)
    for i in range(lines_num):
        x1,y1,x2,y2 = lines[i][0]
        cv2.line(linesimg, (x1,y1), (x2,y2), 255, 1)
##    cv2.imshow('Detect', linesimg)
    
    # [2] Line Filter(based on theta) and class
    line_id = 1
    total_l = max_l = 0.0  # save total length and maximum length
    mylines = []  # save all the lines
    for i in range(lines_num):
        x1,y1,x2,y2 = lines[i][0]
        begin = Point(x1,y1)
        end = Point(x2,y2)
        theta = angle(begin, end)
        # when theta>-4.0(0deg and below) or theta<-85.0(90deg and right), it is useful
        if (theta > -4.0 or theta < -85.0):
            line_length = distance_p2p(begin, end)
            d = distance_p2l(Point(0,0), begin, end)
            total_l += line_length
            if line_length > max_l:
                max_l = line_length
            myline = MyLine(line_id,d,theta,line_length,begin,end)
            mylines.append(myline)
            myline.print()
            line_id += 1

    # [3] sort all lines based on length
    mylines.sort(reverse = True)
    print('sort complete, begin clustering...')

    # [4] clustering
    sum_l = theta_standard = d_standard = 0.0  # save the clustered over length
    maxdislim = thetadislim = 5.0  # maximum length and theta gap
    finallinesets = []  # save the final clusering line set
    lineset_suml = []  # save the total_len in each line set
    for i in range(len(mylines)):
        if mylines[i].flag != 0:
            continue
        # set standard
##        mylines[i].print()
        lineseti = []
        sum_tol = 0
        startpoint = mylines[i].begin
        endpoint = mylines[i].end
        standid = mylines[i].id
        theta_stand = mylines[i].theta
        sum_tol += mylines[i].l
        mylines[i].flag = standid  # set flag to standid
        lineseti.append(mylines[i])
        # compare
        for j in range(i+1, len(mylines)):
            if mylines[j].flag == 0:  # only judge line which flag==0
                thetadis = abs(mylines[j].theta - theta_stand)
                if (thetadis <= thetadislim or thetadis >= 180.0 - thetadislim):
                    target_point1 = mylines[j].begin
                    target_point2 = mylines[j].end
                    pointdis1 = distance_p2l(target_point1, startpoint, endpoint)
                    if pointdis1 <= maxdislim:
                        pointdis2 = distance_p2l(target_point2, startpoint, endpoint)
                        if pointdis2 <= maxdislim:
                            # cluser myline_j to lineset_i
                            sum_tol += mylines[j].l
                            lineseti.append(mylines[j])
                            mylines[j].flag = standid
        # lineseti is clustered
        finallinesets.append(lineseti)
        lineset_suml.append(sum_tol)
    for i in range(len(finallinesets)):
        eachlineset = finallinesets[i]
        print('Lineset',i,': Line ',end='')
        for j in range(len(eachlineset)):
            print(eachlineset[j].id,'',end='')
        print(', suml =',lineset_suml[i])

    # [5] select 2 lineset which sum_tol are largest and second largest
    maxlenid = seclenid = maxsuml = secsuml = 0
    if len(lineset_suml) >= 2:
        for i in range(len(lineset_suml)):
            current = lineset_suml[i]
##            print('Lineset',str(i),'=',str(current))
            if current <= secsuml:
                continue
            else:
                if current < maxsuml:
                    secsuml = current
                    seclenid = i
                else:
                    secsuml = maxsuml
                    maxsuml = current
                    seclenid = maxlenid
                    seclenid = i
        final_line_set = finallinesets[int(maxlenid)]
        sec_line_set = finallinesets[int(seclenid)]
        print('Select',str(maxlenid),'lineset, it includes',str(len(final_line_set)),'lines')
        print('Select',str(seclenid),'lineset, it includes',str(len(sec_line_set)),'lines')
    else:
        final_line_set = finallinesets[int(maxlenid)]
        sec_line_set = None
        maxsuml = lineset_suml[int(maxlenid)]
        print('only detect 1 lineset')
        print('Select',str(maxlenid),'lineset, it includes',str(len(final_line_set)),'lines')

    # [6] use Weighted averaging method to get x bar, y bar and k1
    thetatol = xtol = ytol = 0
    finalpointset = []  # save each line's begin and end point
    for i in range(len(final_line_set)):
        li = final_line_set[i]
        thetatol += li.theta * li.l
        xtol = xtol + li.begin.x + li.end.x
        ytol = ytol + li.begin.y + li.end.y
        finalpointset.append((li.begin.x,li.begin.y))
        finalpointset.append((li.end.x,li.end.y))
    final_theta1 = thetatol / maxsuml
    final_x = xtol / (len(final_line_set) * 2)
    final_y = ytol / (len(final_line_set) * 2)
##    print('final para1:x=',final_x,'y=',final_y,'theta=',final_theta1)
    k1 = math.tan(final_theta1*math.pi/180)

    # [7] use Line Fitting method and get k2
    finalpointset = np.array(finalpointset)
    line_para = cv2.fitLine(finalpointset, cv2.DIST_L2,0,0.01,0.01)
    k2 = line_para[1] / line_para[0]

    # [8] get epsilon(only judge when sec_line_set exists)
    secondpointset = []
    if sec_line_set:
        for i in range(len(sec_line_set)):
            li = sec_line_set[i]
            secondpointset.append((li.begin.x,li.begin.y))
            secondpointset.append((li.end.x,li.end.y))
        secondpointset = np.array(secondpointset)
        line_para2 = cv2.fitLine(secondpointset, cv2.DIST_L2,0,0.01,0.01)
        sec_theta = math.atan(line_para2[1]/line_para2[0]) * 180 / math.pi
        epsilon = abs(sec_theta - final_theta1) * (secsuml/maxsuml)
    else:
        epsilon = 0
    para = [final_x,final_y,k1,k2[0],epsilon]
    print('Clustering Over,para=',para)
    return para

def pointer(meter_nums):
    meter_val=[]
    for meter_num in range(1, meter_nums+1):
        # [2-3] extration
        print('********** Dealing with Meter ' + str(meter_num) + '... **********')
        inside_frame,xr,yr = extraction.extract_roi(meter_num)
        
        ## 4 Otsu-Canny Combinary
        print('[4] Otsu-Canny Combinary...')
        roiimg_path = '/home/youibot/mrobot/src/mmrobot/mm_control/mm_pattern_recognition/src/meter_recognition/roi/roi_' + str(meter_num) + '.bmp'
        roiimg = cv2.imread(roiimg_path,0)
    ##    cv2.imshow('roi_' +str(meter_num), roiimg)
        img = cv2.GaussianBlur(roiimg, (3,3), 0)
        edge_output = cv2.Canny(img, 50, 150)
        h,w = img.shape[:2]

        ## 4-1 prepare the mask
        iteration = 1
        contrastret = contrastmean = 0
        tolnum = h * w
        num = tolnum
        roimask = np.ones((h,w))
        for i in range(h):
            for j in range(w):
                if i > h * 2/3 and j > w * 2/3:
                    roimask[i][j] = 0
                    continue
                if i < h // 10:
                    roimask[i][j] = 0
                    continue
                if i < h // 4 and j < w // 4:
                    roimask[i][j] = 0

        ## 4-2 begin iteration
        while iteration <= 8 and num >= tolnum / 50:
            print('Round ' + str(iteration))
            if iteration == 1:
                # the 1st round just use original otsu
                ret,th = cv2.threshold(img,0,255,cv2.THRESH_OTSU)
    ##            cv2.imshow('thr',th)
    ##            print('ret=',ret)
                contrastret = ret
            else:
                # [1] just use otsu to the remaining point
                temp = img * mask
                ret = otsu(temp)
    ##            # return the ret before enhancement
    ##            if ret <= 127:
    ##                ret = ret * contrastmean / 127.5
    ##            else:
    ##                ret = (ret - 127.5)*(contrastret-contrastmean)/127.5+contrastmean
    ##            ret = int(ret)
                
                ret,th = cv2.threshold(img,ret,255,cv2.THRESH_BINARY)
    ##            print('ret=',ret)
                contrastret = ret
               
            # [2] showimg is used to show(th==0 is useful)
            showimg = np.where(th==0,img,255)
    ##        cv2.imshow('thr'+str(iteration),showimg)
            # [3] nextimg is used to calculate
            nextimg = np.where(th==0,img,0)
            nextimg = nextimg * roimask
            # [4] mask is used to save the useful part(th===0 is useful)
            mask = np.where(th==0,1,0)
            mask = mask * roimask
            mask = mask.astype(np.uint8)
            # [5] get the number and the mean gray of nextimg
            num = mask.sum()
    ##        print('num=',num)
            mean = nextimg.sum()/num
    ##        print('mean=',mean)
            contrastmean = mean
            # [6] contrast enhancement
            # t<mean, t=127.5*t/mean  t>mean, t=127.5+127.5*(t-mean)/(ret-mean)
            img1 = nextimg.astype(np.float64)
            img = np.where(img1>mean,127.5+127.5*(img1-mean)/(ret-mean),127.5*img1/mean)
            img = img.astype(np.uint8)
            img = np.where(img, img, 255) # change 0 to 255
           
            iteration += 1

        ## 4-3 combine the canny and otsu binary img
        for i in range(h):
            for j in range(w):
                if mask[i][j]:
                    edge_output[i][j]=255

        ## 5 contour selection
        print('[5] contour selection...')

        def verify(minrect,w,h):
            ## judge all the minrect
            ## (1) judge area
            width, height = minrect[1]
            area = width * height
            if area <= 2:
                return False
            ## (2) judge r
            r = width / height
            r = r if r >= 1 else 1/r
            if r <= 1.2:
                return False
            ## (3) judge distance
            full_length = (w+h)/2
            max_dis = full_length * 2 / 3
            min_dis = full_length / 2
            box = cv2.boxPoints(minrect)
            vertex = []
            for i in range(len(box)):
                vertex.append((box[i][0],box[i][1]))
            vertex.append(minrect[0])
            score = [0,0,0]
            p0 = (w,h)
            for i in range(5):
                dist = distance(vertex[i],p0)
                if dist < min_dis:
                    score[0] += 1
                elif dist < max_dis:
                    score[1] += 1
                else:
                    score[2] += 1
            if (score[0]==5 or score[1]==5 or score[2]==5):
                return False

            return True
            
        edgeimg = edge_output.copy()
        houghimg = edge_output.copy()
        contours_img, contours, hierarchy = cv2.findContours(edgeimg,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        contours_num = len(contours)
        verified_id = []
        rgb_img = np.zeros(contours_img.shape, np.uint8)
        rgb_img = cv2.cvtColor(rgb_img, cv2.COLOR_GRAY2BGR)
        print('find ' + str(contours_num)+' contours')
        for id in range(contours_num):
            cnt = contours[id]
            minrect = cv2.minAreaRect(cnt)
            if verify(minrect, w, h):
                verified_id.append(id)
            else:
                cv2.drawContours(houghimg, contours, id, (0,0,0),-1,8)
    ##    cv2.imshow('houthimg',houghimg)
                
        ## 6 Line Detection and Clustering
        print('[6] Line Detection and Clustering')
        line_para = line_rec(houghimg)

        ## 7 perspective transformation
        print('[7] perspective transformation...')
        # [1] get the 5th line
        frame_img = cv2.imread("/home/youibot/mrobot/src/mmrobot/mm_control/mm_pattern_recognition/src/meter_recognition/frame/frame_" + str(meter_num) + ".bmp")
    ##    frame_img2 = frame_img.copy()
    ##    cv2.imshow('src'+str(meter_num),frame_img)
        x0,y0,k1,k2,epsilon = line_para
        h1,w1 = frame_img.shape[:2]
        x0 += xr
        y0 += yr
    ##    ah11 = int(y0-x0*k1)
    ##    ah21 = int(y0+(w1-x0)*k1)
        ah12 = int(y0-x0*k2)
        ah22 = int(y0+(w1-x0)*k2)
        # it seems that fitting method behaves better
        cv2.line(frame_img,(0,ah12),(w1,ah22),(0,0,255),1,cv2.LINE_AA)
    ##    cv2.line(frame_img2,(0,ah11),(w1,ah21),(0,255,255))    
    ##    cv2.imshow('linepic_fitting'+str(meter_num),frame_img)
    ##    cv2.imshow('linepic_weighted'+str(meter_num),frame_img2)
        cv2.imwrite("/home/youibot/mrobot/src/mmrobot/mm_control/mm_pattern_recognition/src/meter_recognition/measure/measure_"  + str(meter_num) + ".bmp",frame_img)

        # [2] get 4 corner points
        rk1 = inside_frame[1][0]
        rb1 = inside_frame[1][1]
        if abs(rk1) <= 0.000001:
            rk1 = 999999
        else:
            rk1 = 1/ rk1
        rb1 = -rb1 * rk1
        rk3 = inside_frame[3][0]
        rb3 = inside_frame[3][1]
        if abs(rk3) <= 0.000001:
            rk3 = 999999
        else:
            rk3 = 1/ rk3
        rb3 = -rb3 * rk3
        point1 = extraction.crosspoint(inside_frame[0][0],inside_frame[0][1],rk1,rb1)
        point2 = extraction.crosspoint(rk1,rb1,inside_frame[2][0],inside_frame[2][1])
        point3 = extraction.crosspoint(inside_frame[2][0],inside_frame[2][1],rk3,rb3)
        point4 = extraction.crosspoint(rk3,rb3,inside_frame[0][0],inside_frame[0][1])
        p5 = extraction.crosspoint(rk1,rb1,k2,-k2*x0+y0)
        p6 = extraction.crosspoint(rk3,rb3,k2,-k2*x0+y0)
    ##    cv2.imshow('fff'+str(meter_num),frame_img)
        
        # [3] get pointset1 -> pointset2
        pointset1 = np.float32([point1,point2,point3,point4])
        sidelen = int((distance(point1,point2)+distance(point2,point3)+distance(point3,point4)+distance(point4,point1))/4)
        pointset2 = [point1]
        pointset2.append([point1[0],point1[1]+sidelen])
        pointset2.append([point1[0]+sidelen,point1[1]+sidelen])
        pointset2.append([point1[0]+sidelen,point1[1]])
        pointset2 = np.float32(pointset2)

        # [4] perspective transform
        M = cv2.getPerspectiveTransform(pointset1,pointset2)
        dst = cv2.warpPerspective(frame_img,M,(w1,h1))

        p5 = np.array([p5[0],p5[1],1])
        p6 = np.array([p6[0],p6[1],1])
        x1,y1,z1 = M.dot(p5)
        x2,y2,z2 = M.dot(p6)
        pt5 = [x1/z1,y1/z1]
        pt6 = [x2/z2,y2/z2]
        cv2.circle(dst, (int(pt5[0]),int(pt5[1])),3,(255,0,255),-1)
        cv2.circle(dst, (int(pt6[0]),int(pt6[1])),3,(255,0,255),-1)
    ##    cv2.imshow('perspectiveimg'+str(meter_num),dst)

        ## 8 calculate the final angle of the pointer based on pt5, pt6
        k = (pt6[1]-pt5[1])/(pt6[0]-pt5[0])
        theta_star = math.atan(k)*180/math.pi
        
        if meter_num == 1:
            with open("/home/youibot/mrobot/src/mmrobot/mm_control/mm_pattern_recognition/src/meter_recognition/result/"+'1.txt','w') as f:
                f.write('Find '+str(meter_nums)+' Meters:' + '\n')
        with open("/home/youibot/mrobot/src/mmrobot/mm_control/mm_pattern_recognition/src/meter_recognition/result/"+'1.txt','a') as f:
            f.write('Meter '+str(meter_num)+': '+str(theta_star)+' +- '+str(epsilon)+'\n')
        print('********** meter',meter_num,'result is',theta_star,'deg **********')
        print('-----------------------------------------------------------')
        if theta_star > 2:
            meter_val.append(1)
        else:
            meter_val.append(0)
    return meter_val
        

def main():
    # find all the picture file in 'source' folder
##    filename = "source_06"
##    filetype = "jpg"
    rospy.init_node('meter_recognition')
    rospy.loginfo('started')
    rospy.Subscriber('/mm_visual/apps/meter/goal', AppInnerRequest, callback)

    global pubPhotoMeter
    pubPhotoMeter =rospy.Publisher('/mm_visual/apps/meter/result',VisualAppResponse,queue_size=10)

    rospy.spin()
    
stereo_image_receiver = StereoImageReceiver()
def callback(msg):
    if(msg.object_name != "pointMeter"):
        rospy.logerr("unknown cmd [%s] for point meter node", msg.object_name)

    left_roi = msg.left_roi    
    rospy.loginfo("received cmd for [%s]", msg.object_name)

    global pubPhotoMeter
    rospy.loginfo("try to recognize the point meters...") 
    response = VisualAppResponse()
    response.object_name = "pointMeter"
    response.pose.state = "null"
    response.frame_id = "endeffector"
    response.object_unique_id_on_equipment = msg.object_unique_id_on_equipment
    response.height = msg.object_height
    response.width = msg.object_width
    try:
        cv_image, right_image = stereo_image_receiver.getLatestImages()

        pattern = 100
        roi = [int(left_roi.y)-pattern, int(left_roi.y+left_roi.height)+pattern, 
                int(left_roi.x)-pattern, int(left_roi.x+left_roi.width)+pattern]
        
        meter_nums = preprocess.separation(cv_image[roi[0]:roi[1],roi[2]:roi[3]])
        #cv2.imshow("roi", cv_image[roi[0]:roi[1],roi[2]:roi[3]])
        #cv2.waitKey(0)
        response.object_status = pointer(meter_nums)
        if(len(response.object_status)<2):
            response.success = False
        else:
            response.success = True
   
        
    except CvBridgeError as e:
        response.success = False
        print(e)
    pubPhotoMeter.publish(response)
    print(response)

if __name__ == '__main__':
   main()
