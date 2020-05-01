#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import division 

import numpy as np
import cv2
import warnings
import matplotlib.pyplot as plt
import math
import copy
import preprocess

'''
Define polyfit function, delete the farthest 15% points first
'''
def polyfit(x,y):
    x = np.array(x)
    y = np.array(y)
    n = x.size
    clearnum = int(n/20)
    
    # 3 round total with 5% points each round
    for iteration in range(3):
        z = np.polyfit(x,y,1)
        yc = x * z[0] + z[1]
        # compute delta
        deltay = abs(yc-y)
        # delete points
        for i in range(clearnum):
            argmax = deltay.argmax()
            a = x[argmax]
            b = y[argmax]
            x = np.delete(x,argmax)
            y = np.delete(y,argmax)
            deltay = np.delete(deltay,argmax)
    # fit again
    zn = np.polyfit(x,y,1)
    return zn

def crosspoint(k1,b1,k2,b2):
    x = (b2-b1) / (k1-k2)
    y = (k1 * b2 - k2 * b1) / (k1-k2)
    point = [x,y]
    return point

def extract_roi(meter_num):
    print('[3] extract roi...')
##    print('Dealing with Meter ' + str(meter_num) + '...')
    meter_img_path = '/home/youibot/mrobot/src/mmrobot/mm_control/mm_pattern_recognition/src/meter_recognition/meter/meter-' + str(meter_num) + '.bmp'
    meter_img = cv2.imread(meter_img_path)
##    cv2.imshow('meter_' + str(meter_num), meter_img)
    
    ## 3-1 blur and gray
    grayimg = cv2.cvtColor(meter_img, cv2.COLOR_BGR2GRAY)
    blurimg = cv2.GaussianBlur(grayimg, (5,5), 0)
##    cv2.imshow('blur_' + str(meter_num), blurimg)
    total_h, total_w = grayimg.shape[:2]

    ## 3-2 separate into 9 parts and save 4 parts
    h1, w1 = total_h // 3, total_w // 3
    h2, w2 = h1 * 2, w1 * 2
    crop_part = []  # save 4 parts
    crop1 = blurimg[0:h1, w1:w2]
    crop2 = blurimg[h1:h2, 0:w1]
    crop3 = blurimg[h2:total_h, w1:(w1+w2)//2]
    crop4 = blurimg[h1:(h1+h2)//2, w2:total_w]
    crop_part.extend([crop1,crop2,crop3,crop4])
    
    ## 3-3 operate each part and get 4 centerlines
    nihe = cv2.cvtColor(grayimg, cv2.COLOR_GRAY2BGR)
    center_line = []
    for region_no in range(0,4):
##        print('Dealing with region ' + str(region_no))
        crop1 = crop_part[region_no]
        hc, wc = crop1.shape[:2]
        
        # [1] mean value -> shift
        shift = crop1.mean()
        # [2] gray hist
        blackvaluemax = int(crop1.mean() * 0.6)
        hist_cv = cv2.calcHist([crop1], [0], None, [blackvaluemax], [0,blackvaluemax])
        hlim = hist_cv.max() // 5
        # [3] find the peak value and the low bound
        black_hlim = black_center = 0 
        for i in range(0, blackvaluemax):
            imin = i - 5 if i >=5 else 0
            imax = i + 5 if i <=blackvaluemax-5 else blackvaluemax
            if hist_cv[i] > hlim and np.argmax(hist_cv[imin:imax]) == i-imin:
                black_center = i
                blackhlim = hist_cv[i] // 4 
                break
        # [4] find the limit range of black region
        blacklim1 = blacklim2 = black_center
        # max region should be +-10
        maxregion = 10
        while maxregion and hist_cv[blacklim1] >= blackhlim and blacklim1 > 0:
            blacklim1 -= 1
            maxregion -= 1
        maxregion = 10
        while maxregion and hist_cv[blacklim2] >= blackhlim and blacklim2 < blackvaluemax:
            blacklim2 += 1
            maxregion -= 1
        maxregion = 10
        blacklim = range(blacklim1 + 1, blacklim2)
        croprgb = cv2.cvtColor(crop1,cv2.COLOR_GRAY2BGR)
        # [5] comfirm mask based on black region
        mask = (crop1 > blacklim1) * (crop1 < blacklim2)
        mask = np.where(mask,1,0)
        mask = mask.astype(np.uint8)
        # [6] use dilate and erode
        # region(1,3) is (sum1)，region(0,2) is (sum0)
        blacknum = mask.sum(region_no % 2) # width
        blackregionwidth = blacknum.max() # max width
        ksize = int(blackregionwidth * 2 // 3)
        if ksize >= 3:
            kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (ksize,ksize))
            mask = cv2.dilate(mask, kernel)
            mask = cv2.erode(mask, kernel,iterations = 2)
            mask = cv2.dilate(mask, kernel)

        # [7] find centerline 
        blacknum = mask.sum(region_no % 2)
        loc = np.zeros(mask.shape, np.uint8)
        
        # 1,3区
        if region_no % 2:
            for i in range(wc):
                loc[:,i] += i
            loc = loc * mask
            x,y = [], []
            # centerpoint of each line
            for i in range(hc):
                if blacknum[i] != 0:
                    x.append(i)
                    y.append(loc.sum(1)[i] / blacknum[i])
            if region_no == 1:
                x = [xi + h1 for xi in x]
                y = y
            elif region_no == 3:
                x = [xi + h1 for xi in x]
                y = [yi + w2 for yi in y]
            # centerline
            z = polyfit(x,y)
            loc1 = int(z[1])
            loc2 = int(z[0]*total_h+z[1])
            # transform the coordinate system
            if abs(z[0]) <= 0.000001:
                z[0] = 999999
            else:
                z[0] = 1 / z[0]
            z[1] = -z[1] * z[0]
##            cv2.line(nihe, (loc1,0), (loc2,total_h), (0,0,255), 1, cv2.LINE_AA)
        # 0,2区
        else:
            for i in range(hc):
                loc[i]+=i
            loc = loc * mask
            x,y = [], []
            for i in range(wc):
                if blacknum[i] != 0:
                    x.append(i)
                    y.append(loc.sum(0)[i] / blacknum[i])
            if region_no == 0:
                x = [xi + w1 for xi in x]
                y = y
            if region_no == 2:
                x = [xi + w1 for xi in x]
                y = [yi + h2 for yi in y]
            z = polyfit(x,y)
            loc1 = int(z[1])
            loc2 = int(z[0]*total_w+z[1])
##            cv2.line(nihe, (0,loc1), (total_w,loc2), (0,0,255), 1, cv2.LINE_AA)
##        cv2.imshow('nihe', nihe)
        center_line.append([z[0],z[1]])

    ## 3-4 get the center_lines' crosspoint
    point_frame = []
    # point1,2,3,4 stands for left_up, left_down, right_up, right_down
    point1 = crosspoint(center_line[0][0],center_line[0][1],center_line[1][0],center_line[1][1])
    point2 = crosspoint(center_line[1][0],center_line[1][1],center_line[2][0],center_line[2][1])
    point3 = crosspoint(center_line[2][0],center_line[2][1],center_line[3][0],center_line[3][1])
    point4 = crosspoint(center_line[3][0],center_line[3][1],center_line[0][0],center_line[0][1])

    ## 3-5 get gray-grad distribution map
    shiftmean = int((255-shift/4)*2/3)
    meanimg = cv2.blur(grayimg,(5,5))
    meanimg = meanimg.astype(np.int16)
    gradimg_x = cv2.Sobel(blurimg, cv2.CV_16S, 1, 0)
    gradimg_y = cv2.Sobel(blurimg, cv2.CV_16S, 0, 1)
    gradimg_absx = cv2.convertScaleAbs(gradimg_x)
    gradimg_absy = cv2.convertScaleAbs(gradimg_y)
    gradimg = cv2.addWeighted(gradimg_absx,0.5,gradimg_absy,0.5,0)
    gradimg_mean = cv2.blur(gradimg,(3,3))
    grad_mean_img = gradimg/(meanimg+shiftmean) #灰度梯度分布图

    ## 3-6 get the inside_frame
    inside_frame = []
    # the 5th line is the up edge of up frame
    for linenum in range(0,5):
##        print('Dealing with line ' + str(linenum) + '...')
        # [1] set the start and end point 
        if linenum in [0,4]:
            line1_start = point1[0]
            line1_stop = point4[0]
        elif linenum == 1:
            line1_start = point1[1]
            line1_stop = point2[1]
        elif linenum == 2:
            line1_start = point2[0]
            line1_stop = point3[0]
        elif linenum == 3:
            line1_start = point4[1]
            line1_stop = point3[1]
        line1_len = line1_stop - line1_start
        if linenum in [4]:
            line1_start += line1_len /4
        else:
            line1_start += line1_len / 8
        if linenum in [4]:
            line1_stop -= line1_len / 4
        else:
            line1_stop -= line1_len / 8
        line1_start = math.floor(line1_start)
        line1_stop = math.floor(line1_stop)

        # [2] Getting Neighborhood Maximum
        newx,newy = [],[]
        searchrange = (total_w+total_h) // 35
        # up and down frame
        if linenum in [0,2,4]:
            for c in range(int(line1_start), int(line1_stop)):
                r = int(round(center_line[linenum%4][0]*c + center_line[linenum%4][1]))
                if linenum in [0,2]:
                    region = grad_mean_img[r:r+searchrange,c]
                    shift = region.argmax()
                    maxpoint = shift + r
                elif linenum == 4:
                    lowerbound = r - searchrange if r > searchrange else 0
                    region = grad_mean_img[lowerbound:r,c]
                    shift = region.argmax()
                    maxpoint = shift + r -searchrange
                newx.append(c)
                newy.append(maxpoint)
        # left and right frame
        elif linenum in [1,3]:
            for r in range(int(line1_start), int(line1_stop)):
                # y=kx+b, x=(y-b)/k
                hh = int(round((r-center_line[linenum][1])/center_line[linenum][0]))
                if linenum == 1:
                    region = grad_mean_img[r,hh:hh+searchrange]
                    shift = region.argmax()
                    maxpoint = shift + hh
                elif linenum == 3:
                    region = grad_mean_img[r,hh:hh+searchrange]
                    shift = region.argmax()
                    maxpoint = shift + hh
                newx.append(r)
                newy.append(maxpoint)
        # [3] polyfit
        zn = polyfit(newx,newy)
        inside_frame.append(zn)
        if linenum in [0]:
            loc1 = int(zn[1])
            loc2 = int(zn[0]*total_w+zn[1])
            cv2.line(nihe, (0,loc1), (total_w,loc2), (0,255,255), 1,cv2.LINE_AA)
        elif linenum in [1]:
            loc1 = int(zn[1])
            loc2 = int(zn[0]*total_h+zn[1])
            cv2.line(nihe, (loc1,0),(loc2,total_h),(0,255,255),1,cv2.LINE_AA)

    ## 3-7 get framewidth to correct down and right frame
    zn = inside_frame[0]
    left_pos1 = int(zn[1])
    left_pos2 = int(zn[0]*total_w+zn[1])
    zn = inside_frame[4]
    left_pos3 = int(zn[1])
    left_pos4 = int(zn[0]*total_w+zn[1])
    framewidth = abs(left_pos3 + left_pos4 - left_pos1-left_pos2)/2
    inside_frame[2][1]-= framewidth
    inside_frame[3][1]-= framewidth

    zn = inside_frame[2]
    loc1 = int(zn[1])
    loc2 = int(zn[0]*total_w+zn[1])
    cv2.line(nihe, (0,loc1), (total_w,loc2), (255,255,0), 1,cv2.LINE_AA)
    zn = inside_frame[3]
    loc1 = int(zn[1])
    loc2 = int(zn[0]*total_h+zn[1])
    cv2.line(nihe, (loc1,0),(loc2,total_h),(255,255,0),1,cv2.LINE_AA)
##    cv2.imshow('correct',nihe)
    cv2.imwrite('/home/youibot/mrobot/src/mmrobot/mm_control/mm_pattern_recognition/src/meter_recognition/frame/frame_' + str(meter_num) + '.bmp', nihe)

    ## 3-8 extract roi(reduce 2 framewidth)
    roiframe = copy.deepcopy(inside_frame)
    roiframe[0][1] += round(framewidth * 5/3)
    roiframe[2][1] -= round(framewidth * 5/3)
    roiframe[1][1] += round(framewidth * 5/3)
    roiframe[3][1] -= round(framewidth * 5/3)

    # coordinate transformation
    rk1 = roiframe[1][0]
    rb1 = roiframe[1][1]
    if abs(rk1) <= 0.000001:
        rk1 = 999999
    else:
        rk1 = 1/ rk1
    rb1 = -rb1 * rk1
    rk3 = roiframe[3][0]
    rb3 = roiframe[3][1]
    if abs(rk3) <= 0.000001:
        rk3 = 999999
    else:
        rk3 = 1/ rk3
    rb3 = -rb3 * rk3
    
    point1 = crosspoint(roiframe[0][0],roiframe[0][1],rk1,rb1)
    point2 = crosspoint(rk1,rb1,roiframe[2][0],roiframe[2][1])
    point3 = crosspoint(roiframe[2][0],roiframe[2][1],rk3,rb3)
    point4 = crosspoint(rk3,rb3,roiframe[0][0],roiframe[0][1])
    box = np.array([[point1],[point2],[point3],[point4]])
    box = np.int0(box)
    xr,yr,wr,hr = cv2.boundingRect(box)
    roiimg = grayimg[yr:yr+hr,xr:xr+wr]
    cv2.imwrite('/home/youibot/mrobot/src/mmrobot/mm_control/mm_pattern_recognition/src/meter_recognition/roi/roi_'+str(meter_num)+'.bmp',roiimg)

    return inside_frame,xr,yr

