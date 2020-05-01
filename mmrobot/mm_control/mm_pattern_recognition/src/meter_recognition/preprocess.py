from __future__ import division, print_function

import numpy as np
import cv2
import warnings
import matplotlib.pyplot as plt
import math
import copy
import time

def preprocess(src_img):
    ## 1-1 read src
##    print('1-1 src loading...')
    
    ## 1-2 resize(width=960)
##    print('1-2 resize...')
    src_h,src_w = src_img.shape[:2]
    if src_h > 960:
        resize_img = cv2.resize(src_img, (960,960*src_h//src_w), interpolation = cv2.INTER_AREA)
##    cv2.imwrite('resize_' + file + '.bmp', resize_img)
    else:
        resize_img = src_img.copy()
    ## 1-3 GaussianBlur,Gray
##    print('1-3 blur, gray...')
    blur_img = cv2.GaussianBlur(resize_img, (3,3), 0)
    gray_img = cv2.cvtColor(blur_img, cv2.COLOR_BGR2GRAY)

    ## 1-4 Sobel
##    print('1-4 Sobel...')
    grad_img_x = cv2.Sobel(gray_img, cv2.CV_16S, 1, 0)
    grad_img_y = cv2.Sobel(gray_img, cv2.CV_16S, 0, 1)
    grad_img_absX = cv2.convertScaleAbs(grad_img_x)
    grad_img_absY = cv2.convertScaleAbs(grad_img_y)
    grad_img = cv2.addWeighted(grad_img_absX, 0.5, grad_img_absY, 0.5, 0)
    
    ## 1-5 otsu, close
##    print('1-5 Otsu,close...')
    ret,otsu_img = cv2.threshold(grad_img,0,255,cv2.THRESH_OTSU)
    kernal = cv2.getStructuringElement(cv2.MORPH_RECT, (3,3))
    closed_img = cv2.morphologyEx(otsu_img, cv2.MORPH_CLOSE, kernal)

    return [closed_img,resize_img]

def Find_meters(src_img):


    ## 1 Preprocess
    print('[1] Preprocess...')
    pre_img,clip_source_img = preprocess(src_img)
    total_h, total_w = pre_img.shape[:2]

    ## 2 Find meters
    print('[2] find meters...')
    
    ## 2-1 find contours and select
    def verify(minrect):
        ## judge all the minrect
        ## (1)judge area
        width, height = minrect[1]
        area = width * height
        area_min = 480 * 480 / 36
        if area < area_min:
            return False
        else:
            ## (2) judge r
            r = width / height
            r = r if r >= 1 else 1/r
            if r >= 2.3:
                return False
        return True
    contours_img, contours, hierarchy = cv2.findContours(pre_img,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    rgb_img = np.zeros(contours_img.shape, np.uint8)
    rgb_img = cv2.cvtColor(rgb_img, cv2.COLOR_GRAY2BGR)
    contours_num = len(contours)
    verified_id = []  # save ids of verified contour
    verified_boundingrect = []
    for id in range(contours_num):
        cnt = contours[id]
        # find the minarearect and bounding rect of each contour
        minrect = cv2.minAreaRect(cnt)
        if verify(minrect):
            verified_id.append(id)
            boundingrect = cv2.boundingRect(contours[id])
            verified_boundingrect.append(boundingrect)

    ## 2-2 find the max_rect(score system)
    def isInside(rect1, rect2):
        ## judge whether rect1 is included in rect2
        x1,y1,w1,h1 = rect1
        x2,y2,w2,h2 = rect2
        # (x2,x2+w2)<(x1,x1+w1) and (y2,y2+h2)<(y1,y1+h1)
        if (x1<=x2 and (x2+w2)<=(x1+w1)) and (y1<=y2 and (y2+h2)<=(y1+h1)):
            return True
        else:
            return False
    verify_num = len(verified_id)
##    print('select ' + str(verify_num) + ' contours')
    score = [0] * verify_num
    max_rect = []
    for i in range(verify_num):
        for j in range(i+1, verify_num):
            # which is inside, its score +1
            if isInside(verified_boundingrect[i], verified_boundingrect[j]):
                score[i] += 1
            elif isInside(verified_boundingrect[j], verified_boundingrect[i]):
                score[j] += 1
    for i in range(verify_num):
        # the outside rect's score must be 0
        if score[i] == 0:
            # exclude the edge rect
            x,y,h,w = verified_boundingrect[i]
            if (x <= 10 or y <= 10 or x+w >= total_w-10 or y+h >= total_h-10):
                continue
            else:
                max_rect.append(verified_boundingrect[i])
    if len(max_rect)>1 and max_rect[1][0]<max_rect[0][0]:    
        max_rect.reverse()
    meter_num = len(max_rect)
    #print(max_rect)
    print('find ' + str(meter_num) + ' meters')
    
    ## 2-3 according to the max_rect, clip the meter's region(add 1/3 space)
##    clip_source_img = cv2.imread('resize_' + filename + '.bmp')
    for i in range(meter_num):
        meter = max_rect[i]
        x,y,w,h = meter
        add = round(w/3)
        newx1 = max(x - add, 0)
        newy1 = max(y - add, 0)
        newx2 = min(x + w + add, total_w)
        newy2 = min(y + h + add, total_h)
        crop = clip_source_img[int(newy1):int(newy2), int(newx1):int(newx2)]
        cv2.imwrite("/home/youibot/mrobot/src/mmrobot/mm_control/mm_pattern_recognition/src/meter_recognition/meter/meter-" + str(i+1) + ".bmp", crop)

    return meter_num


# message of source file
def separation(src_img):
    meter_num = Find_meters(src_img)
    return meter_num
