from __future__ import  print_function, division

import numpy as np
import cv2
import warnings
import matplotlib.pyplot as plt
import math
import copy
import time
import operator

class Point:
    def __init__(self, xpos, ypos):
        self.x = xpos
        self.y = ypos

class MyLine:
    def __init__(self, id, d, theta, l, begin, end):
        self.id = id
        self.d = d
        self.theta = theta
        self.l = l
        self.begin = begin
        self.end = end
        self.flag = 0

    # compare function
    def __lt__(self, other):
        return self.l < other.l
    def __eq__(self, other):
        return self.l == other.l
    def __gt__(self, other):
        return self.l > other.l

    def print(self):
        print('Line %d:(%d,%d),(%d,%d) d = %.2f, theta = %.2f, l = %.2f' %(self.id,self.begin.x,self.begin.y,self.end.x,self.end.y,self.d,self.theta,self.l))

def distance_p2p(p1,p2):
    ## calculate distance between point1 and point2
    delta_y = p1.y - p2.y
    delta_x = p1.x - p2.x
    return math.sqrt(delta_x ** 2 + delta_y ** 2)

def distance_p2l(P,A,B):
    ## calculate distance between pointP and line AB
    a = b = c = 0.0  # l:ax+by+c=0
    if A.x == B.x:
        a = 1.0
        b = 0.0
        c = -A.x
    else:
        delta_y = A.y - B.y
        delta_x = A.x - B.x
        k = delta_y / delta_x
        m = A.y - k * A.x
        a = k
        b = -1
        c = m
    # d = up/down
    up = abs(a * P.x + b * P.y + c)
    down = math.sqrt(a * a + b * b)
    return up / down

def angle(p1, p2):
    # calculate angle between p1 and p2
    if p1.x == p2.x:
        return -90.0
    else:
        delta_y = p1.y - p2.y
        delta_x = p1.x - p2.x
        theta = math.atan(delta_y / delta_x) / math.pi * 180
        return theta











        
