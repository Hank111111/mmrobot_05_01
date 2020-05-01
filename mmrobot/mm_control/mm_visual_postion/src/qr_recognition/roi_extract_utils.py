from __future__ import division
# coding: utf-8
import numpy as np
import cv2

def createSlider(name, params, default_callback=(lambda a:0)):
    # opencv gui setup
    cv2.namedWindow(name, cv2.WINDOW_NORMAL)
    cv2.namedWindow(name)
    for param_name in params.keys():
        param = params[param_name]
        cv2.createTrackbar(param_name, name, 0,param["max"], param.get("callback", default_callback))
        cv2.setTrackbarPos(param_name, name, param["init_value"])

def rectBoxToROI(rect, original_img_shape):
    max_length = max(rect[1])
    center_point = rect[0]
    padding = 10
    #width and height is inverse of line and colomn
    pos = np.array([center_point[1] - max_length, center_point[1]+max_length,            center_point[0] - max_length,center_point[0] + max_length])
    pos = pos.astype(np.int)
    pos[0:2] = np.clip(pos[0:2], 0, original_img_shape[0]-1)
    pos[2:4] = np.clip(pos[2:4], 0, original_img_shape[1]-1)
    return pos
class EdgesProcessor():
    
    def __init__(self):
        self.window_name = "edge"
        big_value = 1500
        little_value = 500
        half_kernel_size = 2
        self.params={
            "big_value":{"min":0, "max":10000, "init_value":big_value},
            "little_value":{"min":0, "max":10000, "init_value":little_value},
            "half_kernel_size":{"min":0, "max":3, "init_value":half_kernel_size}
        }
        createSlider(self.window_name,self.params)
        
        
    def getEdges(self, gray, visualisation=False):
        cv2.imshow("gray", gray)
        cv2.waitKey(0)
        self.visualisation = visualisation
        
        if visualisation:
            half_kernel_size = cv2.getTrackbarPos("half_kernel_size", self.window_name)
            if half_kernel_size<1: 
                half_kernel_size=1
            big_value = cv2.getTrackbarPos("big_value", self.window_name)
            little_value = cv2.getTrackbarPos("little_value", self.window_name)
        edges = cv2.Canny(gray,little_value,big_value,apertureSize=(half_kernel_size*2+1))
        if visualisation:
            cv2.imshow(self.window_name, edges)
        return edges

def drawPosBox(pos, img, color=(0,0,255), line_width=2):
    center = ((pos[2] + pos[3])/2.0, (pos[0] + pos[1])/2.0)
    size = ( pos[3] - pos[2], pos[1] - pos[0])
    rect = (center,size, 0)
    box = cv2.boxPoints(rect)
    box = np.int0(box)
    cv2.drawContours(img, [box], -1, color, line_width)

class RectROIFilter():
    def __init__(self):
        pass
    @staticmethod
    def union(a,b):
        y_min = min(a[0], b[0])
        y_max = max(a[1], b[1])
        x_min = min(a[2], b[2]) 
        x_max = max(a[3], b[3]) 
        return (y_min, y_max, x_min, x_max)

    @staticmethod
    def intersection(a,b):
        y_min = max(a[0], b[0])
        y_max = min(a[1], b[1])
        x_min = max(a[2], b[2]) 
        x_max = min(a[3], b[3]) 

        if y_min>y_max or x_min>x_max: return None
        return  (y_min, y_max, x_min, x_max)

    def getOverlapArea(self, A,B):    
        new_rect = self.intersection(A,B) 
        if new_rect is None:
            return 0
        else:
            return self.getArea(new_rect)

    @staticmethod
    def getArea(pos):
        return (pos[1] - pos[0]) * (pos[3] - pos[2])
    '''
    def filter(self, rect_list, original_image_shape, func=rectBoxToROI):
        pos_list = []
        for i in range(len(rect_list)):
            pos_list.append(func(rect_list[i], original_image_shape))
        origin_pos_list = list(pos_list)
        repeat = True
        while(repeat):
            repeat = False
            for i in range(len(pos_list)):
                for j in range(len(pos_list)):
                    if j==i or pos_list[i] is None or pos_list[j] is None:
                        continue
                    if self.getOverlapArea(pos_list[i], pos_list[j]) / self.getArea(pos_list[j]) > 0.8:
                        pos_list[j] = self.union(pos_list[i] ,pos_list[j])
                        pos_list[i] = None
                        repeat = True

        new_pos_list = []
        for pos in pos_list:
            if pos is not None:
                new_pos_list.append(pos)
        return new_pos_list
    '''
    def filter(self, rect_list, original_image_shape, func=rectBoxToROI):
        pos_list = []
        for i in range(len(rect_list)):
            pos_list.append(func(rect_list[i], original_image_shape))

        pos_list_filtered = []
        for i in range(len(rect_list)):
            is_fused = False
            for j in range(len(pos_list_filtered)):
                if self.getOverlapArea(pos_list[i], pos_list_filtered[j]) / self.getArea(pos_list_filtered[j]) > 0.8:
                    pos_list_filtered[j] = self.union(pos_list[i] ,pos_list_filtered[j])
                    is_fused = True
                    break
            if not is_fused:
                pos_list_filtered.append(pos_list[i])

        return pos_list_filtered

class contourProcessor():
    def __init__(self):
        self.window_name = "square"
        min_area = 300
        approxDP_epsilon_100 = 38
        rect_epsilon_100 = 500
        self.params={
            "min_area":{"min":0, "max":500, "init_value":min_area},
            "approxDP_epsilon_100":{"min":0, "max":1000, "init_value":approxDP_epsilon_100},
            "rect_epsilon_100":{"min":0, "max":1000, "init_value":rect_epsilon_100}}
        createSlider(self.window_name,self.params)
        
        
    def getContours(self, edge, origin_image, visualisation=True, roi_pos=None):
        min_area = cv2.getTrackbarPos("min_area", self.window_name)
        approxDP_epsilon_100 = cv2.getTrackbarPos("approxDP_epsilon_100", self.window_name)
        rect_epsilon_100 = cv2.getTrackbarPos("rect_epsilon_100", self.window_name)

        im2,contours_list,hierarchy = cv2.findContours(edge, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

       
        rect_list = []
        gray = cv2.cvtColor(origin_image, cv2.COLOR_BGR2GRAY)
        img = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        for countours in contours_list:
            if cv2.contourArea(cv2.convexHull(countours)) > min_area:
                approx = cv2.approxPolyDP(countours,approxDP_epsilon_100/100,True)
                hull = cv2.convexHull(approx)
                rect = cv2.minAreaRect(hull)
                #verify its rectengle (width should be equal to height)
                width_height_ratio = rect[1][0]/rect[1][1]

                if width_height_ratio < 0.7 or width_height_ratio > 1.0/0.7 :
                    continue
                #if rectFilterAccumulate(rect, edge):
                rect_list.append(rect)
                box = cv2.boxPoints(rect)
                box = np.int0(box)
                cv2.drawContours(img, [box], -1, (0, 0, 255), 1)
                cv2.imshow(self.window_name, img)
                
        if visualisation:
            cv2.imshow(self.window_name, img)
        
        return rect_list

class ROIExtractor():
    def __init__(self):
        #self.edge_processor = EdgesProcessor()
        #self.contour_processor = contourProcessor()
        self.roi_filter = RectROIFilter()
    def getContours(self, edge):
        min_area = 1000
        approxDP_epsilon_100 = 100.0

        im2,contours_list,hierarchy = cv2.findContours(edge, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

       
        rect_list = []
        for countours in contours_list:
            if cv2.contourArea(cv2.convexHull(countours)) > min_area:
                approx = cv2.approxPolyDP(countours,approxDP_epsilon_100/100.0,True)
                hull = cv2.convexHull(approx)
                rect = cv2.minAreaRect(hull)
                if rect[1][1] <= 1:
                     continue
                width_height_ratio = rect[1][0]/rect[1][1]
                if width_height_ratio < 0.7 or width_height_ratio > 1.0/0.7 :
                    continue
                #if rectFilterAccumulate(rect, edge):
                rect_list.append(rect)

        
        return rect_list
    def extract(self,image):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        # Taking a matrix of size 5 as the kernel 
        #gray = cv2.dilate(gray, kernel, iterations=1) 
        #cv2.imshow("gray_dilated", gray)
        #cv2.waitKey(0)
        edge = cv2.Canny(gray,500,1500,apertureSize=5)
        #edge = self.edge_processor.getEdges(gray, True)
        #cv2.waitKey(0)   
        #cv2.imshow("edge", edge)
        #cv2.waitKey(0)
        rect_list = self.getContours(edge)
        #rect_list = self.contour_processor.getContours(edge, image, True)
        # for visualization ,use self.contour_processor.getContours
        pos_list = self.roi_filter.filter(rect_list, image.shape, rectBoxToROI)
        return pos_list



