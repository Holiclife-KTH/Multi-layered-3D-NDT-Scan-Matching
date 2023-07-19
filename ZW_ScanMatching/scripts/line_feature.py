#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import time
import math as m

bridge = CvBridge()

class ShelfDetection():
    def __init__(self):
        self.l_pt_x = []
        self.l_pt_y = []
        self.r_pt_x = []
        self.r_pt_y = []

    def img_callback(self,data):
        try:
            self.cv2_img = bridge.imgmsg_to_cv2(data,"bgr8")
        except CvBridgeError as e:
            print(e)
        
        masked_image = self.RGB_color_selection(self.cv2_img)
        gray_img = cv2.cvtColor(masked_image, cv2.COLOR_RGB2GRAY)
        blur_img = cv2.GaussianBlur(gray_img, (5,5), 0)
        edge_detection_img = cv2.Canny(blur_img,50, 150)
        lines = self.hough_transform(edge_detection_img)
        self.l_pt_x.clear()
        self.l_pt_y.clear()
        self.r_pt_x.clear()
        self.r_pt_y.clear()
        if lines is not None: # 라인 정보를 받았으면
            for i in range(lines.shape[0]):
                if i == 0:
                    if lines[i][0][0] < int(self.cv2_img.shape[1]/2):
                        if (abs(m.atan2((lines[i][0][3] - lines[i][0][1]),(lines[i][0][2] - lines[i][0][0]))) > 0.2 and abs(m.atan2((lines[i][0][3] - lines[i][0][1]),(lines[i][0][2] - lines[i][0][0]))) < 1.4):
                            self.l_pt_x.append((lines[i][0][0], lines[i][0][2]))
                            self.l_pt_y.append((lines[i][0][1], lines[i][0][3]))

                    else:
                        if (abs(m.atan2((lines[i][0][3] - lines[i][0][1]),(lines[i][0][2] - lines[i][0][0]))) > 0.2 and abs(m.atan2((lines[i][0][3] - lines[i][0][1]),(lines[i][0][2] - lines[i][0][0]))) < 1.4):
                            self.r_pt_x.append((lines[i][0][0], lines[i][0][2]))
                            self.r_pt_y.append((lines[i][0][1], lines[i][0][3]))

                
                else:
                    ptx = lines[i][0][0]
                    pty = lines[i][0][1]
                    if ptx < int(self.cv2_img.shape[1]/2):
                        if len(self.l_pt_x) == 0:
                            if (abs(m.atan2((lines[i][0][3] - lines[i][0][1]),(lines[i][0][2] - lines[i][0][0]))) > 0.2 and abs(m.atan2((lines[i][0][3] - lines[i][0][1]),(lines[i][0][2] - lines[i][0][0]))) < 1.4):
                                self.l_pt_x.append((lines[i][0][0], lines[i][0][2]))
                                self.l_pt_y.append((lines[i][0][1], lines[i][0][3]))  
                                
                        else:
                            for j in range(len(self.l_pt_x)):
                                if (abs(self.l_pt_x[j][0] - ptx) < 200) and (abs(self.l_pt_y[j][0]-pty) < 80):

                                    ptx = 0
                                    pty = 0
                            
                            if not ((ptx == 0) and (pty == 0)):
                                
                                if (abs(m.atan2((lines[i][0][3] - lines[i][0][1]),(lines[i][0][2] - lines[i][0][0]))) > 0.2 and abs(m.atan2((lines[i][0][3] - lines[i][0][1]),(lines[i][0][2] - lines[i][0][0]))) < 1.4):
                                    self.l_pt_x.append((lines[i][0][0], lines[i][0][2]))
                                    self.l_pt_y.append((lines[i][0][1], lines[i][0][3]))
                       

                                    

                                
                    else:
                        if len(self.r_pt_y) == 0:
                            if (abs(m.atan2((lines[i][0][3] - lines[i][0][1]),(lines[i][0][2] - lines[i][0][0]))) > 0.2 and abs(m.atan2((lines[i][0][3] - lines[i][0][1]),(lines[i][0][2] - lines[i][0][0]))) < 1.4):
                                self.r_pt_x.append((lines[i][0][0], lines[i][0][2]))
                                self.r_pt_y.append((lines[i][0][1], lines[i][0][3]))     
                        else:
                            for j in range(len(self.r_pt_x)):
                                if (abs(self.r_pt_x[j][0] - ptx) < 200) and (abs(self.r_pt_y[j][0]-pty) < 80):
                                    ptx = 0
                                    pty = 0

                            if not ((ptx == 0) and (pty == 0)):
                                if (abs(m.atan2((lines[i][0][3] - lines[i][0][1]),(lines[i][0][2] - lines[i][0][0]))) > 0.2 and abs(m.atan2((lines[i][0][3] - lines[i][0][1]),(lines[i][0][2] - lines[i][0][0]))) < 1.4):
                                    self.r_pt_x.append((lines[i][0][0], lines[i][0][2]))
                                    self.r_pt_y.append((lines[i][0][1], lines[i][0][3]))            
                

            for i in range(len(self.l_pt_x)):
                a, b = self.cal_ab(self.l_pt_x[i], self.l_pt_y[i])
                cv2.line(self.cv2_img, (0, int(b)), (int(self.cv2_img.shape[1]/2), int(self.cv2_img.shape[1]/2*a+b)), (0, 0, 255), 2, cv2.LINE_AA)
                
            for i in range(len(self.r_pt_x)):
                a, b = self.cal_ab(self.r_pt_x[i], self.r_pt_y[i])
                cv2.line(self.cv2_img, (int(self.cv2_img.shape[1]/2), int(self.cv2_img.shape[1]/2*a+b)),(int(self.cv2_img.shape[1]), int(self.cv2_img.shape[1]*a+b)), (0, 0, 255), 2, cv2.LINE_AA)

        L_yellow_index = self.l_pt_y.index(max(self.l_pt_y))
        L_top_index = self.l_pt_y.index(min(self.l_pt_y))
        R_yellow_index = self.r_pt_y.index(max(self.r_pt_y))
        R_top_index = self.r_pt_y.index(min(self.r_pt_y))

        l_a, l_b = self.cal_ab(self.l_pt_x[L_yellow_index], self.l_pt_y[L_yellow_index])
        r_a, r_b = self.cal_ab(self.r_pt_x[R_yellow_index], self.r_pt_y[R_yellow_index])

        L_points = np.array([[self.l_pt_x[L_top_index][0], self.l_pt_y[L_top_index][0]], 
                             [self.l_pt_x[L_top_index][1], self.l_pt_y[L_top_index][1]], 
                             [self.l_pt_x[L_top_index][1], int(self.l_pt_x[L_top_index][1]*l_a+l_b)],
                             [self.l_pt_x[L_top_index][0], int(self.l_pt_x[L_top_index][0]*l_a+l_b)]], np.int32)
        
        R_points = np.array([[self.r_pt_x[R_top_index][0], self.r_pt_y[R_top_index][0]], 
                             [self.r_pt_x[R_top_index][1], self.r_pt_y[R_top_index][1]], 
                             [self.r_pt_x[R_top_index][1], int(self.r_pt_x[R_top_index][1]*r_a+r_b)],
                             [self.r_pt_x[R_top_index][0], int(self.r_pt_x[R_top_index][0]*r_a+r_b)]], np.int32)
        
        cv2.fillPoly(self.cv2_img, [L_points], color=(255, 0, 0, 1.0))
        cv2.fillPoly(self.cv2_img, [R_points], color=(255, 0, 0, 1.0))



        
        self.img_show(self.cv2_img)

        
    
    def cal_ab(self, x, y):
        a = (y[1]-y[0])/(x[1]-x[0])
        b = (x[1]*y[0]-x[0]*y[1])/(x[1]-x[0])

        return a, b

    def RGB_color_selection(self,image):
        """
        Apply color selection to RGB images to blackout everthing except for specified color you choose
        """

        lower_threshold = np.uint8([5,20,100])
        upper_threshold = np.uint8([40, 180, 185])
        orange_mask = cv2.inRange(image, lower_threshold, upper_threshold)

        lower_threshold = np.uint8([0, 185, 185])
        upper_threshold = np.uint8([40, 255, 255])
        yellow_mask = cv2.inRange(image, lower_threshold, upper_threshold)

        mask = cv2.bitwise_or(orange_mask, yellow_mask)
        masked_image = cv2.bitwise_and(image, image, mask=mask)
        
        return masked_image
    
    def hough_transform(self,image):
        rho = 1 #Distance resolution of the accumulator in pixels
        theta = np.pi/180 #Angle resolution of  the accumulator in radians
        threshold = 50 #Only lines that are greater than threshold will be returned
        minLineLength = 65 #Line segments shorter than that are rejected
        maxLineGap = 300 #Maximum allowed gap between points on the  same line to link them
        return cv2.HoughLinesP(image, rho=rho, theta=theta, threshold = threshold, minLineLength= minLineLength, maxLineGap=5)

    def mouse_callback(self, event,x,y,flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            value = param[y,x,:]
            print(value)

    def cal_ab(self, x, y):
        a = (y[1]-y[0])/(x[1]-x[0])
        b = (x[1]*y[0]-x[0]*y[1])/(x[1]-x[0])

        return a, b

    def img_show(self, image):
        cv2.imshow('img',image)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        


def main():
    rospy.init_node('Shelf_detection')
    shelf = ShelfDetection()
    rospy.Subscriber('/rgb', Image, shelf.img_callback)
    rospy.spin()
    
    
    


   
if __name__ == "__main__":
    main()
    

