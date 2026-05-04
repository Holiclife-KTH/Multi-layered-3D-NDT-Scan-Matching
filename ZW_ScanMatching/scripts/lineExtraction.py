#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge, CvBridgeError
import time
import math as m
import pcl
from sensor_msgs import point_cloud2
from ZW_ScanMatching.msg import plane, Coordinate


bridge = CvBridge()

class ShelfDetection():
    def __init__(self):
        self.points_pub = rospy.Publisher('/transformed_pts', PointCloud2, queue_size=10)
        self.L_plane_pub = rospy.Publisher('/L_points', plane, queue_size = 10)
        self.R_plane_pub = rospy.Publisher('/R_points', plane, queue_size = 10)

        self.camera_factor = rospy.get_param('~camera_factor', 1)
        self.camera_cx = rospy.get_param('~camera_cx', 640)
        self.camera_cy = rospy.get_param('~camera_cy', 360)
        self.camera_fx = rospy.get_param('~camera_fx', 1466)
        self.camera_fy = rospy.get_param('~camera_fy', 1130)
        
        
        self.L_pt_x = []
        self.L_pt_y = []
        self.R_pt_x = []
        self.R_pt_y = []

        self.R_points = np.zeros((4,2))
        self.L_points = np.zeros((4,2))

        self.point = []
        self.depth_img = 0

        self.count = 0
        
        self.L_plane = plane()
        self.L_plane.msg_seq = 1
        self.R_plane = plane()
        self.R_plane.msg_seq = 1

    def depth_callback(self, data):
        try:
            self.depth_img = bridge.imgmsg_to_cv2(data,"32FC1")
        except CvBridgeError as e:
            print(e)

        
        # Iterate through the depth image and calculate 3D points
        # for v in range(720):
        #     for u in range(1280):
        #         d = self.depth_img[v,u]
        #         if d==0:
        #             continue

        #         z = d * self.camera_factor
        #         x = (u - self.camera_cx) * z / self.camera_fx
        #         y = (v - self.camera_cy) * z / self.camera_fy

        #         self.point.append([x, y, z])

        # point_cloud2_msg = self.numpy_to_pointcloud2(np.array(self.point))
        # self.point.clear()
        # self.points_pub.publish(point_cloud2_msg)

    # Convert the Numpy array of points to a PointCloud2 message
    def numpy_to_pointcloud2(self, points):
        header = rospy.Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "Camera"

        point_cloud2_msg = point_cloud2.create_cloud_xyz32(header, points)

        return point_cloud2_msg

    def cvt_uv_to_xyz(self, points):

        Plane = plane()
        xyz_points=[]
        for point in points:
            
            d = self.depth_img[point[1], point[0]]
            if d==0:
                continue

            z = d * self.camera_factor
            x = (point[0] - self.camera_cx) * z / self.camera_fx
            y = (point[1] - self.camera_cy) * z / self.camera_fy

            Plane.plane.append(Coordinate(x=z, y=-x, z=y))
            xyz_points.append([x,y,z])
        # print(xyz_points)
        
        return Plane
    def img_callback(self,data):
        # Convert image from ROS format to OpenCV format
        # print(self.count)
        if self.count == 1:
            self.L_pt_x.clear()
            self.L_pt_y.clear()
            self.R_pt_x.clear()
            self.R_pt_y.clear()
            self.R_points=np.zeros((4,2))
            self.L_points=np.zeros((4,2))
            
        

        try:
            self.cv2_img = bridge.imgmsg_to_cv2(data,"bgr8")
        except CvBridgeError as e:
            print(e)
        
        # Preprocess the image
        masked_image = self.RGB_color_selection(self.cv2_img)
        gray_img = cv2.cvtColor(masked_image, cv2.COLOR_RGB2GRAY)
        blur_img = cv2.GaussianBlur(gray_img, (5,5), 0)
        edge_detection_img = cv2.Canny(blur_img,50, 150)


        # Apply Hough transform to detect lines
        lines = self.hough_transform(edge_detection_img)
        
        if lines is not None: # If line information has been got
            for i in range(lines.shape[0]):
                angle = abs(m.atan2((lines[i][0][3] - lines[i][0][1]),(lines[i][0][2] - lines[i][0][0])))
                point = self.cvt_uv_to_xyz(lines[i])
                # Check if line is not horizontal and not vertical
                if (angle > 0.1) and (angle < 1.5):

                    # Check if line is on the left side of image
                    if lines[i][0][0] < int(self.cv2_img.shape[1]/2):
                        if abs(point.plane[0].x) < 5:
                            if len(self.L_pt_x)==0:
                                self.L_pt_x.append((lines[i][0][0], lines[i][0][2]))
                                self.L_pt_y.append((lines[i][0][1], lines[i][0][3]))

                            else:
                                is_duplicate = False                    
                                
                                for j in range(len(self.L_pt_x)):

                                    a, b = self.cal_ab(self.L_pt_x[j], self.L_pt_y[j]) #slope and
                                    if abs(angle-abs(m.atan2((self.L_pt_y[j][1] - self.L_pt_y[j][0]),(self.L_pt_x[j][1] - self.L_pt_x[j][0]))))<0.1:

                                        if abs(lines[i][0][1] - (a*lines[i][0][0]+b))<40:
                                        # if (abs(self.L_pt_x[j][0] - lines[i][0][0]) < 500) and (abs(self.L_pt_y[j][0]-lines[i][0][1]) < 120):
                                            is_duplicate = True
                                            break
                                
                                if not is_duplicate:
                                    
                                    self.L_pt_x.append((lines[i][0][0], lines[i][0][2]))
                                    self.L_pt_y.append((lines[i][0][1], lines[i][0][3]))

                    else:
                        # print(point.plane[0].x)
                        if abs(point.plane[0].x) < 10:
                            if len(self.R_pt_x)==0:
                                self.R_pt_x.append((lines[i][0][0], lines[i][0][2]))
                                self.R_pt_y.append((lines[i][0][1], lines[i][0][3]))

                            else:
                                is_duplicate = False                    
                                
                                for j in range(len(self.R_pt_x)):
                                    a, b = self.cal_ab(self.R_pt_x[j], self.R_pt_y[j])
                                    if abs(angle-abs(m.atan2((self.R_pt_y[j][1] - self.R_pt_y[j][0]),(self.R_pt_x[j][1] - self.R_pt_x[j][0]))))<0.1:
                                        if abs(lines[i][0][1] - (a*lines[i][0][0]+b))<40:
                                            is_duplicate = True
                                            break
                                
                                if not is_duplicate:
                                    
                                    self.R_pt_x.append((lines[i][0][0], lines[i][0][2]))
                                    self.R_pt_y.append((lines[i][0][1], lines[i][0][3]))
            
            ######
            # for i in range(len(self.L_pt_x)):
            #     a, b = self.cal_ab(self.L_pt_x[i], self.L_pt_y[i])
            #     cv2.line(self.cv2_img, (0, int(b)), (int(self.cv2_img.shape[1]/3), int(self.cv2_img.shape[1]/3*a+b)), (0, 0, 255), 2, cv2.LINE_AA)
                
            # for j in range(len(self.R_pt_x)):
            #     a, b = self.cal_ab(self.R_pt_x[j], self.R_pt_y[j])
            #     cv2.line(self.cv2_img, (int(self.cv2_img.shape[1]/3*2), int(self.cv2_img.shape[1]/3*2*a+b)),(int(self.cv2_img.shape[1]), int(self.cv2_img.shape[1]*a+b)), (0, 0, 255), 2, cv2.LINE_AA)
    
            ######
            
            if len(self.L_pt_y) > 1:
                L_yellow_index = self.L_pt_y.index(max(self.L_pt_y))
                L_top_index = self.L_pt_y.index(min(self.L_pt_y))
                l_a, l_b = self.cal_ab(self.L_pt_x[L_yellow_index], self.L_pt_y[L_yellow_index])
                self.L_points = np.array([[self.L_pt_x[L_top_index][0], self.L_pt_y[L_top_index][0]], 
                                        [self.L_pt_x[L_top_index][1], self.L_pt_y[L_top_index][1]], 
                                        [self.L_pt_x[L_top_index][1], int(self.L_pt_x[L_top_index][1]*l_a+l_b)],
                                        [self.L_pt_x[L_top_index][0], int(self.L_pt_x[L_top_index][0]*l_a+l_b)]], np.int32)
            else:
                self.L_points = np.zeros((4,2))
            
            R_yellow_index = self.R_pt_y.index(max(self.R_pt_y))
            R_top_index = self.R_pt_y.index(min(self.R_pt_y))
            r_a, r_b = self.cal_ab(self.R_pt_x[R_yellow_index], self.R_pt_y[R_yellow_index])
            
            self.R_points = np.array([[self.R_pt_x[R_top_index][0], self.R_pt_y[R_top_index][0]], 
                                    [self.R_pt_x[R_top_index][1], self.R_pt_y[R_top_index][1]], 
                                    [self.R_pt_x[R_top_index][1], int(self.R_pt_x[R_top_index][1]*r_a+r_b)],
                                    [self.R_pt_x[R_top_index][0], int(self.R_pt_x[R_top_index][0]*r_a+r_b)]], np.int32)

            if not self.depth_img.all()==0:
                self.L_plane = self.cvt_uv_to_xyz(self.L_points)
                self.R_plane = self.cvt_uv_to_xyz(self.R_points)
                self.L_plane_pub.publish(self.L_plane)
                self.R_plane_pub.publish(self.R_plane)

                

            
            # cv2.fillPoly(self.cv2_img, [self.L_points], color=(255, 0, 0, 1.0))
            # cv2.fillPoly(self.cv2_img, [self.R_points], color=(255, 0, 0, 1.0))
        if self.count == 0:
            self.count +=1
        
        elif self.count == 1:
            # self.img_show(self.cv2_img)
            self.count = 0
        

        # cv2.imshow('img',edge_detection_img)
        # cv2.imshow('img1', sobel)
        # cv2.imshow('img2',laplacian)
        # cv2.waitKey(0)
        # cv2.destroyAllWindows()
    
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

    def img_show(self, image):
        cv2.imshow('img',image)
        cv2.waitKey(800)
        cv2.destroyAllWindows()


def main():
    rospy.init_node('Shelf_detection')
    shelf = ShelfDetection()
    rospy.Subscriber('/rgb', Image, shelf.img_callback)
    rospy.Subscriber('/depth_img', Image, shelf.depth_callback)
    rospy.spin()

   
if __name__ == "__main__":
    main()
    

