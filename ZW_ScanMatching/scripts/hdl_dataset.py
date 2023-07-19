#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import time
import math as m
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped
from hdl_localization.msg import *
from tf2_msgs.msg import TFMessage
from std_msgs.msg import Empty
import openpyxl
import csv
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import tf




class Dataset:
    def __init__(self, data=False):
        self.ref_position = np.zeros((2,1))
        self.fusion_position = np.zeros((2,1))
        self.data_position = np.zeros(4)
        self.ref_lidar_position = np.zeros((2,1))
        self.ref_chassis_position = np.zeros((2,1))
        
        self.Est_position_ = np.zeros((3,1))
        self.event = 0
        self.prev_time = 0.0
        self.Ref_yaw = 0.0
        self.Est_yaw = 0.0
        self.score_low = 0.0
        self.score_mid = 0.0
        self.score_high = 0.0
        self.event = 0
        
        self.listener = tf.TransformListener()
        self.broadcaster = tf.TransformBroadcaster()

        self.pose = PoseStamped() 

                   
    def callback_tf_world(self,data):
        self.ref_chassis_position[0,0] = data.transforms[0].transform.translation.x
        self.ref_chassis_position[1,0] = data.transforms[0].transform.translation.y
        orientation=[data.transforms[0].transform.rotation.x,data.transforms[0].transform.rotation.y,data.transforms[0].transform.rotation.z,data.transforms[0].transform.rotation.w]
        (_,_,self.Ref_yaw)= euler_from_quaternion(orientation)


    def callback_fusion_position(self, data):
        self.fusion_position[0,0] = data.pose.position.x
        self.fusion_position[1,0] = data.pose.position.y
        orientation = [data.pose.orientation.x,data.pose.orientation.y,data.pose.orientation.z,data.pose.orientation.w]
       
        # (_,_,self.est_yaw)= euler_from_quaternion(orientation)


    def callback_position(self,data):
        self.chassis2lidar = self.listener.lookupTransform(target_frame="chassis_link", source_frame="VLP16", time=rospy.Time(0))
        
        self.pose.header.frame_id = "map"
        self.pose.pose.position.x = data.relative_pose.translation.x + self.chassis2lidar[0][0]
        self.pose.pose.position.y = data.relative_pose.translation.y + self.chassis2lidar[0][1]
        self.pose.pose.position.z = data.relative_pose.translation.z + self.chassis2lidar[0][2]

        self.pose.pose.orientation.x = data.relative_pose.rotation.x
        self.pose.pose.orientation.y = data.relative_pose.rotation.y
        self.pose.pose.orientation.z = data.relative_pose.rotation.z
        self.pose.pose.orientation.w = data.relative_pose.rotation.w

       

        res = self.listener.transformPose(ps=self.pose, target_frame="world")
        

        self.Est_position_ [0,0] = res.pose.position.x
        self.Est_position_ [1,0] = res.pose.position.y

        orientation = [res.pose.orientation.x,res.pose.orientation.y,res.pose.orientation.z,res.pose.orientation.w]
        (_,_,self.Est_yaw)= euler_from_quaternion(orientation)


    def callback_initial(self, data):
        self.Est_position_[0,0] = data.pose.pose.position.x
        self.Est_position_[1,0] = data.pose.pose.position.y
        orientation = [data.pose.pose.orientation.x,data.pose.pose.orientation.y,data.pose.pose.orientation.z,data.pose.pose.orientation.w]
        (_,_,self.Est_yaw)= euler_from_quaternion(orientation)

    def callback_score_low(self, data):
        self.score_low = data.score

    
    def callback_score_mid(self, data):
        self.score_mid = data.score

    
    def callback_score_high(self, data):
        self.score_high = data.score

    def callback_event(self,data):
        self.event = self.event + 1000
        

    def dataset(self):
       if self.Est_position_[0,0] != 0:
            self.ref_position = self.ref_chassis_position     
            # self.data_position[0] = ((self.ref_position[0,0] - self.fusion_position[0,0])**2 + (self.ref_position[1,0] - self.fusion_position[1,0])**2)**0.5 * 1000 #distance Error
            self.data_position[0] = ((self.ref_position[0,0] - self.Est_position_[0,0])**2 + (self.ref_position[1,0] - self.Est_position_[1,0])**2)**0.5 * 1000
            self.data_position[1] = m.degrees(self.Ref_yaw-self.Est_yaw) #heading error
            self.data_position[2] = float(rospy.Time.now().to_sec()) - self.prev_time
            self.data_position[3] = self.event
            print("Position error: ",self.data_position[0])
            print("Yaw_error: ", self.data_position[1])
            print("Reference: ", self.ref_position)
            print("Estimation: ", self.Est_position_)
            # print("Event: ", self.data_position[3])
            self.event = 0
            # if not self.data_position[0] == 0:

            #     f= open('/home/haneul/catkin_ws/data/230529/normal_0104.csv', 'a', newline='')
            #     wr=csv.writer(f)
            #     wr.writerow(self.data_position)
            #     f.close()

    def callback(self, data):
        self.prev_time = float(rospy.Time.now().to_sec())
        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            self.dataset()
            r.sleep()

            



def main():
    dataset = Dataset()
    rospy.Subscriber("/fusion_position", PoseStamped, dataset.callback_fusion_position)
    rospy.Subscriber("/ndt_pose", ScanMatchingStatus, dataset.callback_position)
    rospy.Subscriber("/initialpose", PoseWithCovarianceStamped,dataset.callback_initial)
    rospy.Subscriber("/tf2", TFMessage, dataset.callback_tf_world)
    rospy.Subscriber("/trans", Empty, dataset.callback)
    rospy.Subscriber("/trans2", Empty, dataset.callback_event)


    
        
    rospy.spin()
if __name__== '__main__':
    rospy.init_node('dataset', anonymous=True)
    main()
