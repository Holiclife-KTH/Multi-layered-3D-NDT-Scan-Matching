#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import time
import math as m
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from std_msgs.msg import Empty, Float32
import openpyxl
import csv
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import tf




class Dataset:
    def __init__(self, data=False):
        self.ref_position = np.zeros((3,1))
        self.fusion_position = np.zeros((6,1))
        self.data_position = np.zeros(7)
        self.normal_data = np.zeros(4)
        self.static_layered_data = np.zeros(9)
        self.ref_chassis_position = np.zeros((6,1))
        self.bottom_est_position = np.zeros((6,1))
        self.top_est_position = np.zeros((6,1))
        self.middle_est_position = np.zeros((6,1))
        
        self.Est_position = np.zeros((6,1))
        self.event = 0
        self.prev_time = 0.0
        self.Ref_yaw = 0.0
        self.Est_yaw = 0.0
        self.event = 0
        self.start = False
        
        self.listener = tf.TransformListener()
        self.broadcaster = tf.TransformBroadcaster()

        self.fusion_pose = PoseStamped()
        self.top_pose = PoseStamped()
        self.middle_pose = PoseStamped()
        self.bottom_pose = PoseStamped()
        self.normal_pose = PoseStamped()

        self.exe_time = 0.0
        self.top_exe_time = 0.0
        self.bottom_exe_time = 0.0

        self.score = 0.0
        self.top_score = 0.0
        self.bottom_score = 0.0
        self.middle_score = 0.0

                   
    def callback_tf_world(self,data):
        self.ref_chassis_position[0,0] = data.transforms[0].transform.translation.x
        self.ref_chassis_position[1,0] = data.transforms[0].transform.translation.y
        self.ref_chassis_position[2,0] = data.transforms[0].transform.translation.z
        orientation=[data.transforms[0].transform.rotation.x,data.transforms[0].transform.rotation.y,data.transforms[0].transform.rotation.z,data.transforms[0].transform.rotation.w]
        (self.ref_chassis_position[3,0],self.ref_chassis_position[4,0],self.ref_chassis_position[5,0])= euler_from_quaternion(orientation)


    def callback_fusion_position(self, data):
        # self.chassis2lidar = self.listener.lookupTransform(target_frame="chassis_link", source_frame="VLP16", time=rospy.Time(0))
        
        self.fusion_pose.header.frame_id = "map"
        self.fusion_pose.pose.position.x = data.pose.position.x
        self.fusion_pose.pose.position.y = data.pose.position.y 
        self.fusion_pose.pose.position.z = data.pose.position.z

        self.fusion_pose.pose.orientation.x = data.pose.orientation.x
        self.fusion_pose.pose.orientation.y = data.pose.orientation.y
        self.fusion_pose.pose.orientation.z = data.pose.orientation.z
        self.fusion_pose.pose.orientation.w = data.pose.orientation.w

        res = self.listener.transformPose(ps=self.fusion_pose, target_frame="world")

        self.fusion_position[0,0] = res.pose.position.x
        self.fusion_position[1,0] = res.pose.position.y
        self.fusion_position[2,0] = res.pose.position.z
        orientation = [res.pose.orientation.x,res.pose.orientation.y,res.pose.orientation.z,res.pose.orientation.w]
       
        (self.fusion_position[3,0],self.fusion_position[4,0],self.fusion_position[5,0])= euler_from_quaternion(orientation)

        self.start = True


    def callback_position(self,data):    
        self.normal_pose.header.frame_id = "map"
        self.normal_pose.pose.position.x = data.pose.pose.position.x 
        self.normal_pose.pose.position.y = data.pose.pose.position.y 
        self.normal_pose.pose.position.z = data.pose.pose.position.z 

        self.normal_pose.pose.orientation.x = data.pose.pose.orientation.x
        self.normal_pose.pose.orientation.y = data.pose.pose.orientation.y
        self.normal_pose.pose.orientation.z = data.pose.pose.orientation.z
        self.normal_pose.pose.orientation.w = data.pose.pose.orientation.w

        res = self.listener.transformPose(ps=self.normal_pose, target_frame="world")
        

        self.Est_position[0,0] = res.pose.position.x
        self.Est_position[1,0] = res.pose.position.y
        self.Est_position[2,0] = res.pose.position.z

        orientation = [res.pose.orientation.x,res.pose.orientation.y,res.pose.orientation.z,res.pose.orientation.w]
        (self.Est_position[3,0],self.Est_position[4,0],self.Est_position[5,0])= euler_from_quaternion(orientation)
        self.start = True

    def callback_top_position(self,data):      
        self.top_pose.header.frame_id = "map"
        self.top_pose.pose.position.x = data.pose.pose.position.x 
        self.top_pose.pose.position.y = data.pose.pose.position.y 
        self.top_pose.pose.position.z = data.pose.pose.position.z 

        self.top_pose.pose.orientation.x = data.pose.pose.orientation.x
        self.top_pose.pose.orientation.y = data.pose.pose.orientation.y
        self.top_pose.pose.orientation.z = data.pose.pose.orientation.z
        self.top_pose.pose.orientation.w = data.pose.pose.orientation.w

        res = self.listener.transformPose(ps=self.top_pose, target_frame="world")
        

        self.top_est_position[0,0] = res.pose.position.x
        self.top_est_position[1,0] = res.pose.position.y
        self.top_est_position[2,0] = res.pose.position.z

        

        orientation = [res.pose.orientation.x,res.pose.orientation.y,res.pose.orientation.z,res.pose.orientation.w]
        (self.top_est_position[3,0],self.top_est_position[4,0],self.top_est_position[5,0])= euler_from_quaternion(orientation)

    def callback_middle_position(self,data):      
        self.middle_pose.header.frame_id = "map"
        self.middle_pose.pose.position.x = data.pose.pose.position.x 
        self.middle_pose.pose.position.y = data.pose.pose.position.y 
        self.middle_pose.pose.position.z = data.pose.pose.position.z 

        self.middle_pose.pose.orientation.x = data.pose.pose.orientation.x
        self.middle_pose.pose.orientation.y = data.pose.pose.orientation.y
        self.middle_pose.pose.orientation.z = data.pose.pose.orientation.z
        self.middle_pose.pose.orientation.w = data.pose.pose.orientation.w

        res = self.listener.transformPose(ps=self.middle_pose, target_frame="world")
        

        self.middle_est_position[0,0] = res.pose.position.x
        self.middle_est_position[1,0] = res.pose.position.y
        self.middle_est_position[2,0] = res.pose.position.z

        

        orientation = [res.pose.orientation.x,res.pose.orientation.y,res.pose.orientation.z,res.pose.orientation.w]
        (self.middle_est_position[3,0],self.middle_est_position[4,0],self.middle_est_position[5,0])= euler_from_quaternion(orientation)

    def callback_bottom_position(self,data):       
        self.bottom_pose.header.frame_id = "map"
        self.bottom_pose.pose.position.x = data.pose.pose.position.x 
        self.bottom_pose.pose.position.y = data.pose.pose.position.y 
        self.bottom_pose.pose.position.z = data.pose.pose.position.z 

        self.bottom_pose.pose.orientation.x = data.pose.pose.orientation.x
        self.bottom_pose.pose.orientation.y = data.pose.pose.orientation.y
        self.bottom_pose.pose.orientation.z = data.pose.pose.orientation.z
        self.bottom_pose.pose.orientation.w = data.pose.pose.orientation.w

        res = self.listener.transformPose(ps=self.bottom_pose, target_frame="world")
        

        self.bottom_est_position[0,0] = res.pose.position.x
        self.bottom_est_position[1,0] = res.pose.position.y
        self.bottom_est_position[2,0] = res.pose.position.z

        orientation = [res.pose.orientation.x,res.pose.orientation.y,res.pose.orientation.z,res.pose.orientation.w]
        (self.bottom_est_position[3,0],self.bottom_est_position[4,0],self.bottom_est_position[5,0])= euler_from_quaternion(orientation)


    def callback_initial(self, data):
        self.Est_position[0,0] = data.pose.pose.position.x
        self.Est_position[1,0] = data.pose.pose.position.y
        self.Est_position[2,0] = data.pose.pose.position.z
        orientation = [data.pose.pose.orientation.x,data.pose.pose.orientation.y,data.pose.pose.orientation.z,data.pose.pose.orientation.w]
        (self.Est_position[3,0],self.Est_position[4,0],self.Est_position[5,0])= euler_from_quaternion(orientation)

    def callback_event(self,data):
        self.event = self.event + 1000

    def exe_time_callback(self, msg):
        self.exe_time = msg.data

    def top_exe_time_callback(self, msg):
        self.top_exe_time = msg.data

    def bottom_exe_time_callback(self, msg):
        self.bottom_exe_time = msg.data

    def score_callback(self, msg):
        self.score = msg.data

    def top_score_callback(self, msg):
        self.top_score = msg.data

    def bottom_score_callback(self, msg):
        self.bottom_score = msg.data
        
    def middle_score_callback(self, msg):
        self.middle_score = msg.data
        

    def dataset(self):
       
       if self.start:   
            # self.data_position[0] = float(rospy.Time.now().to_sec()) - self.prev_time 
            # self.data_position[1] = ((self.ref_chassis_position[0,0] - self.fusion_position[0,0])**2 + (self.ref_chassis_position[1,0] - self.fusion_position[1,0])**2 )**0.5 * 1000
            # self.data_position[2] = ((self.ref_chassis_position[0,0] - self.top_est_position[0,0])**2 + (self.ref_chassis_position[1,0] - self.top_est_position[1,0])**2 )**0.5 * 1000
            # self.data_position[3] = ((self.ref_chassis_position[0,0] - self.bottom_est_position[0,0])**2 + (self.ref_chassis_position[1,0] - self.bottom_est_position[1,0])**2 )**0.5 * 1000
            # self.data_position[4] = self.top_score
            # self.data_position[5] = self.bottom_score
            # self.data_position[6] = self.event


            # self.static_layered_data[0] = float(rospy.Time.now().to_sec()) - self.prev_time 
            # self.static_layered_data[1] = ((self.ref_chassis_position[0,0] - self.fusion_position[0,0])**2 + (self.ref_chassis_position[1,0] - self.fusion_position[1,0])**2 )**0.5 * 1000
            # self.static_layered_data[2] = ((self.ref_chassis_position[0,0] - self.top_est_position[0,0])**2 + (self.ref_chassis_position[1,0] - self.top_est_position[1,0])**2 )**0.5 * 1000
            # self.static_layered_data[3] = ((self.ref_chassis_position[0,0] - self.middle_est_position[0,0])**2 + (self.ref_chassis_position[1,0] - self.middle_est_position[1,0])**2 )**0.5 * 1000
            # self.static_layered_data[4] = ((self.ref_chassis_position[0,0] - self.bottom_est_position[0,0])**2 + (self.ref_chassis_position[1,0] - self.bottom_est_position[1,0])**2 )**0.5 * 1000
            # self.static_layered_data[5] = self.top_score
            # self.static_layered_data[6] = self.middle_score
            # self.static_layered_data[7] = self.bottom_score
            # self.static_layered_data[8] = self.event

            self.normal_data[0] = float(rospy.Time.now().to_sec()) - self.prev_time
            self.normal_data[1] = ((self.ref_chassis_position[0,0] - self.Est_position[0,0])**2 + (self.ref_chassis_position[1,0] - self.Est_position[1,0])**2 )**0.5 * 1000
            self.normal_data[2] = self.score
            self.normal_data[3] = self.event
            
            self.event = 0

            # print("Distance error: ", self.data_position[1])
            # print("Top Distance error: ", self.data_position[8])
            # print("Bottom Distance error: ", self.data_position[9])
            # print("top_exe_time: ", self.data_position[11])
            # print("bottom_exe_time: ", self.data_position[10])

            # print("Distance error: ", self.normal_data[1])
            # print("exe_time: ", self.normal_data[4])
            # print("score: ", self.score)
            
            # if not self.data_position[0] == 0:
            #     f= open('/home/irol/catkin_ws/data/230714/adaptive.csv', 'a', newline='')
            #     wr=csv.writer(f)
            #     wr.writerow(self.data_position)
            #     f.close()

            # if not self.static_layered_data[0] == 0:
            #     f= open('/home/irol/catkin_ws/data/230714/static_2.csv', 'a', newline='')
            #     wr=csv.writer(f)
            #     wr.writerow(self.static_layered_data)
            #     f.close()

            if not self.normal_data[0] == 0:

                f= open('/home/irol/catkin_ws/data/230714/normal_2.csv', 'a', newline='')
                wr=csv.writer(f)
                wr.writerow(self.normal_data)
                f.close()

    def callback(self, data):
        self.prev_time = float(rospy.Time.now().to_sec())
        r = rospy.Rate(4)
        while not rospy.is_shutdown():
            self.dataset()
            r.sleep()

            



def main():
    dataset = Dataset()
    rospy.Subscriber("/fusion_position", PoseStamped, dataset.callback_fusion_position)
    rospy.Subscriber("/ndt_pose", Odometry, dataset.callback_position)
    rospy.Subscriber("/top_ndt_pose", Odometry, dataset.callback_top_position)
    rospy.Subscriber("/bottom_ndt_pose", Odometry, dataset.callback_bottom_position)
    rospy.Subscriber("/middle_ndt_pose", Odometry, dataset.callback_middle_position)
    rospy.Subscriber("/initialpose", PoseWithCovarianceStamped,dataset.callback_initial)
    rospy.Subscriber("/tf2", TFMessage, dataset.callback_tf_world)
    rospy.Subscriber("/trans", Empty, dataset.callback)
    rospy.Subscriber("/trans2", Empty, dataset.callback_event)
    rospy.Subscriber("exe_time_ms", Float32, dataset.exe_time_callback)
    rospy.Subscriber("top_exe_time_ms", Float32, dataset.top_exe_time_callback)
    rospy.Subscriber("bottom_exe_time_ms", Float32, dataset.bottom_exe_time_callback)
    rospy.Subscriber("transform_probability", Float32, dataset.score_callback)
    rospy.Subscriber("top_transform_probability", Float32, dataset.top_score_callback)
    rospy.Subscriber("bottom_transform_probability", Float32, dataset.bottom_score_callback)
    rospy.Subscriber("middle_transform_probability", Float32, dataset.middle_score_callback)



    
        
    rospy.spin()
if __name__== '__main__':
    rospy.init_node('dataset', anonymous=True)
    main()
