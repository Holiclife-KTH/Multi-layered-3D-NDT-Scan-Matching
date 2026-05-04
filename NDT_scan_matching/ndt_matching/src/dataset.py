#!/usr/bin/env python3

import rospy
import numpy as np
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped
from autoware_msgs.msg import NDTStat
from tf2_msgs.msg import TFMessage
from std_msgs.msg import Empty
import openpyxl
import csv
import time
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math as m

class Dataset:
    def __init__(self, data=False):
        self.ref_position = np.zeros((2,1))
        self.fusion_position = np.zeros((2,1))
        self.data_position = np.zeros(3)
        self.ref_lidar_position = np.zeros((2,1))
        self.ref_chassis_position = np.zeros((2,1))
        self.pose = Pose()
        self.position_ = np.zeros((2,1))
        self.event = 0
        self.prev_time = 0.0
        self.ref_yaw = 0.0
        self.est_yaw = 0.0
        self.score_low = 0.0
        self.score_mid = 0.0
        self.score_high = 0.0
        self.event = 0

    
       
    def callback_tf_world(self,data):
        self.ref_chassis_position[0,0] = data.transforms[0].transform.translation.x
        self.ref_chassis_position[1,0] = data.transforms[0].transform.translation.y
        orientation=[data.transforms[0].transform.rotation.x,data.transforms[0].transform.rotation.y,data.transforms[0].transform.rotation.z,data.transforms[0].transform.rotation.w]
        (_,_,self.ref_yaw)= euler_from_quaternion(orientation)

    def callback_tf_chassis(self,data):
        self.ref_lidar_position[0,0] = data.transforms[0].transform.translation.x 
        self.ref_lidar_position[1,0] = data.transforms[0].transform.translation.y

    def callback_fusion_position(self, data):
        self.fusion_position[0,0] = data.pose.position.x
        self.fusion_position[1,0] = data.pose.position.y
        orientation = [data.pose.orientation.x,data.pose.orientation.y,data.pose.orientation.z,data.pose.orientation.w]
       
        (_,_,self.est_yaw)= euler_from_quaternion(orientation)


    def callback_position(self,data):
        self.position_[0,0] = data.pose.pose.position.x + 0.25
        self.position_[1,0] = data.pose.pose.position.y
        # print(self.position_)
        orientation = [data.pose.pose.orientation.x,data.pose.pose.orientation.y,data.pose.pose.orientation.z,data.pose.pose.orientation.w]
        # (_,_,self.est_yaw)= euler_from_quaternion(orientation)

    def callback_initial(self, data):
        self.fusion_position[0,0] = data.pose.pose.position.x
        self.fusion_position[1,0] = data.pose.pose.position.y
        orientation = [data.pose.pose.orientation.x,data.pose.pose.orientation.y,data.pose.pose.orientation.z,data.pose.pose.orientation.w]
        (_,_,self.est_yaw)= euler_from_quaternion(orientation)

    def callback_score_low(self, data):
        self.score_low = data.score

    
    def callback_score_mid(self, data):
        self.score_mid = data.score

    
    def callback_score_high(self, data):
        self.score_high = data.score

    def callback_event(self,data):
        self.event = self.event + 1
        

    def dataset(self):
       if self.position_[0,0] != 0:
            self.ref_position = self.ref_chassis_position - self.ref_lidar_position        
            # self.data_position[0] = ((self.ref_position[0,0] - self.fusion_position[0,0])**2 + (self.ref_position[1,0] - self.fusion_position[1,0])**2)**0.5 * 1000 #distance Error
            self.data_position[0] = ((self.ref_position[0,0] - self.position_[0,0])**2 + (self.ref_position[1,0] - self.position_[1,0])**2)**0.5 * 1000
            self.data_position[1] = m.degrees(self.ref_yaw-self.est_yaw) #heading error
            # self.data_position[3] = self.score_mid
            # self.data_position[4] = self.score_high
            # self.data_position[5] = self.event
            self.data_position[2] = float(rospy.Time.now().to_sec()) - self.prev_time
           
            print("Position error: ",self.data_position[0])
            print("Yaw_error: ", self.ref_yaw-self.est_yaw)
            print("Reference: ", self.ref_position)
            self.event = 0
            # if not self.data_position[0] == 0:

            #     f= open('/home/irol/ZW_ws/data/230517/normal.csv', 'a', newline='')
            #     wr=csv.writer(f)
            #     wr.writerow(self.data_position)
            #     f.close()

    def callback(self, data):
        self.prev_time = float(rospy.Time.now().to_sec())
        r = rospy.Rate(3)
        while not rospy.is_shutdown():
            self.dataset()
            r.sleep()

            



def main():
    dataset = Dataset()
    rospy.Subscriber("/fusion_position", PoseStamped, dataset.callback_fusion_position)
    rospy.Subscriber("/localizer_pose", PoseWithCovarianceStamped, dataset.callback_position)
    rospy.Subscriber("/initialpose", PoseWithCovarianceStamped,dataset.callback_initial )
    rospy.Subscriber("/ndt_stat",NDTStat,dataset.callback_score_low)
    # rospy.Subscriber("/ndt_stat2",NDTStat,dataset.callback_score_mid)
    # rospy.Subscriber("/ndt_stat3",NDTStat,dataset.callback_score_high)
    rospy.Subscriber("/tf2", TFMessage, dataset.callback_tf_world)
    rospy.Subscriber("/tf3", TFMessage,dataset.callback_tf_chassis)
    rospy.Subscriber("/trans", Empty, dataset.callback)
    rospy.Subscriber("/trans2", Empty, dataset.callback_event)


    
        
    rospy.spin()
if __name__== '__main__':
    rospy.init_node('dataset', anonymous=True)
    main()
