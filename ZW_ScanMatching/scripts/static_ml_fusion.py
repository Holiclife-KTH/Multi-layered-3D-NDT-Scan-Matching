#!/usr/bin/env python3

import rospy
import numpy as np
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped
from std_msgs.msg import Empty, Float32
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import openpyxl
import csv
import time

class Fusion:
    def __init__(self, data=False):

        self.pub = rospy.Publisher("/fusion_position", PoseStamped, queue_size=1)
        self.pose_low = Pose()
        self.pose_mid = Pose()
        self.pose_upper = Pose()
        self.pose_fusion = PoseStamped()
        self.stat_fusion = np.zeros((6,1))
        self.stat_low = np.zeros((6,1))
        self.stat_mid = np.zeros((6,1))
        self.stat_upper = np.zeros((6,1))
        self.cov_low = np.zeros((6,6))
        self.cov_mid = np.zeros((6,6))
        self.cov_upper = np.zeros((6,6))
        self.fusion_yaw = np.zeros(4)
        self.bottom_score = 0.0
        self.middle_score = 0.0
        self.top_score = 0.0


 
       
        
    def callback_low(self, data):
        self.pose_low.position = data.pose.pose.position
        self.stat_low[0,0] = data.pose.pose.position.x
        self.stat_low[1,0] = data.pose.pose.position.y
        self.stat_low[2,0] = data.pose.pose.position.z
        # self.pose_low.orientation = data.pose.pose.orientation
        self.low_orientation = [data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w ]
        (self.stat_low[3,0],self.stat_low[4,0],self.stat_low[5,0]) = euler_from_quaternion(self.low_orientation)

        for i in range(0,6):
            for j in range(0,6):
                num = i*6 + j
                self.cov_low[i,j] = data.pose.covariance[num]

       
        

        
        
    def callback_mid(self, data):
        self.pose_mid.position = data.pose.pose.position
        self.stat_mid[0,0] = data.pose.pose.position.x
        self.stat_mid[1,0] = data.pose.pose.position.y
        self.stat_mid[2,0] = data.pose.pose.position.z
        # self.pose_mid.orientation = data.pose.pose.orientation
        self.mid_orientation = [data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w ]
        (self.stat_mid[3,0],self.stat_mid[4,0],self.stat_mid[5,0]) = euler_from_quaternion(self.mid_orientation)
        for i in range(0,6):
            for j in range(0,6):
                num = i*6 + j
                self.cov_mid[i,j] = data.pose.covariance[num]


    def callback_upper(self, data):
        self.pose_upper.position = data.pose.pose.position
        self.stat_upper[0,0] = data.pose.pose.position.x
        self.stat_upper[1,0] = data.pose.pose.position.y
        self.stat_upper[2,0] = data.pose.pose.position.z
        # self.pose_mid.orientation = data.pose.pose.orientation
        self.upper_orientation = [data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w ]
        (self.stat_upper[3,0],self.stat_upper[4,0],self.stat_upper[5,0]) = euler_from_quaternion(self.upper_orientation)
        for i in range(0,6):
            for j in range(0,6):
                num = i*6 + j
                self.cov_upper[i,j] = data.pose.covariance[num]
        
    def top_score_callback(self, msg):
        self.top_score = msg.data

    def bottom_score_callback(self, msg):
        self.bottom_score = msg.data

    def middle_score_callback(self, msg):
        self.middle_score = msg.data
        
    def callback_signal(self,data):
        self.fusion()


    def fusion(self):
        scores = [self.bottom_score, self.middle_score, self.top_score]
        print(scores)
        try:
            if scores.index(min(scores)) == 0: 
            # self.position_fusion= self.position_mid + np.dot(np.dot(self.cov_mid, np.linalg.inv(self.cov_mid+self.cov_high)),(self.position_high-self.position_mid))
                print("Lower layer will be rejected!")
                self.stat_fusion = self.stat_mid + np.dot(np.dot(self.cov_mid, np.linalg.inv(self.cov_mid + self.cov_upper)), (self.stat_upper - self.stat_mid))
                self.pose_fusion.pose.position.x = self.stat_fusion[0,0]
                self.pose_fusion.pose.position.y = self.stat_fusion[1,0]
                q = quaternion_from_euler(0.0,0.0,self.stat_fusion[3,0])
                self.pose_fusion.pose.orientation.x = q[0]
                self.pose_fusion.pose.orientation.y = q[1]
                self.pose_fusion.pose.orientation.z = q[2]
                self.pose_fusion.pose.orientation.w = q[3]
                self.pose_fusion.header.frame_id = "map"
                self.pose_fusion.header.stamp = rospy.Time.now()

            elif scores.index(min(scores)) == 1:
                print("Middle layer will be rejected!")
                
                self.stat_fusion = self.stat_low + np.dot(np.dot(self.cov_low, np.linalg.inv(self.cov_low + self.cov_upper)), (self.stat_upper - self.stat_low))
                self.pose_fusion.pose.position.x = self.stat_fusion[0,0]
                self.pose_fusion.pose.position.y = self.stat_fusion[1,0]
                q = quaternion_from_euler(0.0,0.0,self.stat_fusion[3,0])
                self.pose_fusion.pose.orientation.x = q[0]
                self.pose_fusion.pose.orientation.y = q[1]
                self.pose_fusion.pose.orientation.z = q[2]
                self.pose_fusion.pose.orientation.w = q[3]
                self.pose_fusion.header.frame_id = "map"
                self.pose_fusion.header.stamp = rospy.Time.now()


            elif scores.index(min(scores)) == 2:
                print("Upper layer will be rejected!")
                self.stat_fusion = self.stat_low + np.dot(np.dot(self.cov_low, np.linalg.inv(self.cov_low + self.cov_mid)), (self.stat_mid - self.stat_low))
                self.pose_fusion.pose.position.x = self.stat_fusion[0,0]
                self.pose_fusion.pose.position.y = self.stat_fusion[1,0]
                q = quaternion_from_euler(0.0,0.0,self.stat_fusion[3,0])
                self.pose_fusion.pose.orientation.x = q[0]
                self.pose_fusion.pose.orientation.y = q[1]
                self.pose_fusion.pose.orientation.z = q[2]
                self.pose_fusion.pose.orientation.w = q[3]
                self.pose_fusion.header.frame_id = "map"
                self.pose_fusion.header.stamp = rospy.Time.now()


            # print(self.pose_fusion.pose)
            
            self.pub.publish(self.pose_fusion)

        except:
            print("error")


def main():
    fusion = Fusion()
    rospy.Subscriber("/bottom_ndt_pose", Odometry, fusion.callback_low)
    rospy.Subscriber("/top_ndt_pose", Odometry, fusion.callback_upper)
    rospy.Subscriber("/middle_ndt_pose", Odometry, fusion.callback_mid)
    rospy.Subscriber("/signal", Empty, fusion.callback_signal)
    rospy.Subscriber("top_transform_probability", Float32, fusion.top_score_callback)
    rospy.Subscriber("bottom_transform_probability", Float32, fusion.bottom_score_callback)
    rospy.Subscriber("middle_transform_probability", Float32, fusion.middle_score_callback)

    time.sleep(1)
    while not rospy.is_shutdown():
        fusion.fusion()

    rospy.spin()

    # rospy.spin()
if __name__== '__main__':
    rospy.init_node('stat_fusion', anonymous=True)
    main()