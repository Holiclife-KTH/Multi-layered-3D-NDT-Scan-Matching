#!/usr/bin/env python3

import rospy
import numpy as np
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped
from std_msgs.msg import Empty, Float32
from autoware_msgs.msg import NDTStat
from tf2_msgs.msg import TFMessage
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import openpyxl
import csv
import time

class Fusion:
    def __init__(self, data=False):

        self.pub = rospy.Publisher("/fusion_position", PoseStamped, queue_size=1)
        self.pose_low = Pose()
        self.pose_mid = Pose()
        self.pose_fusion = PoseStamped()
        self.stat_fusion = np.zeros((4,1))
        self.stat_low = np.zeros((4,1))
        self.stat_mid = np.zeros((4,1))
        self.cov_low = np.zeros((3,1))
        self.cov_mid = np.zeros((3,1))
        self.cov_low = np.zeros((4,4))
        self.cov_mid = np.zeros((4,4))
        self.fusion_yaw = np.zeros(4)
        self.score_low = 0.0
        self.score_mid = 0.0


 
       
        
    def callback_low(self, data):
        self.pose_low.position = data.pose.pose.position
        self.stat_low[0,0] = data.pose.pose.position.x
        self.stat_low[1,0] = data.pose.pose.position.y
        self.stat_low[2,0] = data.pose.pose.position.z
        # self.pose_low.orientation = data.pose.pose.orientation
        self.low_orientation = [data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w ]
        (_,_,self.stat_low[3,0]) = euler_from_quaternion(self.low_orientation)

        self.cov_low[0,0] = data.pose.covariance[0]
        self.cov_low[0,1] = data.pose.covariance[1]
        self.cov_low[0,2] = data.pose.covariance[2]
        self.cov_low[0,3] = data.pose.covariance[5]
        self.cov_low[1,0] = data.pose.covariance[6]
        self.cov_low[1,1] = data.pose.covariance[7]
        self.cov_low[1,2] = data.pose.covariance[8]
        self.cov_low[1,3] = data.pose.covariance[11]
        self.cov_low[2,0] = data.pose.covariance[12]
        self.cov_low[2,1] = data.pose.covariance[13]
        self.cov_low[2,2] = data.pose.covariance[14]
        self.cov_low[2,3] = data.pose.covariance[17]
        self.cov_low[3,0] = data.pose.covariance[30]
        self.cov_low[3,1] = data.pose.covariance[31]
        self.cov_low[3,2] = data.pose.covariance[32]
        self.cov_low[3,3] = data.pose.covariance[35]
        

        
        
    def callback_mid(self, data):
        self.pose_mid.position = data.pose.pose.position
        self.stat_mid[0,0] = data.pose.pose.position.x
        self.stat_mid[1,0] = data.pose.pose.position.y
        self.stat_mid[2,0] = data.pose.pose.position.z
        # self.pose_mid.orientation = data.pose.pose.orientation
        self.mid_orientation = [data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w ]
        (_,_,self.stat_mid[3,0]) = euler_from_quaternion(self.mid_orientation)
        self.cov_mid[0,0] = data.pose.covariance[0]
        self.cov_mid[0,1] = data.pose.covariance[1]
        self.cov_mid[0,2] = data.pose.covariance[2]
        self.cov_mid[0,3] = data.pose.covariance[5]
        self.cov_mid[1,0] = data.pose.covariance[6]
        self.cov_mid[1,1] = data.pose.covariance[7]
        self.cov_mid[1,2] = data.pose.covariance[8]
        self.cov_mid[1,3] = data.pose.covariance[11]
        self.cov_mid[2,0] = data.pose.covariance[12]
        self.cov_mid[2,1] = data.pose.covariance[13]
        self.cov_mid[2,2] = data.pose.covariance[14]
        self.cov_mid[2,3] = data.pose.covariance[17]
        self.cov_mid[3,0] = data.pose.covariance[30]
        self.cov_mid[3,1] = data.pose.covariance[31]
        self.cov_mid[3,2] = data.pose.covariance[32]
        self.cov_mid[3,3] = data.pose.covariance[35]

        # print(self.stat_mid)

        
    def callback_signal(self,data):
        self.fusion()


    def fusion(self):
        scores = [self.score_low, self.score_mid]
        # print(scores)
        try:
            
            # self.position_fusion= self.position_mid + np.dot(np.dot(self.cov_mid, np.linalg.inv(self.cov_mid+self.cov_high)),(self.position_high-self.position_mid))
            
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

            print(self.pose_fusion)
            
            self.pub.publish(self.pose_fusion)

        except:
            print("error")

        



def main():
    fusion = Fusion()
    rospy.Subscriber("/bottom_localizer_pose", PoseWithCovarianceStamped, fusion.callback_low)
    rospy.Subscriber("/top_localizer_pose", PoseWithCovarianceStamped, fusion.callback_mid)
    rospy.Subscriber("/signal", Empty, fusion.callback_signal)

    time.sleep(1)
    while not rospy.is_shutdown():
        fusion.fusion()

    rospy.spin()

    # rospy.spin()
if __name__== '__main__':
    rospy.init_node('stat_fusion', anonymous=True)
    main()
