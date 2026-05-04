#!/usr/bin/env python3

import rospy
import numpy as np
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped
from autoware_msgs.msg import NDTStat
from tf2_msgs.msg import TFMessage
import openpyxl
import csv
import time




def main():
    
    pub = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=1 )
    time.sleep(1)
    Pose = PoseWithCovarianceStamped()
    Pose.pose.pose.position.x = -8.1
    Pose.pose.pose.position.y = 6.9
    Pose.pose.pose.position.z = 0.0
    Pose.pose.pose.orientation.x = 0.0005
    Pose.pose.pose.orientation.y = -0.0005
    Pose.pose.pose.orientation.z = 0.707
    Pose.pose.pose.orientation.w = 0.707
    Pose.header.frame_id="map"
    pub.publish(Pose)

    print("done")





    # rospy.spin()
if __name__== '__main__':
    rospy.init_node('initial_pose', anonymous=True)
    
    main()
