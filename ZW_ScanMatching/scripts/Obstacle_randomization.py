#!/usr/bin/env python3

import rospy
import numpy as np
from ZW_ScanMatching.srv import ObstacleLayout
from ZW_ScanMatching.srv import ObstacleLayoutRequest
from std_msgs.msg import Empty
import time
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped, Twist
from autoware_msgs.msg import NDTStat
from tf2_msgs.msg import TFMessage
import openpyxl
import csv
import time
import pandas as pd

def teleport_client(msg):
    rospy.wait_for_service("teleport")
    try:
        teleport = rospy.ServiceProxy("teleport", ObstacleLayout)
        teleport(msg)
        return
    except rospy.ServiceException as e:
        print("Service call failed: %s" %e)


def main():
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

 
    pub2 = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=1)
    pub3 = rospy.Publisher("/trans", Empty, queue_size=1)
    A_pose = Pose()
    MSG2 = Empty()
    
    teleport_msg = ObstacleLayoutRequest()
    teleport_msg.names = ["/World/object"]

    pose = PoseWithCovarianceStamped()
    pose.pose.pose.position.x = -8.0 #-8.34, -4.2433
    pose.pose.pose.position.y = 6.0 #5.98, 6.0
    pose.pose.pose.position.z = 0.0
    pose.pose.pose.orientation.x = 0.0
    pose.pose.pose.orientation.y  = 0.0
    pose.pose.pose.orientation.z  = 0.707
    pose.pose.pose.orientation.w = 0.707
    pose.header.frame_id="map"
    

    cmd_vel = Twist()
    cmd_vel.linear.x = 3.0
    cmd_vel.linear.y = 0.0
    cmd_vel.linear.z = 0.0

    cmd_vel.angular.x = 0.0
    cmd_vel.angular.y = 0.0
    cmd_vel.angular.z = 0.0

    

    while not rospy.is_shutdown():
        Key = input("press r or q")
        
        if Key == "r" or Key == "R":
            pub2.publish(pose)
            time.sleep(1)
            # pub.publish(cmd_vel)
            
            pub3.publish(MSG2)
            time.sleep(1)
            
            for i in range(6):
                
                A_pose.position.x = -9.0 
                np.random.seed(6+i)
                A_pose.position.y = 12.0 + i
                A_pose.position.z = 0.0
             
                A_pose.orientation.w = 1
                A_pose.orientation.x = 0
                A_pose.orientation.y = 0
                A_pose.orientation.z = 0

               
                teleport_msg.poses = [A_pose]
                teleport_client(teleport_msg)
                time.sleep(2)
            
            A_pose.position.x = -9.0 # -10.5 / -9.8
            A_pose.position.y = 3.0  # 11.1 / 10.1
            A_pose.position.z = 0.0


            A_pose.orientation.w = 1
            A_pose.orientation.x = 0
            A_pose.orientation.y = 0
            A_pose.orientation.z = 0

            
            

            teleport_msg.poses = [A_pose]
            teleport_client(teleport_msg)

            pub3.publish(MSG2)
        

        elif Key == "q" or Key=="Q":
            print("program is going to be shut down")
            break

        else:
            print("Wrong key has been input")

        cmd_vel.linear.x = 0.0
        pub.publish(cmd_vel)

if __name__== '__main__':
    rospy.init_node('Obstacle_randomization', anonymous=True)
    main()
