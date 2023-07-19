#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import numpy as np
import time
import math as m




def main():
    rospy.init_node('Motion')

    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

    cmd_vel = Twist()
    cmd_vel.linear.x = 5.0
    cmd_vel.linear.y = 0.0
    cmd_vel.linear.z = 0.0

    cmd_vel.angular.x = 0.0
    cmd_vel.angular.y = 0.0
    cmd_vel.angular.z = 0.0

    time.sleep(1)
    pub.publish(cmd_vel)

    for i in range(5):

        # pub.publish(cmd_vel)
        
        time.sleep(2)

    cmd_vel.linear.x = 0.0
    pub.publish(cmd_vel)
if __name__ == "__main__":
    main()