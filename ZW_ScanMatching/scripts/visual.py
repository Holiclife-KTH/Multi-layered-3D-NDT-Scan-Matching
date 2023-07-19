#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import numpy as np
import time
import math as m
from ZW_ScanMatching.msg import plane
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

lf = []
lr = []



marker_pub = rospy.Publisher('marker_topic', Marker, queue_size=10)
marker_pub2 = rospy.Publisher('marker_topic2', Marker, queue_size=10)
r_pt = 0

def L_callback(plane_msg):

    point1 = Point()
    point2 = Point()
    point3 = Point()
    point4 = Point()
    # lf = [(plane_msg.plane[0].x + plane_msg.plane[3].x)/2,(plane_msg.plane[0].y + plane_msg.plane[3].y)/2]
    # lr = [(plane_msg.plane[1].x + plane_msg.plane[2].x)/2, (plane_msg.plane[1].y + plane_msg.plane[2].y)/2]
    point1.x = plane_msg.plane[0].x
    point1.y = plane_msg.plane[0].y
    point1.z = plane_msg.plane[0].z
    point2.x = plane_msg.plane[1].x
    point2.y = plane_msg.plane[1].y
    point2.z = plane_msg.plane[1].z
    point3.x = plane_msg.plane[2].x
    point3.y = plane_msg.plane[2].y
    point3.z = plane_msg.plane[2].z
    point4.x = plane_msg.plane[3].x
    point4.y = plane_msg.plane[3].y
    point4.z = plane_msg.plane[3].z
    # point2.x = lr[0]
    # point2.y = lr[1]

    marker = Marker()
    marker2 = Marker()
    marker.header.frame_id = "VLP16"
    marker.type = Marker.LINE_STRIP
    marker.action = Marker.ADD
    marker.scale.x = 0.1
    marker.color.r = 1.0
    marker.color.a = 1.0

    marker2.header.frame_id = "VLP16"
    marker2.type = Marker.LINE_STRIP
    marker2.action = Marker.ADD
    marker2.scale.x = 0.1
    marker2.color.b = 1.0
    marker2.color.a = 1.0

    marker.points.append(point1)
    marker.points.append(point2)
    marker2.points.append(point3)
    marker2.points.append(point4)
    marker_pub.publish(marker)
    marker_pub2.publish(marker2)
    


def main():
    rospy.init_node('Visual')
    
    rospy.Subscriber('/L_points', plane, L_callback)
    rospy.spin()

if __name__ == "__main__":
    main()