#!/usr/bin/env python3
import rospy
from apriltag_ros.msg import AprilTagDetectionArray

def callback(data):
    # print(data)
    print(data.detections[0])
    # for detection in data.detections:
    #     print("Tag ID: {}".format(detection.id[0]))
    #     print("Tag Pose: {}".format(detection.pose.pose))

def listener():
    rospy.init_node('apriltag_listener', anonymous=True)
    rospy.Subscriber("/tag_detections", AprilTagDetectionArray, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()