#!/usr/bin/env python3

import rospy
import tf
from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseStamped

def apriltag_callback(msg):
    if not msg.detections:
        return

    # World에서 각 AprilTag의 위치
    apriltag_poses = {
        0: {'position': (0.3, 0.5, 0.0005), 'orientation': (0, 0, 0)},
        1: {'position': (-0.6, 0.5, 0.0005), 'orientation': (0, 0, 0)},
        2: {'position': (0.3, -0.5, 0.0005), 'orientation': (0, 0, 0)},
        3: {'position': (-0.6, -0.5, 0.0005), 'orientation': (0, 0, 0)},
    }

    listener = tf.TransformListener()
    
    # 카메라에서 태그까지의 변환을 추출하고, 그에 따라 UR5e base_link의 위치를 계산
    for detection in msg.detections:
        tag_id = detection.id[0]

        if tag_id in apriltag_poses:
            tag_pose = PoseStamped()
            tag_pose.header = detection.pose.header
            tag_pose.pose = detection.pose.pose.pose

            try:
                listener.waitForTransform('Camera', 'base_link', rospy.Time(0), rospy.Duration(3.0))
                tag_pose.header.stamp = rospy.Time(0)
                base_link_pose = listener.transformPose('base_link', tag_pose)

                print("UR5e base_link Pose:")
                print(base_link_pose)

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                # print("yes")
                continue

def main():
    rospy.init_node('apriltag_detection_ver2', anonymous=True)

    rospy.Subscriber('/tag_detections', AprilTagDetectionArray, apriltag_callback)

    rospy.spin()

if __name__ == '__main__':
    main()