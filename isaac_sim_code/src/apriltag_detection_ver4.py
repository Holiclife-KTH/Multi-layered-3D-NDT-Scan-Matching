#!/usr/bin/env python3

import rospy
import tf
import numpy as np
from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseStamped, Quaternion

def apriltag_callback(msg, listener):
    if not msg.detections:
        return

    # Each AprilTag's position in the world
    apriltag_poses = {
        0: {'position': (0.3, 0.5, 0.0005), 'orientation': (0, 0, 0)},
        1: {'position': (-0.6, 0.5, 0.0005), 'orientation': (0, 0, 0)},
        2: {'position': (0.3, -0.5, 0.0005), 'orientation': (0, 0, 0)},
        3: {'position': (-0.6, -0.5, 0.0005), 'orientation': (0, 0, 0)},
    }

    for detection in msg.detections:
        tag_id = detection.id[0]

        if tag_id in apriltag_poses:
            tag_pose_actual = apriltag_poses[tag_id]

            try:
                # Get the tag to camera transform
                tag_to_camera_T = listener.fromTranslationRotation((detection.pose.pose.pose.position.x, detection.pose.pose.pose.position.y, detection.pose.pose.pose.position.z),
                        (detection.pose.pose.pose.orientation.x, detection.pose.pose.pose.orientation.y, detection.pose.pose.pose.orientation.z, detection.pose.pose.pose.orientation.w))
                
                # Convert the actual tag pose to a transformation matrix
                world_to_tag_T_actual = listener.fromTranslationRotation(tag_pose_actual['position'], tf.transformations.quaternion_from_euler(*tag_pose_actual['orientation']))
                
                # Calculate the world to camera transform
                world_to_camera_T = np.dot(world_to_tag_T_actual, np.linalg.inv(tag_to_camera_T))

                listener.waitForTransform('Camera', 'Group', rospy.Time(0), rospy.Duration(3.0))
                (trans, rot) = listener.lookupTransform('Camera', 'Group', rospy.Time(0))
                Camera_to_base_T = listener.fromTranslationRotation(trans, rot)

                # Calculate the world to base transform
                world_to_base_link_T = np.dot(world_to_camera_T, Camera_to_base_T)

                # Print the rotation matrix and Euler angles
                position = world_to_base_link_T[:3, 3]
                rotation_matrix = world_to_base_link_T[:3, :3]
                euler_angles = tf.transformations.euler_from_matrix(rotation_matrix)

                print("ID:")
                print(tag_id)
                print("Position:")
                print(position)
                print("Euler Angles (in degrees):")
                print(np.degrees(euler_angles))
                print("-------------------------------------------------")

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

def main():
    rospy.init_node('apriltag_detection_ver4', anonymous=True)
    listener = tf.TransformListener()
    rospy.Subscriber('/tag_detections', AprilTagDetectionArray, apriltag_callback, callback_args=listener)
    rospy.spin()

if __name__ == '__main__':
    main()
