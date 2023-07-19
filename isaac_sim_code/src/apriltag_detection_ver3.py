#!/usr/bin/env python3

import rospy
import tf
import numpy as np
from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseStamped, Quaternion
from tf.transformations import *

def pose_to_transform(pose):
    T = np.eye(4)
    T[:3, 3] = [pose.position.x, pose.position.y, pose.position.z]
    quat = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
    T[:3, :3] = quaternion_matrix(quat)[:3, :3]
    return T

def transform_to_pose(T):
    pose = PoseStamped()
    pose.pose.position.x = T[0, 3]
    pose.pose.position.y = T[1, 3]
    pose.pose.position.z = T[2, 3]
    pose.pose.orientation = Quaternion(*quaternion_from_matrix(T))
    return pose

def apriltag_callback(msg):
    if not msg.detections:
        return

    # Each AprilTag's position in the world
    apriltag_poses = {
        0: {'position': (0.3, 0.5, 0.0005), 'orientation': (0, 0, 0)},
        1: {'position': (-0.6, 0.5, 0.0005), 'orientation': (0, 0, 0)},
        2: {'position': (0.3, -0.5, 0.0005), 'orientation': (0, 0, 0)},
        3: {'position': (-0.6, -0.5, 0.0005), 'orientation': (0, 0, 0)},
    }

    listener = tf.TransformListener()

    for detection in msg.detections:
        tag_id = detection.id[0]

        if tag_id in apriltag_poses:
            tag_pose_actual = apriltag_poses[tag_id]

            try:
                # Convert the actual tag pose to a transformation matrix
                tag_position_actual = tag_pose_actual['position']
                tag_orientation_actual = quaternion_from_euler(*[np.radians(angle) for angle in tag_pose_actual['orientation']])
                world_to_tag_T_actual = np.eye(4)
                world_to_tag_T_actual[:3, 3] = np.array(tag_position_actual)
                world_to_tag_T_actual[:3, :3] = np.array(quaternion_matrix(tag_orientation_actual)[:3, :3])
                
                # Get the inverse transform from the tag to the camera
                tag_to_camera_T = np.linalg.inv(pose_to_transform(detection.pose.pose.pose))

                # Calculate the world to camera transform
                world_to_camera_T = np.dot(world_to_tag_T_actual, tag_to_camera_T)

                listener.waitForTransform('Camera', 'base_link', rospy.Time(0), rospy.Duration(3.0))
                (trans, rot) = listener.lookupTransform('Camera', 'base_link', rospy.Time(0))
                Camera_to_base_link_T = np.eye(4)
                Camera_to_base_link_T[:3, 3] = np.array(trans)
                Camera_to_base_link_T[:3, :3] = np.array(quaternion_matrix(rot)[:3, :3])

                # Calculate the world to base_link transform
                world_to_base_link_T = np.dot(world_to_camera_T, Camera_to_base_link_T)

                # Apply the offset
                base_link_offset = np.eye(4)
                base_link_offset[:3, 3] = np.array([0.2, 0, -0.415])
                world_to_base_link_T = np.dot(world_to_base_link_T, base_link_offset)

                # Convert the world_to_base_link_T to a pose message
                # base_link_pose = transform_to_pose(world_to_base_link_T)
                # base_link_pose.header.stamp = rospy.Time.now()
                # base_link_pose.header.frame_id = 'world'

                # print("Carter_V2 Pose:")
                # print(base_link_pose)

                # Print the rotation matrix and Euler angles
                position = world_to_base_link_T[:3, 3]
                rotation_matrix = world_to_base_link_T[:3, :3]
                euler_angles = euler_from_matrix(rotation_matrix)

                print("Position:")
                print(position)
                # print("Rotation Matrix:")
                # print(rotation_matrix)
                # print("Euler Angles (in radians):")
                # print(euler_angles)
                print("Euler Angles (in degrees):")
                print(np.degrees(euler_angles))

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

def main():
    rospy.init_node('apriltag_detection_ver3', anonymous=True)

    rospy.Subscriber('/tag_detections', AprilTagDetectionArray, apriltag_callback)

    rospy.spin()

if __name__ == '__main__':
    main()