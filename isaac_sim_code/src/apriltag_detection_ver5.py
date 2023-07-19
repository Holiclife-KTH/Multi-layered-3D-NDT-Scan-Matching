#!/usr/bin/env python3

import rospy
import tf
import numpy as np
from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseStamped, Quaternion

group_poses = []
best_pose_estimate = None

def error_metric(correct_pose, estimated_pose, position_weight=1, orientation_weight=1):
    position_diff = np.linalg.norm(np.array(correct_pose['position']) - np.array(estimated_pose['position']))
    correct_quat = tf.transformations.quaternion_from_euler(*correct_pose['orientation'])
    estimated_quat = tf.transformations.quaternion_from_euler(*estimated_pose['orientation'])
    orientation_diff = np.abs(np.dot(correct_quat, estimated_quat))
    return position_weight * position_diff + orientation_weight * orientation_diff

def ransac_pose_estimation(poses, n_samples=10):
    correct_pose = {'position': (0, 0, 0), 'orientation': (0, 0, 0)}
    error_threshold = 0.3
    max_iterations = 20
    best_inlier_count = 0
    global best_pose_estimate

    for _ in range(max_iterations):
        if len(poses) >= n_samples:
            sample_poses = np.random.choice(poses, n_samples, replace=False)
        else:
            sample_poses = poses

        inlier_count = 0
        inlier_sum = {'position': np.array([0.0, 0.0, 0.0]), 'orientation': np.array([0.0, 0.0, 0.0])}

        for pose in poses:
            error_sum = 0
            for sample_pose in sample_poses:
                error_sum += error_metric(correct_pose, sample_pose)
            error_avg = error_sum / n_samples

            if error_avg < error_threshold:
                inlier_count += 1
                inlier_sum['position'] += np.array(pose['position'])
                inlier_sum['orientation'] += np.array(pose['orientation'])

        if inlier_count > best_inlier_count:
            best_inlier_count = inlier_count
            best_pose_estimate = {'position': tuple(inlier_sum['position'] / inlier_count),
                                  'orientation': tuple(inlier_sum['orientation'] / inlier_count)}


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

    global group_poses

    for detection in msg.detections:
        tag_id = detection.id[0]

        if tag_id in apriltag_poses:
            tag_pose_actual = apriltag_poses[tag_id]

            try:
                tag_to_camera_T = listener.fromTranslationRotation((detection.pose.pose.pose.position.x, detection.pose.pose.pose.position.y, detection.pose.pose.pose.position.z),
                        (detection.pose.pose.pose.orientation.x, detection.pose.pose.pose.orientation.y, detection.pose.pose.pose.orientation.z, detection.pose.pose.pose.orientation.w))
                
                world_to_tag_T_actual = listener.fromTranslationRotation(tag_pose_actual['position'], tf.transformations.quaternion_from_euler(*tag_pose_actual['orientation']))
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

                group_poses.append({'position': position, 'orientation': euler_angles})

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

    # Apply RANSAC to estimate the group pose
    ransac_pose_estimation(group_poses)
    print("Estimated Group Pose:")
    print("Position:")
    print(best_pose_estimate['position'])
    print("Euler Angles (in degrees):")
    print(np.degrees(best_pose_estimate['orientation']))

def main():
    rospy.init_node('apriltag_detection_ver5', anonymous=True)
    listener = tf.TransformListener()
    rospy.Subscriber('/tag_detections', AprilTagDetectionArray, apriltag_callback, callback_args=listener)
    rospy.spin()

if __name__ == '__main__':
    main()