#!/usr/bin/env python3

import rospy
import moveit_commander
import numpy as np

rospy.init_node('get_tool0_to_base_link', anonymous=True)

robot = moveit_commander.RobotCommander()
group_name = "manipulator"
group = moveit_commander.MoveGroupCommander(group_name)

# Get the current pose of the end effector (tool0)
end_effector_pose = group.get_current_pose().pose

# Get the transform from the base link to the end effector
base_link_to_end_effector_transform = group.get_current_pose().header.frame_id + '_tip'

# Get the transform from the base link to the end effector as a 4x4 homogeneous transformation matrix
base_link_to_end_effector_matrix = group.get_planning_frame().getFrameTransform(base_link_to_end_effector_transform).matrix

# Get the inverse of the transform to get the end effector pose relative to the base link
end_effector_to_base_link_matrix = np.linalg.inv(base_link_to_end_effector_matrix)

# Print the end effector pose relative to the base link
print("End effector pose relative to base link:")
print(end_effector_to_base_link_matrix)