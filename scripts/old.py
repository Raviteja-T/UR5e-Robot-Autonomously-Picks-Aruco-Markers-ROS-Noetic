#!/usr/bin/env python3

import rospy
import moveit_commander
from geometry_msgs.msg import PoseStamped
import tf
import numpy as np
from scipy.spatial.transform import Rotation as R
from tf.transformations import quaternion_matrix

def move_to_target_position():
    # Initialize the ROS node
    rospy.init_node('test2', anonymous=True)
    
    # Initialize MoveIt! Commander for controlling the robot
    robot = moveit_commander.RobotCommander()
    group = moveit_commander.MoveGroupCommander("manipulator")

    # Initialize TF listener to listen for transformations
    tf_listener = tf.TransformListener()
    now = rospy.Time(0)
    camera_frame = "camera_color_frame"
    base_frame = "base"
    
    # Wait for the transformation between camera and base frames
    rospy.sleep(2)
    tf_listener.waitForTransform(base_frame, camera_frame, now, rospy.Duration(10.0))
    (trans, rot) = tf_listener.lookupTransform(base_frame, camera_frame, now)

    rotation_matrix = np.array([
    [1 - 2*(rot[2]**2 + rot[3]**2), 2*(rot[1]*rot[2] - rot[0]*rot[3]), 2*(rot[0]*rot[2] + rot[1]*rot[3])],
    [2*(rot[1]*rot[2] + rot[0]*rot[3]), 1 - 2*(rot[1]**2 + rot[3]**2), 2*(rot[2]*rot[3] - rot[0]*rot[1])],
    [2*(rot[1]*rot[3] - rot[0]*rot[2]), 2*(rot[0]*rot[1] + rot[2]*rot[3]), 1 - 2*(rot[1]**2 + rot[2]**2)]
    ])

    transformation_matrix_base = np.eye(4)
    transformation_matrix_base[:3, :3] = rotation_matrix
    transformation_matrix_base[:3, 3] = trans

    # # Get ArUco marker pose in the camera frame
    marker_in_cam_pose = rospy.wait_for_message("/aruco_tracker/pose", PoseStamped, timeout=10)
    marker_in_base = tf_listener.transformPose("base", marker_in_cam_pose)

    # Extract position and orientation values
    position = np.array([0.06539324671030045, -0.001492900773882866, 0.24652165174484253])
    orientation = np.array([-0.6451713877897673, 0.7624654416836087, 0.04828147183965652, -0.00832046228532431])

    # Create a transformation matrix
    transform_matrix = np.identity(4)
    transform_matrix[:3, 3] = position

    # Convert quaternion to rotation matrix and update the transformation matrix
    rotation_matrix = quaternion_matrix(orientation)[:3, :3]
    transform_matrix[:3, :3] = rotation_matrix
    
    # Transformation parameters for UR5e
    transformation = {
        "qw": 0.9193200015852645,
        "qx": -0.015799009397755415,
        "qy": 0.019349078253015017,
        "qz": 0.39271712358652977,
        "x": 0.005916332566921131,
        "y": -0.04187592312900763,
        "z": -0.04717778627054241
    }

    # Extract quaternion and translation components
    qw = transformation["qw"]
    qx = transformation["qx"]
    qy = transformation["qy"]
    qz = transformation["qz"]
    x = transformation["x"]
    y = transformation["y"]
    z = transformation["z"]

    # Create a rotation matrix from the quaternion
    rotation_matrix_rob = quaternion_matrix([qx, qy, qz, qw])[:3, :3]

    # Create a transformation matrix
    transform_matrix_rob = np.identity(4)
    transform_matrix_rob[:3, :3] = rotation_matrix_rob
    transform_matrix_rob[:3, 3] = [x, y, z]

    matrix = np.matmul(transformation_matrix_base,transform_matrix, transform_matrix_rob)

    # Set the target pose based on the calculated transformation matrix
    target_pose = PoseStamped()
    target_pose.header.frame_id = "base"
    target_pose.pose.position.x = matrix[0, 3]
    target_pose.pose.position.y = matrix[1, 3]
    target_pose.pose.position.z = matrix[2, 3]
    target_pose.pose.orientation.x = matrix[0, 0]
    target_pose.pose.orientation.y = matrix[1, 0]
    target_pose.pose.orientation.z = matrix[2, 0]
    target_pose.pose.orientation.w = matrix[3, 0]

    # Set the target pose for the robot
    group.set_pose_target(target_pose)

    # Plan and execute the motion
    plan = group.go(wait=True)
    if plan:
        rospy.loginfo("Robot moved to target position successfully!")
    else:
        rospy.loginfo("Failed to move the robot to the target position.")

if __name__ == '__main__':
    try:
        move_to_target_position()
    except rospy.ROSInterruptException:
        pass