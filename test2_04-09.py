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

    # Create a transformation matrix
    transform_matrix = np.identity(4)

    # Transformation parameters for UR5e
    transformation = {
        "qw": 0.9048048566345402,
        "qx": 0.0167295832083262,
        "qy": 0.03225741341420352,
        "qz": -0.42427320412212066,
        "x": -0.04007844951698071,
        "y": 0.031124296013620312,
        "z": -0.2861671281011557
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

    matrix = np.matmul(transformation_matrix_base, transform_matrix, transform_matrix_rob)

    # Set the target position based on the calculated transformation matrix
    target_position = np.array([matrix[0, 3], matrix[1, 3], matrix[2, 3]])

    # Get the current orientation of the robot's end effector
    current_orientation = group.get_current_pose().pose.orientation

    # Set the target pose with the desired position and current orientation
    target_pose = PoseStamped()
    target_pose.header.frame_id = "base"
    target_pose.pose.position.x = target_position[0]
    target_pose.pose.position.y = target_position[1]
    target_pose.pose.position.z = target_position[2]
    target_pose.pose.orientation = current_orientation

    # Set the target pose for the robot
    group.set_pose_target(target_pose)

    group.set_max_velocity_scaling_factor(0.1)  # 10% of the maximum speed

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