#!/usr/bin/env python3

import rospy
import moveit_commander
from geometry_msgs.msg import PoseStamped
import tf
import time
import sys

# Global variable for the original pose
original_pose = None

def move_to_target_position():
    global original_pose  # Make sure to use the global variable

    # Initialize the ROS node
    rospy.init_node('robot_control', anonymous=True)
    
    # Initialize MoveIt! Commander for controlling the robot
    robot = moveit_commander.RobotCommander()
    group = moveit_commander.MoveGroupCommander("manipulator")

    # Set the end effector link for planning (replace 'custom_end_effector_link' with the actual link name)
    group.set_end_effector_link('custom_end_effector_link')

    # Initialize TF listener to listen for transformations
    tf_listener = tf.TransformListener()
    camera_frame = "camera_color_optical_frame"
    base_frame = "base"
    
    # Wait for the transformation between camera and base frames
    tf_listener.waitForTransform(base_frame, camera_frame, rospy.Time(), rospy.Duration(10.0))

    # Get ArUco marker pose in the camera frame
    marker_in_cam_pose = rospy.wait_for_message("/aruco_tracker/pose", PoseStamped, timeout=10)

    try:
        # Store the original pose of the robot arm
        original_pose = group.get_current_pose().pose

        # Transform the marker pose to the base frame
        marker_in_base = tf_listener.transformPose(base_frame, marker_in_cam_pose)

        # Set the target pose based on the transformed marker pose with offset
        target_pose = PoseStamped()
        target_pose.header.frame_id = base_frame

        # Consider the offset from wrist 3 to the custom end effector
        offset_from_wrist3_to_custom_end_effector = 0.2  # Replace with the actual offset in meters
        target_pose.pose.position.x = marker_in_base.pose.position.x + offset_from_wrist3_to_custom_end_effector

        # Modify the Z-axis value (move up by 200 mm)
        target_pose.pose.position.z = marker_in_base.pose.position.z + 0.16  # 200 mm = 0.2 meters

        # Set a fixed orientation (no rotation)
        target_pose.pose.orientation.x = 0.0
        target_pose.pose.orientation.y = 0.0
        target_pose.pose.orientation.z = 0.0
        target_pose.pose.orientation.w = 1.0

        # Set the target pose for the custom end effector link
        group.set_pose_target(target_pose)

        # Plan and execute the motion
        plan = group.go(wait=True)
        
        if plan:
            rospy.loginfo("Robot moved to target position successfully!")

            # Wait for 3 seconds at the target position
            rospy.loginfo("Waiting for 3 seconds...")
            time.sleep(3.0)

        else:
            rospy.loginfo("Failed to move the robot to the target position.")
    
    except tf.LookupException as e:
        rospy.logerr("TF LookupException: %s", e)
    except tf.ConnectivityException as e:
        rospy.logerr("TF ConnectivityException: %s", e)
    except tf.ExtrapolationException as e:
        rospy.logerr("TF ExtrapolationException: %s", e)

def return_to_original_position():
    global original_pose  # Make sure to use the global variable

    # Return to the original position
    group.set_pose_target(original_pose)  # Use the original pose
    group.go(wait=True)
    rospy.loginfo("Returned to the original position.")

if __name__ == '__main__':
    try:
        # Initialize MoveIt! Commander for controlling the robot
        group = moveit_commander.MoveGroupCommander("manipulator")

        move_to_target_position()

        # Return to the original position after waiting for 3 seconds
        return_to_original_position()

    except rospy.ROSInterruptException:
        pass