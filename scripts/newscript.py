#!/usr/bin/env python3

import rospy
import moveit_commander
from geometry_msgs.msg import PoseStamped
import tf
import time
import math

def move_to_target_position():
    # Initialize the ROS node
    rospy.init_node('newscript', anonymous=True)
    
    # Initialize MoveIt! Commander for controlling the robot
    robot = moveit_commander.RobotCommander()
    group = moveit_commander.MoveGroupCommander("manipulator")

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

        # Set the target pose based on the transformed marker pose
        target_pose = PoseStamped()
        target_pose.header.frame_id = base_frame
        target_pose.pose = marker_in_base.pose

        # Modify the Z-axis value (move up by 200 mm)
        target_pose.pose.position.z += 0.14  # 200 mm = 0.2 meters

        # Set a fixed orientation (quaternion) to avoid gripper rotation
        target_orientation = [0.0, 0.0, 0.0, 1.0]  # Replace with your desired orientation
        target_pose.pose.orientation.x = target_orientation[0]
        target_pose.pose.orientation.y = target_orientation[1]
        target_pose.pose.orientation.z = target_orientation[2]
        target_pose.pose.orientation.w = target_orientation[3]

        # Get the current pose of the robot arm
        current_pose = group.get_current_pose().pose

        # Check the distance between the current pose and the target pose
        distance_threshold = 0.05  # Adjust the threshold as needed
        distance = math.sqrt((current_pose.position.x - target_pose.pose.position.x)**2 +
                             (current_pose.position.y - target_pose.pose.position.y)**2 +
                             (current_pose.position.z - target_pose.pose.position.z)**2)

        if distance > distance_threshold:
            # Set the target pose for the robot
            group.set_pose_target(target_pose)

            # Plan and execute the motion
            plan = group.go(wait=True)
            
            if plan:
                rospy.loginfo("Robot moved to the target position successfully!")

                # Wait for 3 seconds at the target position
                rospy.loginfo("Waiting for 3 seconds...")
                time.sleep(3.0)

                # Return to the original position
                group.set_pose_target(original_pose)  # Use the original pose
                group.go(wait=True)
                
                rospy.loginfo("Returned to the original position.")

            else:
                rospy.loginfo("Failed to move the robot to the target position.")
        else:
            rospy.loginfo("Already close to the target position. No need to move.")

    except tf.LookupException as e:
        rospy.logerr("TF LookupException: %s", e)
    except tf.ConnectivityException as e:
        rospy.logerr("TF ConnectivityException: %s", e)
    except tf.ExtrapolationException as e:
        rospy.logerr("TF ExtrapolationException: %s", e)

if __name__ == '__main__':
    try:
        move_to_target_position()
    except rospy.ROSInterruptException:
        pass
