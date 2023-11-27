#!/usr/bin/env python3

import rospy
import moveit_commander
from geometry_msgs.msg import PoseStamped
import tf
from std_msgs.msg import Float64

def move_to_target_position():
    # Initialize the ROS node
    rospy.init_node('test2', anonymous=True)
    
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
        # Transform the marker pose to the base frame
        marker_in_base = tf_listener.transformPose(base_frame, marker_in_cam_pose)

        # Set the target pose based on the transformed marker pose
        target_pose = PoseStamped()
        target_pose.header.frame_id = base_frame
        target_pose.pose = marker_in_base.pose

        # Get the current pose of the robot arm
        current_pose = group.get_current_pose().pose

        # Preserve the entire current orientation
        target_pose.pose.orientation = current_pose.orientation

        # Set the target pose for the robot
        group.set_pose_target(target_pose)

        # Plan and execute the motion
        plan = group.go(wait=True)
        
        if plan:
            rospy.loginfo("Robot moved to target position successfully!")

            # Example: Control the gripper
            gripper_cmd = Float64()
            gripper_cmd.data = 0.5  # Replace with the desired gripper position (0.0 = fully open, 1.0 = fully closed)
            gripper_pub = rospy.Publisher('/OnRobotRGInput', Float64, queue_size=10)
            gripper_pub.publish(gripper_cmd)
            rospy.sleep(2)  # Allow some time for the gripper to move
            
            # Optionally, you can close the gripper after a few seconds
            gripper_cmd.data = 0.0  # Replace with the desired gripper position (0.0 = fully open, 1.0 = fully closed)
            gripper_pub.publish(gripper_cmd)
            rospy.loginfo("Gripper closed.")
        else:
            rospy.loginfo("Failed to move the robot to the target position.")
    
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
