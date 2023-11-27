#!/usr/bin/env python3

import rospy
import moveit_commander
from geometry_msgs.msg import PoseStamped
import tf
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist  # Import Twist message for base movement
import time

def move_to_target_position():
    # Initialize the ROS node
    rospy.init_node('mobile_manipulation', anonymous=True)
    
    # Initialize MoveIt! Commander for controlling the robot
    robot = moveit_commander.RobotCommander()
    group = moveit_commander.MoveGroupCommander("manipulator")

    # Initialize TF listener to listen for transformations
    tf_listener = tf.TransformListener()
    camera_frame = "camera_color_optical_frame"
    base_frame = "base"

    # Initialize a publisher for mobile base movement
    base_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    
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

        # Modify the Z-axis value (move up by 200 mm)
        target_pose.pose.position.z += 0.16  # 200 mm = 0.2 meters

        # Move the mobile base to the target position
        base_cmd = Twist()
        base_cmd.linear.x = 0.2  # Adjust the linear velocity as needed
        base_pub.publish(base_cmd)

        # Plan and execute the manipulator motion
        group.set_pose_target(target_pose)
        plan = group.go(wait=True)

        if plan:
            rospy.loginfo("Robot moved to target position successfully!")

            # Wait for 3 seconds at the target position
            rospy.loginfo("Waiting for 3 seconds...")
            time.sleep(3.0)

            # Open the gripper
            gripper_cmd = Float64()
            gripper_cmd.data = 0.0  # 0.0 = fully open
            gripper_pub = rospy.Publisher('/OnRobotRGSimpleController_57411_1698987076251', Float64, queue_size=10)
            gripper_pub.publish(gripper_cmd)

            rospy.loginfo("Gripper opened.")

            # Stop the base movement
            base_cmd.linear.x = 0.0
            base_pub.publish(base_cmd)

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
