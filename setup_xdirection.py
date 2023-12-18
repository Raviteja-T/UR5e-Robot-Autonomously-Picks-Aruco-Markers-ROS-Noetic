#!/usr/bin/env python3

import rospy
import moveit_commander
from geometry_msgs.msg import PoseStamped
import tf
import time
import sys
from onrobot_rg_control.msg import OnRobotRGOutput

X_OFFSET = 0.03
Y_OFFSET = -0.035
TARGET_POSITION_OFFSET = 0.17

is_gripper_opened = False

def gen_command(command, is_open):
    max_width = 800
    max_force = 400

    if is_open:
        command.rGFR = 400
        command.rGWD = max_width
        command.rCTR = 16
    else:
        command.rGFR = 400
        command.rGWD = 0
        command.rCTR = 16

    return command

def move_to_target_position(group, gripper_pub, target_offset_x):
    global original_pose
    global is_gripper_opened

    tf_listener = tf.TransformListener()
    camera_frame = "camera_color_optical_frame"
    base_frame = "base"

    tf_listener.waitForTransform(base_frame, camera_frame, rospy.Time(), rospy.Duration(10.0))
    marker_in_cam_pose = rospy.wait_for_message("/aruco_tracker/pose", PoseStamped, timeout=10)

    try:
        original_pose = group.get_current_pose().pose

        marker_in_base = tf_listener.transformPose(base_frame, marker_in_cam_pose)

        target_pose = PoseStamped()
        target_pose.header.frame_id = base_frame
        target_pose.pose = marker_in_base.pose

        target_pose.pose.position.x = marker_in_base.pose.position.x + X_OFFSET
        target_pose.pose.position.y = marker_in_base.pose.position.y + Y_OFFSET
        target_pose.pose.position.z = marker_in_base.pose.position.z + TARGET_POSITION_OFFSET

        current_pose = group.get_current_pose().pose
        target_pose.pose.orientation = current_pose.orientation

        group.set_pose_target(target_pose)

        plan = group.go(wait=True)

        if plan:
            rospy.loginfo("Robot moved to target position successfully!")

            if not is_gripper_opened:
                gripper_command = gen_command(OnRobotRGOutput(), is_open=True)
                rospy.loginfo("Gripper command to open: {}".format(gripper_command))
                gripper_pub.publish(gripper_command)
                rospy.loginfo("Gripper opened.")
                time.sleep(1.0)
                is_gripper_opened = True

                # Add this section to close the gripper immediately after opening
                gripper_command = gen_command(OnRobotRGOutput(), is_open=False)
                rospy.loginfo("Gripper command to close: {}".format(gripper_command))
                gripper_pub.publish(gripper_command)
                rospy.loginfo("Gripper closed after opening.")

                # Return to the original position
                group.set_pose_target(original_pose)
                group.go(wait=True)
                rospy.loginfo("Returned to the original position.")

                # Move 3mm away from the previous target position along the x-axis
                target_pose.pose.position.x += target_offset_x
                group.set_pose_target(target_pose)
                plan = group.go(wait=True)

                if plan:
                    rospy.loginfo("Robot moved 3mm away from the previous target position along the x-axis.")

                    # Open the gripper
                    gripper_command = gen_command(OnRobotRGOutput(), is_open=True)
                    rospy.loginfo("Gripper command to open: {}".format(gripper_command))
                    gripper_pub.publish(gripper_command)
                    rospy.loginfo("Gripper opened.")
                    time.sleep(1.0)

                    # Return to the original position
                    group.set_pose_target(original_pose)
                    group.go(wait=True)
                    rospy.loginfo("Returned to the original position.")

                else:
                    rospy.loginfo("Failed to move the robot 3mm away from the previous target position along the x-axis.")

            else:
                rospy.loginfo("Gripper is already opened.")

        else:
            rospy.loginfo("Failed to move the robot to the target position.")

    except tf.LookupException as e:
        rospy.logerr("TF LookupException: %s", e)
    except tf.ConnectivityException as e:
        rospy.logerr("TF ConnectivityException: %s", e)
    except tf.ExtrapolationException as e:
        rospy.logerr("TF ExtrapolationException: %s", e)

def return_to_original_position(group, original_pose, gripper_pub):
    group.set_pose_target(original_pose)
    group.go(wait=True)
    rospy.loginfo("Returned to the original position.")

if __name__ == '__main__':
    try:
        moveit_commander.roscpp_initialize(sys.argv)
        group = moveit_commander.MoveGroupCommander("manipulator")
        rospy.init_node('automatic_gripper_controller', anonymous=True)
        gripper_pub = rospy.Publisher('/OnRobotRGOutput', OnRobotRGOutput, queue_size=1)

        original_pose = group.get_current_pose().pose

        move_to_target_position(group, gripper_pub, target_offset_x=0.03)  # Provide the desired offset in the x-axis
        return_to_original_position(group, original_pose, gripper_pub)

    except rospy.ROSInterruptException:
        pass
    finally:
        moveit_commander.roscpp_shutdown()
