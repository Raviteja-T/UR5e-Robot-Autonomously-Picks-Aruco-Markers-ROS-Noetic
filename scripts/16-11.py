#!/usr/bin/env python3

import rospy
import moveit_commander
from geometry_msgs.msg import PoseStamped
import tf
import time
import sys
from onrobot_rg_control.msg import OnRobotRGOutput

sys.path.append('/home/toor/catkin_ws/src/onrobot')

original_pose = None

def gen_command(command, is_open):
    max_width = 850
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


def move_to_target_position(gripper_pub):
    global original_pose

    robot = moveit_commander.RobotCommander()
    group = moveit_commander.MoveGroupCommander("manipulator")

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

        target_pose.pose.position.z += 0.20

        print(f"------ {target_pose.pose.position.z}")

        current_pose = group.get_current_pose().pose
        target_pose.pose.orientation = current_pose.orientation

        group.set_pose_target(target_pose)

        plan = group.go(wait=True)

        if plan:
            rospy.loginfo("Robot moved to target position successfully!")

            is_gripper_closed = True

            if is_gripper_closed:
                gripper_command = gen_command(OnRobotRGOutput(), is_open=True)
                rospy.loginfo("Gripper command to open: {}".format(gripper_command))
                gripper_pub.publish(gripper_command)
                rospy.loginfo("Gripper opened.")
                time.sleep(1.0)
                gripper_command = gen_command(OnRobotRGOutput(), is_open=False)
                rospy.loginfo("Gripper command to close: {}".format(gripper_command))
                gripper_pub.publish(gripper_command)
                rospy.loginfo("Gripper closed after opening.")

            else:
                gripper_command = gen_command(OnRobotRGOutput(), is_open=True)
                rospy.loginfo("Gripper command to open: {}".format(gripper_command))
                gripper_pub.publish(gripper_command)
                rospy.loginfo("Gripper opened.")
                time.sleep(5.0)
                gripper_command = gen_command(OnRobotRGOutput(), is_open=False)
                rospy.loginfo("Gripper command to close: {}".format(gripper_command))
                gripper_pub.publish(gripper_command)
                rospy.loginfo("Gripper closed.")

        else:
            rospy.loginfo("Failed to move the robot to the target position.")

    except tf.LookupException as e:
        rospy.logerr("TF LookupException: %s", e)
    except tf.ConnectivityException as e:
        rospy.logerr("TF ConnectivityException: %s", e)
    except tf.ExtrapolationException as e:
        rospy.logerr("TF ExtrapolationException: %s", e)

def return_to_original_position(gripper_pub):
    global original_pose

    group.set_pose_target(original_pose)
    group.go(wait=True)
    rospy.loginfo("Returned to the original position.")

    gripper_command = gen_command(OnRobotRGOutput(), is_open=True)
    rospy.loginfo("Gripper command to open: {}".format(gripper_command))
    gripper_pub.publish(gripper_command)
    rospy.loginfo("Gripper opened after returning to the original position.")

if __name__ == '__main__':
    try:

        print(f"===={original_pose}")

        group = moveit_commander.MoveGroupCommander("manipulator")
        rospy.init_node('automatic_gripper_controller', anonymous=True)
        gripper_pub = rospy.Publisher('/OnRobotRGOutput', OnRobotRGOutput, queue_size=1)

        move_to_target_position(gripper_pub)
        return_to_original_position(gripper_pub)

        

    except rospy.ROSInterruptException:
        pass

    
