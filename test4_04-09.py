import rospy
import moveit_commander
from geometry_msgs.msg import PoseStamped
import tf
import numpy as np
from tf.transformations import quaternion_matrix
import sys
import math

def move_to_target_position():
    # Initialize the ROS node
    rospy.init_node('test2', anonymous=True)

    # Initialize MoveIt! Commander for controlling the robot
    moveit_commander.roscpp_initialize(sys.argv)
    robot = moveit_commander.RobotCommander()
    group = moveit_commander.MoveGroupCommander("manipulator")

    # Initialize TF listener to listen for transformations
    tf_listener = tf.TransformListener()
    rospy.sleep(2)  # Wait for some time before starting transformations

    camera_frame = "camera_color_frame"
    base_frame = "base"

    now = rospy.Time(0)
    tf_listener.waitForTransform(base_frame, camera_frame, now, rospy.Duration(10.0))
    (trans, rot) = tf_listener.lookupTransform(base_frame, camera_frame, now)

    # Calculate rotation matrix from quaternion
    rotation_matrix = quaternion_matrix(rot)[:3, :3]

    transformation_matrix_base = np.eye(4)
    transformation_matrix_base[:3, :3] = rotation_matrix
    transformation_matrix_base[:3, 3] = trans

    # Get ArUco marker pose in the camera frame
    marker_in_cam_pose = rospy.wait_for_message("/aruco_tracker/pose", PoseStamped, timeout=10)
    marker_in_base = tf_listener.transformPose(base_frame, marker_in_cam_pose)

    # Create a new PoseStamped message with the same orientation as the current end effector
    target_pose = PoseStamped()
    target_pose.header.frame_id = base_frame
    target_pose.pose.position = marker_in_base.pose.position

    # Keep the orientation the same as the current end effector orientation
    current_pose = group.get_current_pose()
    target_pose.pose.orientation = current_pose.pose.orientation

    print(target_pose)

    # Set the target pose for the robot
    group.set_pose_target(target_pose)

    # Plan the motion
    plan = group.plan()
    
    if not plan:
        rospy.loginfo("Failed to plan the robot motion.")
        return

    # Define the desired wrist 3 joint angle (in radians)
    desired_wrist3_angle = math.radians(90)  # Change this angle as needed

    # Execute the motion while checking the wrist 3 joint angle
    for waypoint in plan.joint_trajectory.points:
        current_joint_values = waypoint.positions
        current_wrist3_angle = current_joint_values[-1]  # Wrist 3 joint angle

        if abs(current_wrist3_angle - desired_wrist3_angle) <= math.radians(5):  # Check within 5 degrees
            rospy.loginfo("Wrist 3 joint angle is within 5 degrees of the desired angle. Stopping.")
            break

        group.set_joint_value_target(current_joint_values)  # Set the current joint values
        group.go(wait=True)  # Move to the next waypoint

if __name__ == "__main__":
    move_to_target_position()
