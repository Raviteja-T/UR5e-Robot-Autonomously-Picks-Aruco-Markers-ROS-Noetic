import rospy
import moveit_commander
from geometry_msgs.msg import PoseStamped
import tf
import numpy as np
from tf.transformations import quaternion_matrix
import sys

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

    print(marker_in_base)

    # Set the target pose for the robot
    group.set_pose_target(marker_in_base.pose)

    # Plan and execute the motion
    plan = group.go(wait=True)
    if plan:
        rospy.loginfo("Robot moved to target position successfully!")
    else:
        rospy.loginfo("Failed to move the robot to the target position.")

if __name__ == "__main__":
    move_to_target_position()
