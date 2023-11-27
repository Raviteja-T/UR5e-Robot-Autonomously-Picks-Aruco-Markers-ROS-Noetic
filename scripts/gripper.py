#!/usr/bin/env python3

import rospy
from onrobot_rg_control.msg import OnRobotRGOutput

def gen_command(command, is_open):
    """Generates the gripper command based on whether it should open or close."""
    max_width = 850  # Adjust based on your gripper specifications
    max_force = 400   # Adjust based on your gripper specifications

    if is_open:
        command.rGFR = 400
        command.rGWD = max_width
        command.rCTR = 16
    else:
        command.rGFR = 400
        command.rGWD = 400
        command.rCTR = 16

    return command

def main():
    rospy.init_node('automatic_gripper_controller', anonymous=True)
    gripper_pub = rospy.Publisher('/OnRobotRGOutput', OnRobotRGOutput, queue_size=1)

    command = OnRobotRGOutput()  # Replace with your actual message type

    rate = rospy.Rate(0.2)  # Adjust the rate based on your requirements (e.g., every 5 seconds)
    is_open = False

    while not rospy.is_shutdown():
        command = gen_command(command, is_open)
        gripper_pub.publish(command)
        is_open = not is_open  # Toggle between open and close

        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
