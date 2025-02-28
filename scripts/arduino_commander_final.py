#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Point32

# Mapping of keyboard inputs to movement commands
keyboard_commands = {
    "0": Point32(z=0, y=84, x=2335),
    "1": Point32(z=4, y=100, x=4200),
    "2": Point32(z=4, y=100, x=2335),
    "3": Point32(z=4, y=100, x=500),
    "4": Point32(z=4, y=84, x=4200),
    "5": Point32(z=4, y=84, x=2335),
    "6": Point32(z=4, y=84, x=500),
    "7": Point32(z=4, y=70, x=4200),
    "8": Point32(z=4, y=70, x=2335),
    "9": Point32(z=4, y=70, x=500),
}

def main():
    rospy.init_node("keyboard_controller")
    pub = rospy.Publisher("/commands", Point32, queue_size=1)
    rospy.loginfo("Keyboard Controller Initialized. Enter a number (0-9) to send commands.")

    try:
        while not rospy.is_shutdown():
            key = input("Enter command (0-9): ").strip()
            if key in keyboard_commands:
                pub.publish(keyboard_commands[key])
                rospy.loginfo(f"Sent command: {keyboard_commands[key]}")
            else:
                rospy.logwarn("Invalid input. Enter a number between 1 and 9.")
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down keyboard controller.")

if __name__ == "__main__":
    main()

