#!/usr/bin/env python3

import rospy
import time
from move_head.msg import Face  
from std_msgs.msg import Int32, String

# ROS Node Initialization
rospy.init_node('aa_controller', anonymous=True) 

# Publishers
pub = rospy.Publisher("/arduino_command", String, queue_size=10)

# Initial Calibration
command = "cal"  
rospy.loginfo(f"Sending calibration command: {command}")
pub.publish(command)

rospy.sleep(20)

# Looping until the node is shut down

while not rospy.is_shutdown():
    command = "pos 500"  
    rospy.loginfo(f"Sending position command: {command}")
    pub.publish(command)
    
    rospy.sleep(5)

    command = "pos 4200"  
    rospy.loginfo(f"Sending position command: {command}")
    pub.publish(command)
    rospy.sleep(5)

# No need for rospy.spin() here since we are using a while loop
