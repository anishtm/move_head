#!/usr/bin/env python3

import rospy
from std_msgs.msg import String, Int32, Int32MultiArray, Bool
from move_head.msg import Face, Movement
from sensor_msgs.msg import Image  # Added for camera images
from time import time

# Store timestamps for different events
last_camera_time = None        # Image capture time
last_camera_det_time = None    # Detection processing time
last_detect_time = None        # Face detection time
last_command_time = None       # Command execution time
last_interrupt_time = None     # Interruption time

def chatter_callback(msg):
    current_time = rospy.get_time()
    if last_command_time is not None:
        delay = current_time - last_command_time  # Calculate the delay
        print(f"command to response received at: {rospy.get_time()} seconds, Delay: {delay:.3f} seconds")

    if last_detect_time is not None:
        delay = rospy.get_time() - last_detect_time  # Calculate the delay
        print(f"detection to response received at: {rospy.get_time()} seconds, Delay: {delay:.3f} seconds")

# Callback for image capture
def image_callback(msg):
    global last_camera_time
    last_camera_time = rospy.get_time()
    rospy.loginfo(f"Image published at: {last_camera_time:.3f} seconds")

# Callback for camera detection
def camera_det_cb(msg):
    global last_camera_time, last_camera_det_time
    last_camera_det_time = rospy.get_time()
    
    if last_camera_time is not None:
        delay = last_camera_det_time - last_camera_time
        rospy.loginfo(f"Image to Detection Delay: {delay:.3f} seconds")
    else:
        rospy.loginfo(f"Detection received at {last_camera_det_time:.3f}, but no prior image.")

# Callback for face detection
def face_callback(msg):
    global last_detect_time, last_camera_det_time, last_interrupt_time
    last_detect_time = rospy.get_time()
    rospy.loginfo(f"Face detected at: {last_detect_time:.3f} seconds")

    if last_camera_det_time is not None:
        delay = last_detect_time - last_camera_det_time
        rospy.loginfo(f"Detection Processing Delay: {delay:.3f} seconds")

    if last_interrupt_time is not None:
        delay = last_detect_time - last_interrupt_time
        rospy.loginfo(f"Detection to Interruption Delay: {delay:.3f} seconds")

# Callback for command execution
def command_callback(msg):
    global last_command_time
    last_command_time = rospy.get_time()
    rospy.loginfo(f"Command issued at: {last_command_time:.3f} seconds")

    if last_detect_time is not None:
        delay = last_command_time - last_detect_time
        rospy.loginfo(f"Detection to Command Execution Delay: {delay:.3f} seconds")

# Callback for interruption events
def interrupt_cb(msg):
    global last_interrupt_time
    last_interrupt_time = rospy.get_time()
    rospy.loginfo(f"Interruption detected at: {last_interrupt_time:.3f} seconds")

# ROS Node Initialization
def listener():
    rospy.init_node('delay_monitor', anonymous=True)

    rospy.Subscriber('/camera/image', Image, image_callback)  # Subscribe to image stream
    rospy.Subscriber('/camera_detection', Int32MultiArray, camera_det_cb)  # Detection from camera
    rospy.Subscriber('/chatter', String, chatter_callback)
    rospy.Subscriber('/face_detection', Face, face_callback)  # Face detection data
    rospy.Subscriber('/commands', Movement, command_callback)  # Commands sent to actuators
    rospy.Subscriber('/interrupt', Bool, interrupt_cb)  # Interruption handling

    rospy.spin()

if __name__ == '__main__':
    listener()
