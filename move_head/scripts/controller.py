#!/usr/bin/env python3

import rospy
import time
from move_head.msg import Face  
from std_msgs.msg import Int32, String

# Track timestamps
last_command_time = 0
last_face_time = time.time()
calibration_done = False  # Flag to track if calibration is completed

def calibrate():
    """Send the calibration command and wait for completion."""
    global calibration_done
    rospy.loginfo("Sending calibration command: cal")
    pub.publish("cal")
    rospy.sleep(2)
    calibration_done = True
    rospy.loginfo("Calibration complete. Ready to process face detection.")

def face_callback(msg):
    """Handle face detection messages."""
    global last_command_time, last_face_time

    if not calibration_done:
        return  # Ignore face detection until calibration is complete

    current_time = time.time()

    # Update last face detected time
    last_face_time = current_time

    # Compute face center
    center_x = (msg.box_left + msg.box_right) / 2
    center_y = (msg.box_top + msg.box_bottom) / 2

    # Map to motor positions
    stepper_pos = int(500 + (center_x / 256.0) * 3700)
    servo_angle = int((center_y / 256.0) * 36.0)

    # Throttle command sending (only send if last sent > 5 sec ago)
    if current_time - last_command_time > 0.5:
        command = f"pos {stepper_pos}"  
        rospy.loginfo(f"Sending command: {command}")
        pub.publish(command)
        last_command_time = current_time  # Update last command time

    # Publish to motor topics
    stepper_pub.publish(stepper_pos)
    servo_pub.publish(servo_angle)

    rospy.loginfo(f"Stepper: {stepper_pos}, Servo: {servo_angle}")

def check_no_face_detected():
    """Check if no face is detected for a period and send 'cen' command."""
    global last_face_time
    current_time = time.time()

    if not calibration_done:
        return  # Don't perform checks until calibration is complete

    # If no face detected for 30+ seconds, send "cen" command
    if current_time - last_face_time > 30:
        rospy.loginfo("No face detected for 30 sec. Sending 'cen' command.")
        pub.publish("cen")
        # Reset last_face_time to avoid repeated "cen" commands
        last_face_time = current_time

# ROS Node Initialization
rospy.init_node('head_controller', anonymous=True) 

# Publishers
pub = rospy.Publisher("/arduino_command", String, queue_size=10)
stepper_pub = rospy.Publisher('/stepper_position', Int32, queue_size=10)
servo_pub = rospy.Publisher('/servo_angle', Int32, queue_size=10)

# Calibration (blocking until completed)
calibrate()

# Subscribe to Face Detection Topic
rospy.Subscriber('/face_detection', Face, face_callback)

# Main loop for periodic checks
rate = rospy.Rate(1)  # 1Hz loop (1-second checks)
while not rospy.is_shutdown():
    check_no_face_detected()  # Check if no face is detected for 30 seconds
    rate.sleep()

rospy.spin()
