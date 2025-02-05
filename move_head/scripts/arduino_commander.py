#!/usr/bin/env python3
import rospy
import serial
import time
import subprocess
from std_msgs.msg import String  # ROS message type for commands

# Initialize serial connection
def open_serial_connection(port, baudrate, timeout):
    try:
        return serial.Serial(port=port, baudrate=baudrate, timeout=timeout)
    except serial.SerialException as e:
        rospy.logerr(f"Error: {e}")
        rospy.loginfo("Attempting to free up the port...")
        try:
            subprocess.run(["fuser", "-k", port], check=True)
            rospy.loginfo("Port freed. Retrying...")
            return serial.Serial(port=port, baudrate=baudrate, timeout=timeout)
        except Exception as inner_e:
            rospy.logerr(f"Failed to free the port or retry connection: {inner_e}")
            return None

# Functions to generate Arduino commands
def calibrate():
    rospy.loginfo("Calibrating...")
    return f"CAL\n"

def move_servo(angle):
    rospy.loginfo(f"Moving servo to angle {angle}")
    return f"ANG{angle:03d}\n"

def move_stepper(position):
    rospy.loginfo(f"Moving stepper to position {position}")
    return f"POS{position:04d}\n"

def move_stepper_servo(position, angle):
    rospy.loginfo(f"Moving stepper to {position} and servo to angle {angle}")
    return f"MOV{position:04d}{angle:03d}\n"

def center():
    rospy.loginfo(f"Centering")
    return f"CEN\n"

# Command handler
def command_callback(msg):
    """Handles incoming commands from ROS topic"""
    global arduino1  # Reference to serial connection

    parts = msg.data.strip().split()
    if len(parts) == 0:
        rospy.logwarn("Empty command received!")
        return

    commandToSend = None
    try:
        if parts[0].lower() == "cal":
            commandToSend = calibrate()

        elif parts[0].lower() == "ang" and len(parts) == 2:
            angle = int(parts[1])
            commandToSend = move_servo(angle)

        elif parts[0].lower() == "pos" and len(parts) == 2:
            position = int(parts[1])
            commandToSend = move_stepper(position)

        elif parts[0].lower() == "mov" and len(parts) == 3:
            position = int(parts[1])
            angle = int(parts[2])
            commandToSend = move_stepper_servo(position, angle)

        elif parts[0].lower() == "cen":
            commandToSend = center()
            
        else:
            rospy.logwarn("Invalid command format!")

        # Send the command if valid
        if commandToSend and arduino1 and arduino1.is_open:
            arduino1.write(commandToSend.encode('utf-8'))
            rospy.loginfo(f"Sent command: {commandToSend.strip()}")

            # Wait and read response
            time.sleep(0.1)
            while arduino1.in_waiting:
                response = arduino1.readline().decode("utf-8").strip()
                rospy.loginfo(f"Arduino Response: {response}")

    except ValueError as e:
        rospy.logerr(f"Command parsing error: {e}")

# Main ROS node
if __name__ == "__main__":
    rospy.init_node("arduino_commander", anonymous=True)

    port = rospy.get_param("~serial_port", "/dev/ttyACM0")  # ~ makes it private to this node
    baudrate = 9600
    timeout = 2
    arduino1 = open_serial_connection(port, baudrate, timeout)

    if arduino1 and arduino1.is_open:
        rospy.loginfo(f"Serial connection established on {port}")

        # Read any initial message from Arduino
        initial_message = arduino1.readline().decode("utf-8").strip()
        rospy.loginfo(f"Arduino sent: {initial_message}")

        # Subscribe to `/arduino_command` topic to receive commands
        rospy.Subscriber("/arduino_command", String, command_callback)

        # Keep the node running
        rospy.spin()

    else:
        rospy.logerr("Failed to open serial connection.")
