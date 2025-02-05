#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from move_head.msg import Face
# from std_msgs.msg import Int32MultiArray

# Define canvas dimensions
CANVAS_WIDTH = 1636  # Mapped from 1351 to 2987
CANVAS_HEIGHT = 36 * 3  # Changed from 32 to 36 and multiplied by 3

# Horizontal mapping (256 -> 0 to 1636)
HORIZONTAL_MIN = 0
HORIZONTAL_MAX = 1636

# Vertical mapping (256 -> 108 pixels)
VERTICAL_MIN = 0
VERTICAL_MAX = 108

# Callback function
def face_callback(msg):
    # Flip horizontally (255 - x)
    flipped_left = 255 - msg.box_right
    flipped_right = 255 - msg.box_left
    
    # Calculate center
    center_x = (flipped_left + flipped_right) // 2
    center_y = (msg.box_top + msg.box_bottom) // 2
    
    # Map coordinates to the new canvas range
    mapped_x = int(np.interp(center_x, [0, 255], [HORIZONTAL_MIN, HORIZONTAL_MAX]))
    mapped_y = int(np.interp(center_y, [0, 255], [VERTICAL_MIN, VERTICAL_MAX]))
    
    # Publish the center
    
    
    # Draw canvas
    canvas = np.zeros((CANVAS_HEIGHT, CANVAS_WIDTH, 3), dtype=np.uint8)
    cv2.circle(canvas, (mapped_x, mapped_y), 2, (0, 255, 0), -1)
    
    # Show image
    cv2.imshow('Face Center Visualization', canvas)
    cv2.waitKey(1)

if __name__ == '__main__':
    rospy.init_node('face_mapper', anonymous=True)
    rospy.Subscriber('/face_detection', Face, face_callback)
    rospy.on_shutdown(cv2.destroyAllWindows)
    rospy.spin()

