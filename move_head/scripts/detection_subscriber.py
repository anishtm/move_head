#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from move_head.msg import Face, FaceDetection

nodeName = 'messageVisualizer'
topicName = '/Detections'

# Scaling factor for coordinates and canvas size
SCALE_FACTOR = 3

# Callback function for incoming messages
def callBackFunction(message):
    rospy.loginfo(f"Received FaceDetection message with {message.num_faces} faces.")
    
    # Create a blank canvas with scaled size (4x larger than 256x256)
    canvas = np.zeros((256 * SCALE_FACTOR, 256 * SCALE_FACTOR, 3), dtype=np.uint8)
    
    # Draw the outer frame (rectangle) from (0, 0) to (255 * SCALE_FACTOR, 255 * SCALE_FACTOR)
    cv2.rectangle(canvas, (0, 0), (256 * SCALE_FACTOR, 256 * SCALE_FACTOR), (255, 255, 255), thickness=1)

    max_area = 0
    max_area_face = None
    color = (0, 128, 0) if message.num_faces < 2 else (255, 255, 255)

    # Iterate through each detected face and draw the bounding box
    for face in message.faces:
        try:
            # Scale the coordinates by the scaling factor
            x1 = face.box_left * SCALE_FACTOR
            y1 = face.box_top * SCALE_FACTOR
            x2 = face.box_right * SCALE_FACTOR
            y2 = face.box_bottom * SCALE_FACTOR
            box_confidence = face.box_confidence

            # Calculate the area of the bounding box
            width = x2 - x1
            height = y2 - y1
            area = width * height

            # Determine if this is the largest box
            if area > max_area:
                max_area = area
                max_area_face = face

            # Define the coordinates for the rectangle (scaled)
            small_box_top_left = (x1, y1)
            small_box_bottom_right = (x2, y2)

            # Set the color of the rectangle based on whether it's the largest
            if message.num_faces < 2:
                color = (0, 128, 0)  # Green for less than 2 faces
            else:
                color = (0, 0, 255) if face == max_area_face else (255, 255, 255) 

            # Draw the rectangle on the canvas (scaled)
            cv2.rectangle(canvas, small_box_top_left, small_box_bottom_right, color, thickness=2)

            # Display the confidence score in YOLO format
            confidence_text = f"{box_confidence}"
            font = cv2.FONT_HERSHEY_SIMPLEX
            # Place confidence text at the top-left corner of the bounding box
            text_size, _ = cv2.getTextSize(confidence_text, font, 0.5, 1)
            text_width, text_height = text_size
            background_top_left = (x1, y1 - 10 - text_height)
            background_bottom_right = (x1 + text_width, y1 - 10)
            cv2.rectangle(canvas, background_top_left, background_bottom_right, (255, 255, 255), -1)  # White background

            # Display the confidence score with black text
            cv2.putText(canvas, confidence_text, (x1, y1 - 10), font, 0.5, (0, 0, 0), 1, cv2.LINE_AA)  # Black text


        except Exception as e:
            rospy.logwarn(f"Error while drawing bounding box: {e}")

    # Display the canvas
    cv2.imshow('Face Detection Canvas', canvas)
    cv2.waitKey(1)  # Non-blocking key press to allow continuous updates

if __name__ == '__main__':
    rospy.init_node(nodeName, anonymous=True)
    rospy.loginfo("Node started")
    
    # Subscribe to the /Detections topic
    rospy.Subscriber(topicName, FaceDetection, callback=callBackFunction)

    rospy.on_shutdown(cv2.destroyAllWindows)

    rospy.spin()


