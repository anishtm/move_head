#!/usr/bin/env python3

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def publish_image():
    rospy.init_node('image_publisher', anonymous=True)
    image_pub = rospy.Publisher('/camera/image', Image, queue_size=10)
    bridge = CvBridge()

    cap = cv2.VideoCapture(0)  # Open webcam (change to video file path if needed)

    rate = rospy.Rate(30)  # Publish at 10 Hz

    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if not ret:
            rospy.logerr("Failed to capture image")
            continue
        
        ros_image = bridge.cv2_to_imgmsg(frame, encoding="bgr8")  # Convert OpenCV image to ROS image
        image_pub.publish(ros_image)
        #rospy.loginfo("Published an image")
        
        rate.sleep()

    cap.release()

if __name__ == '__main__':
    try:
        publish_image()
    except rospy.ROSInterruptException:
        pass
