#!/usr/bin/env python3

import cv2
import os
import sys
import rospy
import time
import numpy as np
from sort import Sort
from geometry_msgs.msg import Point32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

sys.path.append(os.path.join(os.path.dirname(__file__), 'yunet'))

script_dir = os.path.dirname(os.path.abspath(__file__))
MODEL_WEIGHTS = os.path.join(script_dir, "yunet", "face_detection_yunet_2023mar.onnx")

from yunet import YuNet


class FaceTrackingNode:
    def __init__(self):
        rospy.init_node('face_tracking_node')

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/image", Image, self.image_callback)
        self.command_pub = rospy.Publisher('/commands', Point32, queue_size=1)

        self.yunet = YuNet(
            modelPath=MODEL_WEIGHTS,
            inputSize=(320, 320),
            confThreshold=0.7,
            nmsThreshold=0.3,
            topK=8
        )

        self.tracker = Sort()
        self.imp_face_id = None
        self.imp_face_area = 0
        self.last_detection_time = time.time()
        self.no_face_timeout = 30  # Timeout in seconds
        self.latest_frame = None
        self.new_frame_available = False

    def image_callback(self, msg):
        self.latest_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        self.new_frame_available = True

    def process_frame(self, frame):
        height, width, _ = frame.shape
        self.yunet.setInputSize((width, height))

        faces = self.yunet.infer(frame)
        detections = []

        if faces is not None:
            for face in faces:
                x1, y1, w, h = face[:4]
                score = face[-1]
                x2, y2 = x1 + w, y1 + h
                detections.append([x1, y1, x2, y2, score])

        detections = np.array(detections) if detections else np.empty((0, 5))
        tracked_objects = self.tracker.update(detections)

        tracked_id_list = tracked_objects[:, -1] if len(tracked_objects) > 0 else []

        if self.imp_face_id not in tracked_id_list:
            self.imp_face_id = None
            self.imp_face_area = 0

        if self.imp_face_id is None:
            largest_face_id = None
            largest_face_area = 0

            for obj in tracked_objects:
                x1, y1, x2, y2, track_id = map(int, obj)
                face_area = (x2 - x1) * (y2 - y1)

                if face_area > largest_face_area:
                    largest_face_area = face_area
                    largest_face_id = track_id

            self.imp_face_id = largest_face_id

        if len(tracked_objects) > 0:
            self.last_detection_time = time.time()

        for obj in tracked_objects:
            x1, y1, x2, y2, track_id = map(int, obj)
            center_x = (x1 + x2) / 2
            center_y = (y1 + y2) / 2
            stepper_pos = int(500 + (center_x / width) * 3700)
            servo_angle = int(70 + (center_y / height) * 30.0)

            if track_id == self.imp_face_id:
                color = (0, 255, 0)
                msg = Point32()
                msg.x = stepper_pos
                msg.y = servo_angle
                msg.z = 4
                self.command_pub.publish(msg)
            else:
                color = (255, 255, 255)

            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
            cv2.putText(frame, f'ID {track_id}', (x1, y1 - 10), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

            if track_id == self.imp_face_id:
                cv2.putText(frame, f'pos: {stepper_pos}, ang: {servo_angle}', (10, 50), 
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 2)

        if time.time() - self.last_detection_time > self.no_face_timeout:
            msg = Point32()
            msg.x = 2335
            msg.y = 84
            msg.z = 1
            self.command_pub.publish(msg)

            cv2.putText(frame, "No face detected! Moving to default position.", (50, 100),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

            self.last_detection_time = time.time()

        cv2.imshow("SORT + YuNet", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            rospy.signal_shutdown("User Quit")

    def run(self):
        rate = rospy.Rate(30)  
        while not rospy.is_shutdown():
            if self.new_frame_available and self.latest_frame is not None:
                self.process_frame(self.latest_frame)
                self.new_frame_available = False
            rate.sleep()



if __name__ == "__main__":
    node = FaceTrackingNode()
    node.run()
