#include <Wire.h>
#include "person_sensor.h"
#include <ros.h>
#include <move_head/Face.h>

const int32_t SAMPLE_DELAY_MS = 200;

ros::NodeHandle nh;
move_head::Face largest_face;
ros::Publisher face_publish("face_detection", &largest_face);

void setup() {
  Wire.begin();  // Initialize I2C
  nh.initNode();  // Initialize ROS node
  nh.advertise(face_publish);  // Advertise the topic
}

void loop() {
  person_sensor_results_t results = {};
  int largest_face_index = -1;
  float largest_area = 0;

  // Read face detection data
  if (!person_sensor_read(&results)) {
    nh.logwarn("Failed to read from person sensor");
    delay(SAMPLE_DELAY_MS);
    return;
  }

  // Iterate through detected faces and find the largest one
  for (int i = 0; i < results.num_faces; ++i) {
    const person_sensor_face_t* face = &results.faces[i];
    float length = face->box_right - face->box_left;
    float height = face->box_bottom - face->box_top;
    float area = length * height;

    if (area > largest_area) {
      largest_area = area;
      largest_face_index = i;
    }
  }

  // If a largest face is found, populate and publish it
  if (largest_face_index != -1) {
    const person_sensor_face_t* face = &results.faces[largest_face_index];

    // Log the number of detected faces
    char log_msg[50];
    sprintf(log_msg, "%d faces detected", results.num_faces);
    nh.loginfo(log_msg);

    // Populate largest face message
    largest_face.face_detected = "face_detected";
    largest_face.index = largest_face_index;
    largest_face.box_left = face->box_left;
    largest_face.box_top = face->box_top;
    largest_face.box_right = face->box_right;
    largest_face.box_bottom = face->box_bottom;
    largest_face.box_confidence = face->box_confidence;
    largest_face.id_confidence = face->id_confidence;
    largest_face.id = face->id;
    largest_face.is_facing = face->is_facing;

    // Publish the largest face
    face_publish.publish(&largest_face);
  } else {
    nh.loginfo("No faces detected");
  }

  nh.spinOnce();
  delay(SAMPLE_DELAY_MS);
}
