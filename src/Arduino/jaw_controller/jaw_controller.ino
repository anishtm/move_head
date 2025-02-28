#include <ros.h>
#include <Servo.h>
#include <std_msgs/String.h>

const int32_t SAMPLE_DELAY_MS = 10;
const int SERVO_STEP_DELAY = 20;  // Non-blocking delay for servo movement
const int STEPS = 20;  // Steps for sinusoidal movement

ros::NodeHandle nh;

Servo servo2;
const int servo2Pin = 7;
int servo2Angle = 0;
bool triggerServo = false;  

// Timing Variables
unsigned long lastSensorReadTime = 0;
unsigned long lastServoUpdateTime = 0;
int servoStep = 0;
bool servoMovingUp = true;

// vairaibles for jaw delay.
bool waitingForProcessing = false;
unsigned long processingStartTime = 0;
const int PROCESSING_DELAY = 1500; // 1.5 seconds delay


// Callback for ROS subscriber
void voiceCallback(const std_msgs::String& msg) {
  //nh.loginfo("Message recieved");
  waitingForProcessing = true;
  processingStartTime = millis();

  if (strcmp(msg.data, "audio started") == 0) {
    //nh.loginfo("Jaw started");
    triggerServo = true;  // Start servos when "audio started" is received
  } 
  else if (strcmp(msg.data, "audio finished") == 0) {
    triggerServo = false;  // Stop servos when "audio finished" is received
    servo2.write(0);       // Return to initial position
    //nh.loginfo("Jaw stopped");
  }
}

// ROS Subscriber for voice commands
ros::Subscriber<std_msgs::String> voice_sub("/voice", &voiceCallback);


void setup() {
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(voice_sub);

  delay(1000);
  servo2.attach(servo2Pin);
  servo2.write(servo2Angle);


  nh.loginfo("Setup complete, waiting for messages...");
}

void loop() {
  unsigned long currentMillis = millis();

  // **2️ Non-blocking servo movement**

  if (triggerServo && currentMillis - lastServoUpdateTime >= SERVO_STEP_DELAY && waitingForProcessing && currentMillis - processingStartTime >= PROCESSING_DELAY) {
    lastServoUpdateTime = currentMillis;
    updateServoMovement();
  }


  nh.spinOnce();  // Process incoming messages
}

// **Non-blocking sinusoidal servo movement**
void updateServoMovement() {
  int maxAngle = random(70, 91);
  if (servoStep <= STEPS) {
      float t = (float)servoStep / STEPS;
      int angle = servoMovingUp 
        ? (0 + (maxAngle - 0) * (1 - cos(t * PI)) / 2)  // 0 to maxAngle
        : (maxAngle + (0 - maxAngle) * (1 - cos(t * PI)) / 2); // maxAngle to 0
      
      servo2.write(angle);

      // Move step forward or backward
      if (servoMovingUp) {
          servoStep++;  // Increase step when moving up
      } else {
          servoStep--;  // Decrease step when moving down
      }
  } else {
      servoMovingUp = !servoMovingUp; // Toggle direction

      if (servoMovingUp) {
          // When resetting to move up again, choose a new random maxAngle (between 20 and 90)
          maxAngle = random(20, 91);
          servoStep = 0;  // Reset step for next movement
      }
  } 

}
