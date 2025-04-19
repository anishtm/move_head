#include <ros.h>
#include <Servo.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>

const int32_t SAMPLE_DELAY_MS = 10;
const int SERVO_STEP_DELAY = 20;  // Non-blocking delay for servo movement
const int STEPS = 20;  // Steps for sinusoidal movement

ros::NodeHandle nh;

Servo jawServo;
Servo eyeServo;

const int jawServoPin = 13;
const int eyeServoPin = 11;

int jawServoAngle = 10;
int eyePositions[] = {90, 40, 90, 150, 90};  // Eye movement sequence
int currentEyeIndex = 0;
int eyeServoAngle = 90;

bool triggerServo = false;  
bool eyeMovementEnabled = true;
bool detectionActive = false;

// Timing Variables
unsigned long lastServoUpdateTime = 0;
unsigned long processingStartTime = 0;
unsigned long lastEyeMoveTime = 0;

const int PROCESSING_DELAY = 1500; // 1.5 seconds delay

int servoStep = 0;
bool servoMovingUp = true;
bool waitingForProcessing = false;

const int EYE_SEQUENCE_STEPS = sizeof(eyePositions) / sizeof(eyePositions[0]);
const int EYE_INTERVAL = 10000; // Total time for full sequence
const int EYE_STEP_INTERVAL = EYE_INTERVAL / EYE_SEQUENCE_STEPS;

unsigned long lastEyeStepTime = 0;


// Callback for ROS subscriber /voice
void voiceCallback(const std_msgs::String& msg) {
  waitingForProcessing = true;
  processingStartTime = millis();

  if (strcmp(msg.data, "audio started") == 0) {
    triggerServo = true;
    eyeMovementEnabled = false; // stop eye movement
  } 
  else if (strcmp(msg.data, "audio finished") == 0) {
    triggerServo = false;
    jawServo.write(jawServoAngle);
    eyeMovementEnabled = true;
  }
}

// Callback for ROS subscriber /detection
void detectionCallback(const std_msgs::Bool& msg) {
  detectionActive = msg.data;
  eyeServo.write(eyeServoAngle);
}

// ROS Subscribers
ros::Subscriber<std_msgs::String> voice_sub("/voice", &voiceCallback);
ros::Subscriber<std_msgs::Bool> detection_sub("/detection", &detectionCallback);

void setup() {
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(voice_sub);
  nh.subscribe(detection_sub);

  delay(1000);
  jawServo.attach(jawServoPin);
  eyeServo.attach(eyeServoPin);

  jawServo.write(jawServoAngle);
  eyeServo.write(eyeServoAngle);  // Resting pose
  nh.loginfo("Setup complete, waiting for messages...");
}

void loop() {
  unsigned long currentMillis = millis();

  // Handle jaw movement
  if (triggerServo && currentMillis - lastServoUpdateTime >= SERVO_STEP_DELAY && 
      waitingForProcessing && currentMillis - processingStartTime >= PROCESSING_DELAY) {
    lastServoUpdateTime = currentMillis;
    updateJawServo();
  }

  // Handle eye movement only if jaw is not active and detection is false
  if (!triggerServo && !detectionActive && eyeMovementEnabled &&
      currentMillis - lastEyeStepTime >= EYE_STEP_INTERVAL) {
    updateEyeServo();
    lastEyeStepTime = currentMillis;
  }

  nh.spinOnce();
}

// Smooth sinusoidal jaw motion
void updateJawServo() {
  int maxAngle = random(70, 91);
  if (servoStep <= STEPS) {
    float t = (float)servoStep / STEPS;
    int angle = servoMovingUp 
      ? (0 + (maxAngle - 0) * (1 - cos(t * PI)) / 2) 
      : (maxAngle + (0 - maxAngle) * (1 - cos(t * PI)) / 2);
    
    jawServo.write(angle);
    servoMovingUp ? servoStep++ : servoStep--;
  } else {
    servoMovingUp = !servoMovingUp;
    if (servoMovingUp) {
      servoStep = 0;
    }
  }
}

// Eye servo stepping through defined positions
void updateEyeServo() {
  currentEyeIndex = (currentEyeIndex + 1) % (sizeof(eyePositions) / sizeof(eyePositions[0]));
  eyeServo.write(eyePositions[currentEyeIndex]);
}
