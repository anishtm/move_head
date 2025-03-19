#ifndef JAW_MOVEMENT_H
#define JAW_MOVEMENT_H

#include <Servo.h>

#define JAW_SERVO 50
#define SERVO_STEP_DELAY 20
#define STEPS 20
#define PROCESSING_DELAY 1500

Servo jawServo;
bool triggerServo = false;
unsigned long lastServoUpdateTime = 0;
unsigned long processingStartTime = 0;
bool waitingForProcessing = false;
bool servoMovingUp = true;
int servoStep = 0;

void setupJawMovement(ros::NodeHandle& nh) {
    pinMode(JAW_SERVO, OUTPUT);
    jawServo.attach(JAW_SERVO);
    jawServo.write(0);
    nh.loginfo("Jaw movement setup complete.");
}


void updateServoMovement() {
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
            maxAngle = random(20, 91);
            servoStep = 0;
        }
    }
}

void updateJawMovement() {
    unsigned long currentMillis = millis();
    if (triggerServo && currentMillis - lastServoUpdateTime >= SERVO_STEP_DELAY && 
        waitingForProcessing && currentMillis - processingStartTime >= PROCESSING_DELAY) {
        
        lastServoUpdateTime = currentMillis;
        updateServoMovement();
    }
}

void processJawMovement(String msg) {
    waitingForProcessing = true;
    processingStartTime = millis();

    if (msg.equals("audio started")) {  // ✅ Use .equals() instead of strcmp()
        triggerServo = true;
    } else if (msg.equals("audio finished")) {
        triggerServo = false;
        jawServo.write(0);
    }
}

#endif
