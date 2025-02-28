#include <AccelStepper.h>
#include <Servo.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Point32.h>

#define SERVO_PIN 2
#define DIR_PIN 9
#define STEP_PIN 10
#define LEFT_LIMIT 3  
#define RIGHT_LIMIT 4  
#define ENABLE_PIN 12  

#define LEFT_END 0
#define RIGHT_END 4670
#define CENTER 2335
#define UPRIGHT_POSITION 84
#define UPPER_MAX 100
#define LOWER_MAX 64
#define STEPPER_SPEED 4000
#define STEPPER_ACCELERATION 8000
#define STEP_TOLERANCE 10
#define LIMIT_BACKSTEP 200
#define SMOOTHING_STEPS 50

AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);
Servo myServo; 
ros::NodeHandle nh;
std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);
geometry_msgs::Point32 latest_msg;

int current_angle = UPRIGHT_POSITION;
bool calibrated = false;

void messageCb(const geometry_msgs::Point32& msg) {
    latest_msg = msg;
}

ros::Subscriber<geometry_msgs::Point32> sub("commands", messageCb);


void sinusoidalSmoothing(int startAngle, int endAngle) {
    endAngle = constrain(endAngle, LOWER_MAX, UPPER_MAX);
    
    for (int step = 0; step <= SMOOTHING_STEPS; step++) {
        float t = (float)step / SMOOTHING_STEPS;
        int angle = startAngle + (endAngle - startAngle) * (1 - cos(t * PI)) / 2;
        myServo.write(angle);
        delay(10);
    }
    current_angle = endAngle;
}

void move_to_position(int target_position) {
    limit_checker();
    stepper.moveTo(target_position);
    while (stepper.distanceToGo() != 0) {
        stepper.run();
        //nh.spinOnce();
        limit_checker();
    }
}

void move(bool move_to_right, int step_size) {
    int step = move_to_right ? step_size : -step_size;
    int limit = move_to_right ? RIGHT_LIMIT : LEFT_LIMIT;

    while (digitalRead(limit) == HIGH) {
        stepper.move(step);
        stepper.run();
    }
}

void syncMovement(int stepperTarget, int servoTarget) {
    int stepperStart = stepper.currentPosition();
    int servoStart = myServo.read();
    stepperTarget  = constrain(stepperTarget, LEFT_END, RIGHT_END);
    servoTarget = constrain(servoTarget, LOWER_MAX, UPPER_MAX);

    int numStepperSteps = abs(stepperTarget - stepperStart);
    int numServoSteps = abs(servoTarget - servoStart);

    if (numStepperSteps == 0 && numServoSteps == 0) return;  // No movement needed

    if (numStepperSteps == 0) {
      sinusoidalSmoothing(current_angle, servoTarget);
      return;
    }

    float durationMs = numStepperSteps * 0.30;  // Estimated total movement time
    float stepDelay = numStepperSteps > 0 ? durationMs / numStepperSteps : 1;
    float servoDelay = numServoSteps > 0 ? durationMs / numServoSteps : 1;

    int servoPos = servoStart;
    unsigned long lastServoTime = millis();

    stepper.moveTo(stepperTarget);  // Set target position

    while (stepper.distanceToGo() != 0 || servoPos != servoTarget) {
        unsigned long currentTime = millis();
        if (currentTime - lastServoTime >= servoDelay && servoPos != servoTarget) {
            servoPos += (servoTarget > servoStart) ? 1 : -1;
            myServo.write(servoPos);
            lastServoTime = millis();
        }

        stepper.run(); 
        limit_checker();
    }
    current_angle = servoTarget;
}

void limit_checker() {
  int current_position = stepper.currentPosition();

  if (digitalRead(LEFT_LIMIT) == LOW) {
    if (abs(current_position - LEFT_LIMIT) > 10) {
      calibration();
    }
    stepper.move(200); 
    while (stepper.distanceToGo() != 0) {
      stepper.run();
    }
  } 
  else if (digitalRead(RIGHT_LIMIT) == LOW) {
    if (abs(current_position - RIGHT_LIMIT) > 10) {
      calibration();
    }
    stepper.move(-200); 
    while (stepper.distanceToGo() != 0) {
      stepper.run();
    }
  }
}

void calibration() {
    myServo.write(UPRIGHT_POSITION);
    delay(1000);

    move(false, 100);
    stepper.setCurrentPosition(LEFT_END);
    delay(500);
    move_to_position(40);

    move(false, 100);
    stepper.setCurrentPosition(LEFT_END);
    move_to_position(CENTER);
}

void setup() {
    nh.getHardware()->setBaud(115200);
    nh.initNode();
    nh.advertise(chatter);
    nh.subscribe(sub);

    pinMode(LEFT_LIMIT, INPUT_PULLUP);
    pinMode(RIGHT_LIMIT, INPUT_PULLUP);
    pinMode(SERVO_PIN, OUTPUT);

    myServo.attach(SERVO_PIN);
    stepper.setMaxSpeed(STEPPER_SPEED);
    stepper.setAcceleration(STEPPER_ACCELERATION);
    stepper.setSpeed(STEPPER_SPEED);

    latest_msg.x = -1;
}

void loop() {
    nh.spinOnce();
    if (!calibrated) {
        calibration();
        calibrated = true;
    }

    if (latest_msg.x != -1){
      process_msg(latest_msg);
    }
    delay(1);
}

void process_msg(const geometry_msgs::Point32& msg){
    uint8_t type = int(msg.z);
    int16_t position = int(msg.x);
    uint8_t angle = int(msg.y);
    char buffer[30];

 
    sprintf(buffer, "Type: %d, Pos: %d servo: %d", type, position, angle);
    str_msg.data = buffer;
    switch (type) {
        case 0:  
            calibration();
           // chatter.publish(&str_msg);
            break;
        case 1:  
            // center
            syncMovement(CENTER, UPRIGHT_POSITION);
            break;
        case 2:  
            // servo
            sinusoidalSmoothing(current_angle, angle);
            break;
        case 3:  
            // stepper
            syncMovement(position, angle);
            break;
        case 4:  
            // both & together
            syncMovement(position, angle);
            break;
        case 5:  
            // both but one by one
            sinusoidalSmoothing(current_angle, angle);
            move_to_position(position);
            break;
        default:
            nh.loginfo("Invalid command!");
            return;
    }      
    chatter.publish(&str_msg);
    latest_msg.x = -1;  // Reset to default values

}
