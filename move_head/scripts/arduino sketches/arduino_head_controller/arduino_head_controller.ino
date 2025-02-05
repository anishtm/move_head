#include <AccelStepper.h>
#include <Servo.h>

#define SERVO_PIN 2
#define dirPin 9
#define stepPin 10
#define left_limit 3   // Left homing switch
#define right_limit 4  // Right homing switch
#define enable 12



AccelStepper stepper(AccelStepper::DRIVER, stepPin, dirPin);
Servo myServo; 

int left_end = 0;     // Default value if not calibrated
int right_end = 4670;  // Default value
int center = 2335;     // Default value
int upright_position = 84;
int upper_max = 100;
int lower_max = 64;
int steps = 50;
int current_angle = upright_position; // Always start with this angle if not specified
int current_position;


int tilt_straight() {
  myServo.write(upright_position);
  Serial.println("In upright position");
  return upright_position;
}

void move(bool move_to_right, int step_size) {
  int step = move_to_right ? step_size : -step_size;  // Move direction
  const char* pos = move_to_right ? "Right" : "Left";  // Direction string
  int limit = move_to_right ? right_limit : left_limit;  // Limit switch to check

  Serial.print("Moving ");
  Serial.print(pos);
  Serial.println(" ...");

  while (digitalRead(limit) == HIGH) {  // Continue moving until switch is pressed
    stepper.move(step);
    stepper.run();
  }

  if (digitalRead(limit) == LOW) {
    Serial.print(pos);
    Serial.println(" limit reached. Resetting position...");
  }
}

void limit_checker(){
  if (digitalRead(left_limit) == LOW) {
    Serial.println("At left limit! Moving slightly right...");
    stepper.move(200);  // Move slightly away from the left limit
    while (stepper.distanceToGo() != 0) {
      stepper.run();
    }
  } else if (digitalRead(right_limit) == LOW) {
    Serial.println("At right limit! Moving slightly left...");
    stepper.move(-200);  // Move slightly away from the right limit
    while (stepper.distanceToGo() != 0) {
      stepper.run();
    }
  }
  return ;
}

void move_to_position(int target_position) {
  Serial.print("Moving to position: ");
  Serial.println(target_position);

  limit_checker();

  // Now move to the desired position
  stepper.moveTo(target_position);
  while (stepper.distanceToGo() != 0) {
    stepper.run();

    // If a limit switch is triggered, move away slightly before stopping
    limit_checker();
  }

  Serial.println("Position " + String(target_position) + " reached.");
}
//int &left_end, int &right_end, int &center
void calibration() {

  current_angle = tilt_straight();
  delay(2000);
  Serial.print("At angle ");
  Serial.println(current_angle);

  Serial.println("Homing: Moving left to find the switch...");
  move(false, 100);  // Move left

  stepper.setCurrentPosition(0);  // Set zero position at home
  delay(500);
  move_to_position(40);

  move(false, 100);
  left_end = stepper.currentPosition();

  move(true, 100);  // Move right
  right_end = stepper.currentPosition();  // Record right end position
  Serial.print("Right end position: ");
  Serial.println(right_end);
  delay(1000);
    
  Serial.print("Left: ");
  Serial.print(left_end);
  Serial.print(", Right: ");
  Serial.println(right_end);

  center = (left_end + right_end) / 2;
  Serial.print("Center position: ");
  Serial.println(center);

  move_to_position(center);

  Serial.println("Calibration Complete!");
  
}

void move_center(){
  tilt_straight();
  move_to_position(center);
}


void sinusoidalSmoothing(int startAngle, int endAngle, int steps) {
  for (int step = 0; step <= steps; step++) {
    float t = (float)step / steps;
    int angle = startAngle + (endAngle - startAngle) * (1 - cos(t * PI)) / 2; // Sinusoidal easing
    myServo.write(angle);
    delay(20); // Adjust delay for speed control
  }
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);      // Set the built-in LED pin as output
  
  Serial.begin(9600);
  Serial.println("Arduino is ready!"); // Debug message to indicate readiness
  Serial.println("Bnabana!");
  pinMode(left_limit, INPUT_PULLUP);
  pinMode(right_limit, INPUT_PULLUP);
  myServo.attach(SERVO_PIN);


  

  stepper.setMaxSpeed(8000);
  stepper.setAcceleration(8000);
  //calibration();
}

void loop(){
  if (Serial.available() > 0) {

    String command = Serial.readStringUntil('\n');
    command.trim();
    Serial.print("Received command: ");
    Serial.println(command);
    String type = command.substring(0, 3); // Extracts 'MOV' or 'CAL'

    if (type.equalsIgnoreCase("POS")) {
        // Extract the numeric part of the command
      String numPart = command.substring(3); // Extracts '003002' or '090003'

      // Split and convert to integers
      int position = numPart.substring(0, 4).toInt(); 

      move_to_position(position);

      Serial.println("Done.");
    } else if (type.equalsIgnoreCase("CAL")) {

      Serial.println("Calibrating the head...");

      calibration(); //left_end, right_end, center

      Serial.print("Left End: ");
      Serial.print(left_end);
      Serial.print(", Right End: ");
      Serial.print(right_end);
      Serial.print(", Center: ");
      Serial.println(center);

    } else if (type.equalsIgnoreCase("CEN")) {

      Serial.println("Centering the head...");
      move_center();


    } else {
        // Handle invalid commands
        Serial.println("Invalid command type!");
    }
  }
}
