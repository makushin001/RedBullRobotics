// WORKING VERSION INITIAL
#include <Servo.h>  // Include the Servo library

// Motor Pins
#define m1 7   // Right Motor MA1
#define m2 8   // Right Motor MA2
#define m3 12  // Left Motor MB1
#define m4 11  // Left Motor MB2
#define e1 3  // Right Motor Enable Pin EA this changed from 9 to 3
#define e2 6  // Left Motor Enable Pin EB

// 5 Channel IR Sensor Connection
#define ir1 A0
#define ir2 A1
#define ir3 A2
#define ir4 A3
#define ir5 A4

// Ultrasonic Sensor Pins
#define pingPin 4  // Trigger pin of ultrasonic sensor
#define echoPin 5  // Echo pin of ultrasonic sensor

// Servo Configuration
Servo myServo;
const int servo_pin = 13;  // Servo connected to pin 13

const float DISTANCE_THRESHOLD = 20.0;  // Distance threshold for obstacle detection in cm

void setup() {
  // Initialize motor control pins
  pinMode(m1, OUTPUT);
  pinMode(m2, OUTPUT);
  pinMode(m3, OUTPUT);
  pinMode(m4, OUTPUT);
  pinMode(e1, OUTPUT);
  pinMode(e2, OUTPUT);

  // Initialize IR sensor pins
  pinMode(ir1, INPUT);
  pinMode(ir2, INPUT);
  pinMode(ir3, INPUT);
  pinMode(ir4, INPUT);
  pinMode(ir5, INPUT);

  // Initialize ultrasonic sensor pins
  pinMode(pingPin, OUTPUT);  // Trigger pin
  pinMode(echoPin, INPUT);   // Echo pin

  // Attach servo to the servo object
  myServo.attach(servo_pin);
  myServo.write(90);  // Start with the servo facing forward

  Serial.begin(9600);  // Start serial communication for debugging
}

void loop() {
  // Reading the distance from the ultrasonic sensor
  float distance = distance_cm();

  // Checking if the distance is less than the desired distance threshold
  if (distance < DISTANCE_THRESHOLD) {
    // Starting obstacle avoidance procedure 
    stop_motors();
    avoid_obstacle();
  } else {
    // Starting line following procedure
    follow_line();
  }
}

void follow_line() {
  const int PROPORTIONAL_GAIN = 35;
  // Reading sensor values (inverted if necessary)
  int s1 = digitalRead(ir1);
  int s2 = digitalRead(ir2);
  int s3 = digitalRead(ir3);
  int s4 = digitalRead(ir4);
  int s5 = digitalRead(ir5);

  int skewness = 0;
  if (s1 == 0) skewness += -3;
  if (s2 == 0) skewness += -1.5;
  if (s3 == 0) skewness += 0;
  if (s4 == 0) skewness += 1.5;
  if (s5 == 0) skewness += 3;

  int base_speed = 120;
  int adjustment_speed = skewness * PROPORTIONAL_GAIN;

  int left_speed = constrain(base_speed + adjustment_speed, 0, 255);
  int right_speed = constrain(base_speed - adjustment_speed, 0, 255);

  move_forward(left_speed, right_speed);
  delay(5);  // Reduced delay for quicker response
}



void avoid_obstacle() {
  // Initialising constants
  const float PROPORTIONAL_GAIN = 5.0;
  const float DESIRED_DISTANCE = 20.0;
  const int BASE_SPEED = 120;
  const int MAX_ADJUSTMENT = 100;
  const int DELAY_ON_TURN_TO_THE_RIGHT = 100;
  const int ULTRASONIC_POSITION_FORWARD = 90;
  const int ULTRASONIC_POSITION_LEFT = 150;
  
  // Making sure that ultrasonic sensor is pointing forward
  myServo.write(ULTRASONIC_POSITION_FORWARD);
  delay(500);

  // Step 1: Turning right until obstacle is no longer detected in front
  while (distance_cm() < (DISTANCE_THRESHOLD + 10)) {
    move_right(150, 150);
    delay(DELAY_ON_TURN_TO_THE_RIGHT);
  }

  // Step 2: Stopping the robot
  stop_motors();
  delay(200);

  // Step 3: Turning the ultrasonic sensor to the left to scan the obstacle and wait for servo to reach position
  myServo.write(ULTRASONIC_POSITION_LEFT);
  delay(500);


  // Step 4: Following the obstacle, maintaining safe distance, until the line is found again
  while (!is_line_detected()) {
    // Measuring side distance
    float side_distance = distance_cm();

    // Computing distance away from the desired distance (DESIRED_DISTANCE is 20.0 cm)
    float away_from_desired_distance_in_cm = side_distance - DESIRED_DISTANCE;  // Positive if too far, negative if too close

    // Computing adjustment
    int adjustment_speed = PROPORTIONAL_GAIN * away_from_desired_distance_in_cm;

    // Limiting adjustment to avoid exceeding PWM limits
    adjustment_speed = constrain(adjustment_speed, -MAX_ADJUSTMENT, MAX_ADJUSTMENT);

    // Adjusting motor speeds based on the distance
    int speed_left = BASE_SPEED - adjustment_speed;   // Adjust left motor speed
    int speed_right = BASE_SPEED + adjustment_speed;  // Adjust right motor speed

    // Limiting motor speeds to valid PWM range from 0 to 255
    speed_left = constrain(speed_left, 0, 255);
    speed_right = constrain(speed_right, 0, 255);

    // Moving forward in the adjusted direction 
    move_forward(speed_left, speed_right);
    delay(50);
  }

  // Stop the motors
  stop_motors();
  delay(500);
  move_right(175, 175);
  delay(30);
  myServo.write(ULTRASONIC_POSITION_FORWARD);
  delay(500);
}



bool is_line_detected() {
  // Read IR sensors
  int s1 = digitalRead(ir1);
  int s2 = digitalRead(ir2);
  int s3 = digitalRead(ir3);
  int s4 = digitalRead(ir4);
  int s5 = digitalRead(ir5);

  int line_count = 0;
  if (s1 == 0) line_count++;
  if (s2 == 0) line_count++;
  if (s3 == 0) line_count++;
  if (s4 == 0) line_count++;
  if (s5 == 0) line_count++;

  return (line_count >= 2); // Adjust the threshold as needed
}

void stop_motors() {
  digitalWrite(m1, LOW);
  digitalWrite(m2, LOW);
  digitalWrite(m3, LOW);
  digitalWrite(m4, LOW);
  analogWrite(e1, 0);
  analogWrite(e2, 0);
}

void move_forward(int speed_left, int speed_right) {
  analogWrite(e1, speed_right);
  analogWrite(e2, speed_left);
  digitalWrite(m1, HIGH);
  digitalWrite(m2, LOW);
  digitalWrite(m3, HIGH);
  digitalWrite(m4, LOW);
}

void move_left(int speed_left, int speed_right) {
  analogWrite(e1, speed_right);
  analogWrite(e2, speed_left);
  digitalWrite(m1, LOW);
  digitalWrite(m2, HIGH);
  digitalWrite(m3, HIGH);
  digitalWrite(m4, LOW);
}

void move_right(int speed_left, int speed_right) {
  analogWrite(e1, speed_right);
  analogWrite(e2, speed_left);
  digitalWrite(m1, HIGH);
  digitalWrite(m2, LOW);
  digitalWrite(m3, LOW);
  digitalWrite(m4, HIGH);
}


// Function to measure average distnace to an object in centimeters
float distance_cm() {
  const int num_readings = 5;
  float total_distance = 0;

  // Take multiple readings and average them to improve accuracy
  for (int i = 0; i < num_readings; i++) {
    total_distance += single_distance_read();
    delay(10);  // Short delay between readings
  }
  return total_distance / num_readings;
}

// Function to measure ditance upto an obsticle
float single_distance_read() {
  // Declaring wait time and distance variables
  long duration;
  float distance;

  // Triggering the ultrasonic sensor
  digitalWrite(pingPin, LOW);
  delayMicroseconds(2);
  digitalWrite(pingPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(pingPin, LOW);

  // Reading the echo time
  duration = pulseIn(echoPin, HIGH, 30000);  // Timeout after 30ms

  // Calculating distance
  distance = (duration / 2.0) * 0.0343;  // Speed of sound is 343 m/s

  // Capping the distance measurement at 400 cm if either duration is 0 or distance larger than 400 cm
  if (duration == 0 || distance >= 400) {
    distance = 400;
  }
  return distance;
}


