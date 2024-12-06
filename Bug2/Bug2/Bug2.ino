#include <Servo.h>
// Infrared sensors
#define ir1 A0
#define ir2 A1
#define ir3 A2
#define ir4 A3
#define ir5 A4

// Motor pins
#define speedPinR 3   
#define speedPinL 6    
#define RightMotorDirPin1  12    
#define RightMotorDirPin2  11   
#define LeftMotorDirPin1  7    
#define LeftMotorDirPin2  8  

// Ultrasonic Sensor Pins
#define Trig_PIN 4   
#define Echo_PIN 5    

// Servo motor 
#define servo_pin 13   
Servo myservo;

const int closestdistance = 20;
const int turningTime = 300;
const int D0 = 20;

void setup() {
  // put your setup code here, to run once:

  pinMode(ir1, INPUT);
  pinMode(ir2, INPUT);
  pinMode(ir3, INPUT);
  pinMode(ir4, INPUT);
  pinMode(ir5, INPUT);

  pinMode(RightMotorDirPin1, OUTPUT); 
	pinMode(RightMotorDirPin2, OUTPUT); 
	pinMode(speedPinL, OUTPUT);  
  pinMode(LeftMotorDirPin1, OUTPUT);
  pinMode(LeftMotorDirPin2, OUTPUT); 
  pinMode(speedPinR, OUTPUT); 

  pinMode(Trig_PIN, OUTPUT);
  pinMode(Echo_PIN, INPUT);
  
  myservo.attach(servo_pin);
  myservo.write(70);
}

// Function to measure distance using ultrasonic sensor
int getDistance() {
  // Trigger the ultrasonic sensor
  digitalWrite(Trig_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(Trig_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(Trig_PIN, LOW);

  // Measure the duration of the echo pulse
  long duration = pulseIn(Echo_PIN, HIGH);

  // Calculate the distance in centimeters
  int distance = duration * 0.034 / 2;

  return distance;
}

bool foundLine() {
  //Reading Sensor Values
  int LeftLeft = digitalRead(ir1);  //Left Most Sensor
  int Left = digitalRead(ir2);  //Left Sensor
  int middle = digitalRead(ir3);  //Middle Sensor
  int Right = digitalRead(ir4);  //Right Sensor
  int RightRight = digitalRead(ir5);  //Right Most Sensor

  return (LeftLeft == 0 || Left == 0 || middle == 0 || Right == 0 || RightRight == 0);
}

void motion(int speedR, int speedL, int Left1, int Left2, int Right1, int Right2) {
    analogWrite(speedPinR, speedR); //you can adjust the speed of the motors from 0-255
    analogWrite(speedPinL, speedL); //you can adjust the speed of the motors from 0-255
    digitalWrite(LeftMotorDirPin1, Left1);
    digitalWrite(LeftMotorDirPin2, Left2);
    digitalWrite(RightMotorDirPin1, Right1);
    digitalWrite(RightMotorDirPin2, Right2);
}

void lineFollowing(){
  myservo.write(70);
  //Reading Sensor Values
  int LeftLeft = digitalRead(ir1);  //Left Most Sensor
  int Left = digitalRead(ir2);  //Left Sensor
  int middle = digitalRead(ir3);  //Middle Sensor
  int Right = digitalRead(ir4);  //Right Sensor
  int RightRight = digitalRead(ir5);  //Right Most Sensor

  //if only middle sensor detects black line
  if((LeftLeft == 1) && (Left == 1) && (middle == 0) && (Right == 1) && (RightRight == 1))
  {
    //going forward  
    motion(120, 120, HIGH, LOW, HIGH, LOW);
  }
  
  //if only left sensor detects black line
  if((LeftLeft == 1) && (Left == 0) && (middle == 1) && (Right == 1) && (RightRight == 1))
  {
    //going left
    motion(150, 150, LOW, LOW, HIGH, LOW);
  }
  
  //if only left most sensor detects black line
  if((LeftLeft == 0) && (Left == 1) && (middle == 1) && (Right == 1) && (RightRight == 1))
  {
    //going left with full speed 
    motion(180, 180, LOW, HIGH, HIGH, LOW);
  }

  //if only right sensor detects black line
  if((LeftLeft == 1) && (Left == 1) && (middle == 1) && (Right == 0) && (RightRight == 1))
  {
    //going right
    motion(150, 150, HIGH, LOW, LOW, LOW);
  }

  //if only right most sensor detects black line
  if((LeftLeft == 1) && (Left == 1) && (middle == 1) && (Right == 1) && (RightRight == 0))
  {
    //going right with full speed 
    motion(180, 180, HIGH, LOW, LOW, HIGH);
  }

  //if middle and right sensor detects black line
  if((LeftLeft == 1) && (Left == 1) && (middle == 0) && (Right == 0) && (RightRight == 1))
  {
    //going right
    motion(150, 150, HIGH, LOW, LOW, LOW);
  }

  //if middle and left sensor detects black line
  if((LeftLeft == 1) && (Left == 0) && (middle == 0) && (Right == 1) && (RightRight == 1))
  {
    //going left
    motion(150, 150, LOW, LOW, HIGH, LOW);
  }

  //if middle, left and left most sensor detects black line
  if((LeftLeft == 0) && (Left == 0) && (middle == 0) && (Right == 1) && (RightRight == 1))
  {
    //going left
    motion(150, 150, LOW, LOW, HIGH, LOW);
  }


  //if middle, right and right most sensor detects black line
  if((LeftLeft == 1) && (Left == 1) && (middle == 0) && (Right == 0) && (RightRight == 0))
  {
    //going right
    motion(150, 150, HIGH, LOW, LOW, LOW);
  }

  //if left and left most sensor detects black line
  if((LeftLeft == 0) && (Left == 0) && (middle == 1) && (Right == 1) && (RightRight == 1))
  {
    //going left
    motion(150, 150, LOW, LOW, HIGH, LOW);
  }

  //if right and right most sensor detects black line
  if((LeftLeft == 1) && (Left == 1) && (middle == 1) && (Right == 0) && (RightRight == 0))
  {
    //going right
    motion(150, 150, HIGH, LOW, LOW, LOW);
  }
  
  //if all sensors are on a black line
  if((LeftLeft == 0) && (Left == 0) && (middle == 0) && (Right == 0) && (RightRight == 0))
  {
    //forward
    motion(120, 120, HIGH, LOW, HIGH, LOW);

    //going right
    motion(150, 150, HIGH, LOW, LOW, LOW);

    //going left
    motion(150, 150, LOW, LOW, HIGH, LOW);
  }

}

void wallFollowing() {
  int distance = getDistance();

  if (distance < closestdistance) {
      // Stop 
      motion(0, 0, LOW, LOW, LOW, LOW);
      
      while (getDistance() < closestdistance) {

      // turning right
      motion(180, 180, HIGH, LOW, LOW, LOW);
      delay(500);

      }

      motion(180, 180, HIGH, HIGH, LOW, LOW);
      delay(500);
    // Turned away from object
    // Stop 
    motion(0, 0, LOW, LOW, LOW, LOW);  
   

    int startingAngle = 160;
    // Turn sensor towards object
    myservo.write(startingAngle);
    delay(500);

   

    while (!foundLine()) {

      int d = getDistance();

      if (d < D0) {
        //going right
        motion(120, 120, HIGH, LOW, LOW, LOW);
      }
      else if (d > D0) {
        //going left
        motion(120, 120, LOW, LOW, HIGH, LOW);
      }
    
    }
    
  } else {
    lineFollowing();
  }
}
void loop() {
  // put your main code here, to run repeatedly:
  wallFollowing();
}
