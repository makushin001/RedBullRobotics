// Ultrasonic Sensor Pins
#define Trig_PIN 4    // Trigger pin for ultrasonic sensor
#define Echo_PIN 5    // Echo pin for ultrasonic sensor

void setup() {
  // Initialize serial communication for distance display
  Serial.begin(9600);
  
  // Setup ultrasonic sensor pins
  pinMode(Trig_PIN, OUTPUT);
  pinMode(Echo_PIN, INPUT);
}

void loop() {
  // Measure distance
  int distance = getDistance();
  
  // Print distance in centimeters
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");

  delay(500);  // Wait 500 ms before the next reading
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
