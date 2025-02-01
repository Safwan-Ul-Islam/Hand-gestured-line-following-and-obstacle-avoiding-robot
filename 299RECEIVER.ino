#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Servo.h>

// Motor control pins
int ENA = 3;
int ENB = 9;
int MotorA1 = 4;
int MotorA2 = 5;
int MotorB1 = 6;
int MotorB2 = 7;

// Ultrasonic sensor pins
const int trigPin = 12;
const int echoPin = 13;
long duration;
int distance;

// Servo motor pin
int servoPin = 11;
Servo Myservo;

// IR sensor pins for line following
const int IR_SENSOR_RIGHT = 22; // Replace with the actual pin
const int IR_SENSOR_LEFT = 23;  // Replace with the actual pin

// nRF24L01 setup
RF24 radio(8, 10); // CE and CSN pins for nRF24L01
const byte address[6] = "00001"; // Communication address

struct Data {
  int xAxis;
  int yAxis;
  int mode;  // 0: Gesture Control, 1: Obstacle Avoidance, 2: Line Following
};
Data receiveData;

void setup() {
  Serial.begin(9600);

  // Initialize nRF24L01
  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_MIN);
  radio.setDataRate(RF24_250KBPS);
  radio.startListening();

  // Motor pins setup
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(MotorA1, OUTPUT);
  pinMode(MotorA2, OUTPUT);
  pinMode(MotorB1, OUTPUT);
  pinMode(MotorB2, OUTPUT);

  // Ultrasonic sensor setup
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // IR sensor pins setup
  pinMode(IR_SENSOR_RIGHT, INPUT);
  pinMode(IR_SENSOR_LEFT, INPUT);

  // Servo setup
  Myservo.attach(servoPin);
  Myservo.write(90);  // Initialize servo to center position
}

void loop() {
  while (radio.available()) {
    radio.read(&receiveData, sizeof(Data));

    // Print the received mode
    Serial.print("Received Mode: ");
    Serial.println(receiveData.mode);

    if (receiveData.mode == 0) {
      Serial.println("Gesture Control Mode");
      gestureControl();
    } else if (receiveData.mode == 1) {
      Serial.println("Obstacle Avoidance Mode");
      obstacleAvoidance();
    } else if (receiveData.mode == 2) {
      Serial.println("Line Following Mode");
      lineFollowing();
    }
  }
}

void gestureControl() {
  if (abs(receiveData.xAxis) < 49 && abs(receiveData.yAxis) < 49) {
    stopMotors();
  } else if (receiveData.xAxis < -50) {  // Forward
    moveForward();
  } else if (receiveData.xAxis > 50) {  // Backward
    moveBackward();
  } else if (receiveData.yAxis < -50) {  // Left
    turnLeft();
  } else if (receiveData.yAxis > 50) {  // Right
    turnRight();
  }
}

void obstacleAvoidance() {
  // Ultrasonic sensor distance calculation
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = duration / 58.2;

  Serial.print("Distance: ");
  Serial.println(distance);

  if (distance > 15) {
    Serial.println("Path Clear - Moving Forward");
    Myservo.write(90);
    moveForward();
  } else if (distance <= 15 && distance > 0) {
    Serial.println("Obstacle Detected - Avoiding");
    stopMotors();
    delay(100);

    Myservo.write(0);  // Check left
    delay(500);
    Myservo.write(180);  // Check right
    delay(500);
    Myservo.write(90);  // Reset to center
    delay(500);

    moveBackward();  // Reverse
    delay(500);
    stopMotors();

    turnLeft();  // Turn left to avoid
    delay(500);
  }
}

void lineFollowing() {
  int rightIRSensorValue = digitalRead(IR_SENSOR_RIGHT); // Reading IR sensor values
  int leftIRSensorValue = digitalRead(IR_SENSOR_LEFT);

  // Print IR sensor values for debugging
  Serial.print("Left Sensor: ");
  Serial.print(leftIRSensorValue);
  Serial.print("  Right Sensor: ");
  Serial.println(rightIRSensorValue);

  if (leftIRSensorValue == LOW && rightIRSensorValue == LOW) {
    // Both sensors detect the line
    Serial.println("Both sensors on line - Moving forward");
    moveForward();
  } else if (leftIRSensorValue == LOW && rightIRSensorValue == HIGH) {
    // Left sensor detects the line, right does not
    Serial.println("Left sensor on line - Turning left");
    turnLeft();
  } else if (leftIRSensorValue == HIGH && rightIRSensorValue == LOW) {
    // Right sensor detects the line, left does not
    Serial.println("Right sensor on line - Turning right");
    turnRight();
  } else if (leftIRSensorValue == HIGH && rightIRSensorValue == HIGH) {
    // Both sensors do not detect the line (lost track)
    Serial.println("Lost line - Stopping");
    stopMotors();
  }
}

void controlMotors(int rightSpeed, int leftSpeed) {
  analogWrite(ENA, abs(rightSpeed));
  analogWrite(ENB, abs(leftSpeed));

  if (rightSpeed > 0) {
    digitalWrite(MotorA1, LOW);
    digitalWrite(MotorA2, HIGH);
  } else {
    digitalWrite(MotorA1, HIGH);
    digitalWrite(MotorA2, LOW);
  }

  if (leftSpeed > 0) {
    digitalWrite(MotorB1, LOW);
    digitalWrite(MotorB2, HIGH);
  } else {
    digitalWrite(MotorB1, HIGH);
    digitalWrite(MotorB2, LOW);
  }
}

void stopMotors() {
  digitalWrite(MotorA1, LOW);
  digitalWrite(MotorA2, LOW);
  digitalWrite(MotorB1, LOW);
  digitalWrite(MotorB2, LOW);
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}

void moveForward() {
  digitalWrite(MotorA1, LOW);
  digitalWrite(MotorA2, HIGH);
  digitalWrite(MotorB1, HIGH);
  digitalWrite(MotorB2, LOW);
  analogWrite(ENA, 100);  // Slower speed (80)
  analogWrite(ENB, 100);  // Slower speed (80)
}

void moveBackward() {
  digitalWrite(MotorA1, HIGH);
  digitalWrite(MotorA2, LOW);
  digitalWrite(MotorB1, LOW);
  digitalWrite(MotorB2, HIGH);
  analogWrite(ENA, 100);  // Slower speed (80)
  analogWrite(ENB, 100);  // Slower speed (80)
}

void turnLeft() {
  digitalWrite(MotorA1, HIGH);
  digitalWrite(MotorA2, LOW);
  digitalWrite(MotorB1, HIGH);
  digitalWrite(MotorB2, LOW);
  analogWrite(ENA, 100);  // Slower speed (80)
  analogWrite(ENB, 100);  // Slower speed (80)
}

void turnRight() {
  digitalWrite(MotorA1, LOW);
  digitalWrite(MotorA2, HIGH);
  digitalWrite(MotorB1, LOW);
  digitalWrite(MotorB2, HIGH);
  analogWrite(ENA, 100);  // Slower speed (80)
  analogWrite(ENB, 100);  // Slower speed (80)
}
